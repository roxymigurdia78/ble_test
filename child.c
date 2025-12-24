// child.c (回転対応: hall_bno2方式 / キャリブ復元対応 / CH3<->CH5 入替 / 他機能は維持)

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <string.h>
#include <time.h>
#include <errno.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <dirent.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

// BlueZ (BLE HCI)
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define CHILD_ADDR_ARG  argv[1]
#define PARENT_ADDR_ARG argv[2]

#define CHILD_MAP_FILE_PATH "mapping.txt"

// プロトコル時間定義
#define P_MAG_TIME_SEC   20
#define P_SCAN_TIME_SEC  10
#define C_MT_TIME_SEC    10
#define C_ACT_TIME_SEC   20
#define C_TX_TIME_SEC    10
#define COOLDOWN_SEC      5

#define ENV_CHILD_COUNT "TOTAL_CHILD_NODES"
static int g_total_child_nodes = 1;

static char g_my_addr[18] = "(unknown)";
static volatile int electromagnet_active    = 0;
static pthread_mutex_t mag_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int map_end_received = 0;
static volatile int g_scan_phase = 0; // 0=MT待ち, 1=MAP受信

// GPIO
const int BCM_GPIO_PIN = 20;

// ==========================================================
// 子機側マッピング保存構造体
// ==========================================================
#define CHILD_MAX_MAP  32
typedef struct {
    int  valid;
    int  key;
    char addr[18];
    int  x;
    int  y;
    int  z;
} ChildMapEntry;

static ChildMapEntry g_child_map[CHILD_MAX_MAP];

// ==========================================================
// BNO055/ADC
// ==========================================================
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7

#define DIFF_THRESHOLD        0.008
#define STABLE_COUNT_REQUIRED 10
#define BASELINE_SAMPLES      50

#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_OPR_MODE_ADDR   0x3D
#define BNO055_CALIB_STAT_ADDR 0x35

#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C
#define EULER_UNIT             16.0f

#define BNO055_CALIB_START     0x55
#define BNO055_CALIB_LEN       22

static const char *CALIB_FILE  = "bno055_calib.bin";
static const char *OFFSET_FILE = "bno055_heading_offset.txt";

static int   i2c_fd = -1;
static float g_heading_offset = 0.0f;

// 「確定瞬間にHeadingを読まない」ため、直前の安定Headingを保持
static float g_last_heading_north = NAN;

// レポートキュー
#define MAX_REPORT_QUEUE 20
typedef struct { char surface[16]; } ReportQueueEntry;
static ReportQueueEntry g_report_queue[MAX_REPORT_QUEUE];
static volatile int g_report_queue_count = 0;
static pthread_mutex_t report_queue_mutex = PTHREAD_MUTEX_INITIALIZER;

// ==========================================================
// プロトタイプ
// ==========================================================
static void msleep(int ms);
static int i2c_open(void);
static int bno055_init_ndof(void);
static int bno055_set_mode(uint8_t mode);
static int i2c_read_len(uint8_t reg, uint8_t *buf, int len);
static int i2c_write_len_reg(uint8_t reg, const uint8_t *data, int len);
static int bno055_restore_calibration_from_file(const char *path);
static void print_calib_status(void);

static int load_heading_offset(const char *path, float *offset);
static float read_bno055_heading_north(float heading_offset);

double read_adc_voltage(int ch);

int BLE_send_ready(const char *my_addr);
int BLE_send_surface_data(const char *my_addr, const char *parent_addr, const char *surface_name);

static void child_map_init(void);
static void expand_compact_addr12(const char *compact, char *out, size_t out_size);
static void child_store_map_frame(int key, const char *addr, int x, int y, int z);
static void child_dump_map_summary(const char *parent_addr);

int BLE_scan_for_targets(const char *target_addr, int timeout_ms, int phase);
int BLE_scan_for_MT(const char *target_addr, int timeout_sec);

static int gpio_init_and_on(void);
static void gpio_off_and_unexport(void);

void monitor_hall_sensor_and_report(const double *baseline, const char *my_addr, const char *parent_addr, int duration_sec);
void send_queued_reports(const char *my_addr, const char *parent_addr, int duration_sec);

// 回転マッピング（hall_bno2方式 + CH3<->CH5入替）
static const char *surface_from_channel_and_heading(int ch, float heading_north);

// ==========================================================
// GPIO
// ==========================================================
static int gpio_init_and_on(void) {
    pinMode(BCM_GPIO_PIN, OUTPUT);
    digitalWrite(BCM_GPIO_PIN, HIGH);
    printf("[GPIO] 子機電磁石 ON (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
    return 0;
}
static void gpio_off_and_unexport(void) {
    digitalWrite(BCM_GPIO_PIN, LOW);
    printf("[GPIO] 子機電磁石 OFF (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
}

// ==========================================================
// BNO055 / I2C
// ==========================================================
static void msleep(int ms) { usleep(ms * 1000); }

static int i2c_open(void) {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) { perror("open(/dev/i2c-1)"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        perror("ioctl(I2C_SLAVE)");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }
    return i2c_fd;
}

static int i2c_read_len(uint8_t reg, uint8_t *buf, int len) {
    if (write(i2c_fd, &reg, 1) != 1) { perror("i2c_read_len:write(reg)"); return -1; }
    if (read(i2c_fd, buf, len) != len) { perror("i2c_read_len:read"); return -1; }
    return 0;
}

static int i2c_write_len_reg(uint8_t reg, const uint8_t *data, int len) {
    uint8_t buf[1 + 64];
    if (len <= 0 || len > 64) return -1;
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    if (write(i2c_fd, buf, 1 + len) != (1 + len)) { perror("i2c_write_len_reg"); return -1; }
    return 0;
}

static int bno055_set_mode(uint8_t mode) {
    uint8_t buf[2] = { BNO055_OPR_MODE_ADDR, mode };
    if (write(i2c_fd, buf, 2) != 2) return -1;
    msleep(30);
    return 0;
}

static int bno055_init_ndof(void) {
    uint8_t id;
    if (i2c_read_len(BNO055_CHIP_ID_ADDR, &id, 1) < 0) return -1;
    if (id != BNO055_ID_EXPECTED) return -1;

    printf("BNO055 detected! ID=0x%02X\n", id);
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) return -1;
    msleep(50);
    printf("BNO055 initialized in NDOF mode.\n");
    return 0;
}

static void print_calib_status(void) {
    uint8_t v = 0;
    if (i2c_read_len(BNO055_CALIB_STAT_ADDR, &v, 1) < 0) {
        printf("CALIB status read failed.\n");
        return;
    }
    int sys = (v >> 6) & 0x03;
    int gyr = (v >> 4) & 0x03;
    int acc = (v >> 2) & 0x03;
    int mag = (v >> 0) & 0x03;
    printf("CALIB SYS:%d ACC:%d GYR:%d MAG:%d\n", sys, acc, gyr, mag);
}

static int bno055_restore_calibration_from_file(const char *path) {
    struct stat st;
    if (stat(path, &st) != 0) {
        printf("Calibration file not found: %s (skip restore)\n", path);
        return -1;
    }
    uint8_t calib[BNO055_CALIB_LEN];
    FILE *fp = fopen(path, "rb");
    if (!fp) { perror("fopen(calib)"); return -1; }
    size_t n = fread(calib, 1, BNO055_CALIB_LEN, fp);
    fclose(fp);
    if (n != BNO055_CALIB_LEN) {
        fprintf(stderr, "Calibration file size mismatch (read=%zu, need=%d)\n", n, BNO055_CALIB_LEN);
        return -1;
    }

    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0) return -1;
    msleep(50);

    if (i2c_write_len_reg(BNO055_CALIB_START, calib, BNO055_CALIB_LEN) < 0) {
        fprintf(stderr, "Failed to write calib to BNO055.\n");
        return -1;
    }
    msleep(30);

    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) return -1;
    msleep(50);

    printf("Calibration restored from %s\n", path);
    print_calib_status();
    return 0;
}

static int load_heading_offset(const char *path, float *offset) {
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    if (fscanf(fp, "%f", offset) != 1) { fclose(fp); return -1; }
    fclose(fp);
    return 0;
}

static float read_bno055_heading_north(float heading_offset) {
    uint8_t buf[2];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 2) < 0) return NAN;

    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
    float heading_raw = raw_heading / EULER_UNIT;

    float heading_north = heading_raw - heading_offset;
    while (heading_north < 0.0f) heading_north += 360.0f;
    while (heading_north >= 360.0f) heading_north -= 360.0f;
    return heading_north;
}

// hall_bno2方式 + CH3<->CH5入替
static const char *surface_from_channel_and_heading(int ch, float heading_north) {
    if (ch == 1) return "TOP";
    if (ch == 6) return "BOTTOM";

    int rotation_index = 0;
    if (!isnan(heading_north)) {
        if (heading_north >= 45.0f  && heading_north < 135.0f) rotation_index = 1;
        else if (heading_north >= 135.0f && heading_north < 225.0f) rotation_index = 2;
        else if (heading_north >= 225.0f && heading_north < 315.0f) rotation_index = 3;
        else rotation_index = 0;
    }

    // CH2=FRONT, CH3=RIGHT(★), CH4=BACK, CH5=LEFT(★)
    int base_index = -1; // 0:FRONT 1:LEFT 2:BACK 3:RIGHT
    if (ch == 2) base_index = 0;
    else if (ch == 3) base_index = 3; // swap
    else if (ch == 4) base_index = 2;
    else if (ch == 5) base_index = 1; // swap

    if (base_index < 0) return "UNKNOWN";
    int mapped_index = (base_index + rotation_index) % 4;
    static const char *names[4] = {"FRONT", "LEFT", "BACK", "RIGHT"};
    return names[mapped_index];
}

// ==========================================================
// ADC
// ==========================================================
double read_adc_voltage(int ch) {
    unsigned char data[3];
    data[0] = 1;
    data[1] = (8 + ch) << 4;
    data[2] = 0;
    wiringPiSPIDataRW(SPI_CH, data, 3);
    int value = ((data[1] & 3) << 8) | data[2];
    double voltage = (double)value * 3.3 / 1023.0;
    return voltage;
}

// ==========================================================
// 子機側マッピング保存
// ==========================================================
static void child_map_init(void) {
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        g_child_map[i].valid = 0;
        g_child_map[i].key = 0;
        g_child_map[i].addr[0] = '\0';
        g_child_map[i].x = g_child_map[i].y = g_child_map[i].z = 0;
    }
}

static void expand_compact_addr12(const char *compact, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    if (!compact) { snprintf(out, out_size, "UNKNOWN"); return; }
    size_t len = strlen(compact);
    if (len < 12) { snprintf(out, out_size, "%s", compact); return; }

    char buf[18];
    int j = 0;
    for (int i = 0; i < 12 && j < 17; i += 2) {
        buf[j++] = compact[i];
        if (j >= 17) break;
        buf[j++] = compact[i + 1];
        if (i != 10 && j < 17) buf[j++] = ':';
    }
    buf[j] = '\0';
    snprintf(out, out_size, "%s", buf);
}

static void child_store_map_frame(int key, const char *addr, int x, int y, int z) {
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid && g_child_map[i].key == key) {
            if (g_child_map[i].x == x && g_child_map[i].y == y && g_child_map[i].z == z) return;
            g_child_map[i].x = x; g_child_map[i].y = y; g_child_map[i].z = z;
            if (addr && addr[0] != '\0') { strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr)); g_child_map[i].addr[17] = '\0'; }
            printf("[MAP-RECV] key=%d addr=%s -> (%d,%d,%d) (UPDATED)\n",
                   key, g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN", x, y, z);
            return;
        }
    }
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (!g_child_map[i].valid) {
            g_child_map[i].valid = 1;
            g_child_map[i].key = key;
            g_child_map[i].x = x; g_child_map[i].y = y; g_child_map[i].z = z;
            if (addr && addr[0] != '\0') { strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr)); g_child_map[i].addr[17] = '\0'; }
            else g_child_map[i].addr[0] = '\0';
            printf("[MAP-RECV] key=%d addr=%s -> (%d,%d,%d)\n",
                   key, g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN", x, y, z);
            return;
        }
    }
    fprintf(stderr, "[WARN] Child map table full. Could not store key=%d\n", key);
}

static void child_dump_map_summary(const char *parent_addr) {
    printf("\n===== 受信したマッピング情報 (Child) =====\n");
    printf("Parent: %s\n", parent_addr);
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid) {
            printf("Key: %d, Addr: %s, Coords: (%d, %d, %d)\n",
                   g_child_map[i].key, g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                   g_child_map[i].x, g_child_map[i].y, g_child_map[i].z);
        }
    }
    printf("============================================\n");

    FILE *fp = fopen(CHILD_MAP_FILE_PATH, "w");
    if (!fp) return;
    fprintf(fp, "# Cube Mapping Log\n");
    fprintf(fp, "\n# Final Cube Coordinates\n");
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid) {
            fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n",
                    g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                    g_child_map[i].key, g_child_map[i].x, g_child_map[i].y, g_child_map[i].z);
        }
    }
    fprintf(fp, "\n# Raw Reports\n");
    fclose(fp);
}

// ==========================================================
// BLE送信（あなたの元コード維持）
// ==========================================================
static int setup_adv_parameters(int sock, uint16_t interval_ms) {
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(interval_ms * 1.6);
    adv_params_cp.min_interval = htobs(interval);
    adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype = 0x00;
    adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07;
    adv_params_cp.filter = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[BLE] Failed to set advertising parameters");
        return -1;
    }
    return 0;
}

int BLE_send_surface_data(const char *my_addr,
                          const char *parent_addr,
                          const char *surface_name)
{
    (void)parent_addr;
    printf("[COMM] Attempting BLE ADV send: Surface=%s\n", surface_name);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route"); return -1; }
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev"); return -1; }
    if (setup_adv_parameters(sock, 100) < 0) { close(sock); return -1; }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;

    adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;
    char name_field[31];
    snprintf(name_field, sizeof(name_field), "CubeNode|SURFACE:%s", surface_name);
    int name_len = (int)strlen(name_field); if (name_len > 29) name_len = 29;
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(&adv_data[len], name_field, name_len); len += name_len;

    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;
    adv_data_cp_struct.length = (uint8_t)len;
    memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
    memcpy(adv_data_cp_struct.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp_struct) < 0) {
        perror("[BLE] Failed to set advertising data");
        close(sock);
        return -1;
    }

    for (int i = 0; i < 3; i++) {
        uint8_t enable = 0x01;
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable) < 0) {
            perror("[BLE] Failed to enable advertising");
            close(sock);
            return -1;
        }
        usleep(150000);
        uint8_t disable = 0x00;
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable);
        usleep(15000);
    }

    close(sock);
    return 0;
}

int BLE_send_ready(const char *my_addr) {
    (void)my_addr;
    printf("[COMM] Sending READY advertise.\n");
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route (READY)"); return -1; }
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev (READY)"); return -1; }
    if (setup_adv_parameters(sock, 100) < 0) { close(sock); return -1; }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;

    adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;
    const char *name_field = "READY";
    int name_len = (int)strlen(name_field);
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(adv_data + len, name_field, name_len); len += name_len;

    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;
    adv_data_cp_struct.length = (uint8_t)len;
    memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
    memcpy(adv_data_cp_struct.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp_struct) < 0) {
        perror("[BLE] Failed to set advertising data (READY)");
        close(sock);
        return -1;
    }

    time_t start_time = time(NULL);
    uint8_t enable = 0x01, disable = 0x00;
    while (time(NULL) - start_time < 10) {
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
        usleep(100000);
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable);
        usleep(50000);
    }
    printf("[COMM] READY advertising finished.\n");
    close(sock);
    return 0;
}

// ==========================================================
// BLEスキャン（あなたの元コード維持）
// ==========================================================
int BLE_scan_for_targets(const char *target_addr, int timeout_ms, int phase) {
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) return 0;
    int sock = hci_open_dev(dev_id);
    if (sock < 0) return 0;

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01;
    scan_params_cp.interval = htobs(0x0010);
    scan_params_cp.window   = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00;
    scan_params_cp.filter = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);

    uint8_t enable = 0x01, filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    struct timeval tv = { .tv_sec  = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000 };
    int ret_val = 0;

    while (ret_val == 0) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);

        struct timeval current_tv = tv;
        int r = select(sock + 1, &fds, NULL, NULL, &current_tv);
        if (r <= 0) break;

        int len = read(sock, buf, sizeof(buf));
        if (len < 0) { if (errno == EINTR) continue; break; }
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;

        uint8_t *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;

            char name[128] = "";
            int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
                if (field_len == 0 || pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];
                if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
                    if (name_len > (int)sizeof(name)-1) name_len = (int)sizeof(name)-1;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                }
                pos += field_len + 1;
            }

            if (name[0] != '\0') {
                if (phase == 1) {
                    const char *p = NULL;
        if (name[0] == '#') p = name + 1;       // 新しい短縮形式
        else if (strncmp(name, "MK:", 3) == 0) p = name + 3; // 旧形式も一応残す
        else if (strncmp(name, "MAPK:", 5) == 0) p = name + 5;
                    if (p) {
                        char *endp;
                        long key = strtol(p, &endp, 10);
                        if (endp && *endp == ':') {
                            p = endp + 1;
                            char full_addr[18]; full_addr[0] = '\0';

                            const char *next_colon  = strchr(p, ':');
                            const char *first_comma = strchr(p, ',');
                            if (next_colon && (!first_comma || next_colon < first_comma)) {
                                char compact[13];
                                size_t addr_len = (size_t)(next_colon - p);
                                if (addr_len >= sizeof(compact)) addr_len = sizeof(compact) - 1;
                                memcpy(compact, p, addr_len);
                                compact[addr_len] = '\0';
                                expand_compact_addr12(compact, full_addr, sizeof(full_addr));
                                p = next_colon + 1;
                            } else {
                                snprintf(full_addr, sizeof(full_addr), "UNKNOWN");
                            }

                            long x = strtol(p, &endp, 10);
                            if (endp && *endp == ',') {
                                p = endp + 1;
                                long y = strtol(p, &endp, 10);
                                if (endp && *endp == ',') {
                                    p = endp + 1;
                                    long z = strtol(p, &endp, 10);
                                    child_store_map_frame((int)key, full_addr, (int)x, (int)y, (int)z);
                                }
                            }
                        }
                    }

                    if (strncmp(name, "MAP_END:", 8) == 0) {
                        printf("\n[SCAN] 親機からのマッピング送信完了通知を受信: %s\n", name);
                        ret_val = 2;
                        map_end_received = 1;
                        break;
                    }
                }
            }

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
        if (ret_val != 0) break;
    }

    uint8_t disable2 = 0x00;
    cmd[0] = disable2; cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
    close(sock);
    return ret_val;
}

int BLE_scan_for_MT(const char *target_addr, int timeout_sec) {
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) return 0;
    int sock = hci_open_dev(dev_id);
    if (sock < 0) return 0;

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01;
    scan_params_cp.interval = htobs(0x0010);
    scan_params_cp.window   = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00;
    scan_params_cp.filter = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);

    uint8_t enable = 0x01, filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    time_t start_time = time(NULL);
    int mt_found = 0;

    while (time(NULL) - start_time < timeout_sec) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) continue;

        int len = read(sock, buf, sizeof(buf));
        if (len < 0) { if (errno == EINTR) continue; break; }
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;

        uint8_t *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;

            char name[128] = "";
            int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
                if (field_len == 0 || pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];
                if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
                    if (name_len > 127) name_len = 127;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                }
                pos += field_len + 1;
            }

            if (strncmp(name, "MT:", 3) == 0 && strcmp(name + 3, target_addr) == 0) {
                mt_found = 1;
                printf("[INFO] MT notification received! Waiting for phase end...\n");
            }

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    uint8_t disable2 = 0x00;
    cmd[0] = disable2; cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
    close(sock);
    return mt_found;
}

// ==========================================================
// ホールセンサー検知（ここをhall_bno2の回転対応に統一）
// ==========================================================
void monitor_hall_sensor_and_report(const double *baseline, const char *my_addr, const char *parent_addr, int duration_sec) {
    printf("[SENSOR] ホールセンサー検知に専念 (%d秒)...\n", duration_sec);

    time_t start_time = time(NULL);
    static char last_surface[16] = "";
    static int  same_count = 0;

    const int MAX_REPORTS_PER_PHASE = 10;
    int reports_in_this_phase = 0;

    while (time(NULL) - start_time < duration_sec) {
        if (reports_in_this_phase >= MAX_REPORTS_PER_PHASE) {
            printf("[SENSOR] 検知上限 (%d回) に達しました。残り時間を待機します。\n", MAX_REPORTS_PER_PHASE);
            time_t elapsed_time = time(NULL) - start_time;
            if (duration_sec > elapsed_time) sleep((unsigned int)(duration_sec - elapsed_time));
            break;
        }

        // 先にHeadingを更新（確定瞬間に読まないため）
        float hn = read_bno055_heading_north(g_heading_offset);
        if (!isnan(hn)) g_last_heading_north = hn;

        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) window[ch] = read_adc_voltage(ch);

        double maxDiff = 0.0;
        int maxCh = -1;
        for (int ch = 1; ch < NUM_CH; ch++) {
            // hall_bno2同様: baseline<1.0V or window<1.0V は無視
            if (baseline[ch] < 1.0 || window[ch] < 1.0) continue;
            double diff = fabs(window[ch] - baseline[ch]);
            if (diff > maxDiff) { maxDiff = diff; maxCh = ch; }
        }

        int is_candidate = (maxCh != -1 && maxDiff >= DIFF_THRESHOLD);

        if (is_candidate) {
            const char *detected_surface = surface_from_channel_and_heading(maxCh, g_last_heading_north);

            char current_surface[16];
            strncpy(current_surface, detected_surface, sizeof(current_surface));
            current_surface[sizeof(current_surface)-1] = '\0';

            if (strcmp(last_surface, current_surface) == 0) same_count++;
            else { strncpy(last_surface, current_surface, sizeof(last_surface)); last_surface[sizeof(last_surface)-1] = '\0'; same_count = 1; }

            if (same_count >= STABLE_COUNT_REQUIRED) {
                printf("\n[EVENT CONFIRMED] CH%d confirmed: diff=%.3f V  HeadingNorth=%.1f°  Detected Surface: %s\n",
                       maxCh, maxDiff,
                       isnan(g_last_heading_north) ? -1.0f : g_last_heading_north,
                       current_surface);

                if (reports_in_this_phase < MAX_REPORTS_PER_PHASE &&
                    (duration_sec == C_ACT_TIME_SEC || duration_sec == P_MAG_TIME_SEC)) {

                    pthread_mutex_lock(&report_queue_mutex);
                    if (g_report_queue_count < MAX_REPORT_QUEUE) {
                        strncpy(g_report_queue[g_report_queue_count].surface, current_surface, 16);
                        g_report_queue[g_report_queue_count].surface[15] = '\0';
                        g_report_queue_count++;
                        reports_in_this_phase++;
                        printf("[REPORT QUEUED] Surface %s (Total: %d)\n", current_surface, g_report_queue_count);
                    }
                    pthread_mutex_unlock(&report_queue_mutex);
                } else {
                    printf("[EVENT] Detected during Passive Phase: %s\n", current_surface);
                }

                same_count = 0;
                last_surface[0] = '\0';
                usleep(500000);
            }
        }

        usleep(10000);
    }
}

// ==========================================================
// レポート送信（元ロジック維持）
// ==========================================================
void send_queued_reports(const char *my_addr, const char *parent_addr, int duration_sec) {
    time_t start_time = time(NULL);
    printf("[COMM] C-TX Phase: Sending %d queued reports for %d seconds...\n", g_report_queue_count, duration_sec);

    while (time(NULL) - start_time < duration_sec) {
        pthread_mutex_lock(&report_queue_mutex);

        if (g_report_queue_count > 0) {
            char surface_to_send[16];
            strncpy(surface_to_send, g_report_queue[0].surface, 16);
            surface_to_send[15] = '\0';

            printf("[COMM] Sending queued report: %s\n", surface_to_send);
            pthread_mutex_unlock(&report_queue_mutex);

            BLE_send_surface_data(my_addr, parent_addr, surface_to_send);

            pthread_mutex_lock(&report_queue_mutex);
            for (int i = 0; i < g_report_queue_count - 1; i++) g_report_queue[i] = g_report_queue[i + 1];
            g_report_queue_count--;
            pthread_mutex_unlock(&report_queue_mutex);

            if (g_report_queue_count == 0) {
                time_t elapsed_time = time(NULL) - start_time;
                if (duration_sec > elapsed_time) {
                    printf("[COMM] All reports sent. Waiting %ld seconds for C-TX end.\n", duration_sec - elapsed_time);
                    sleep((unsigned int)(duration_sec - elapsed_time));
                }
                break;
            }
        } else {
            pthread_mutex_unlock(&report_queue_mutex);
            time_t elapsed_time = time(NULL) - start_time;
            if (duration_sec > elapsed_time) {
                printf("[COMM] No reports in queue. Waiting %ld seconds for C-TX end.\n", duration_sec - elapsed_time);
                sleep((unsigned int)(duration_sec - elapsed_time));
            }
            break;
        }
    }
}

// ==========================================================
// main（キャリブ復元＋heading_offset読込を追加）
// ==========================================================
int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc < 3) {
        fprintf(stderr, "Usage: ./child <my_full_address> <parent_full_address>\n");
        return 1;
    }

    const char *my_addr     = CHILD_ADDR_ARG;
    const char *parent_addr = PARENT_ADDR_ARG;

    strncpy(g_my_addr, my_addr, sizeof(g_my_addr));
    g_my_addr[sizeof(g_my_addr) - 1] = '\0';

    printf("\n==================================\n");
    printf("🌱 CHILD PROGRAM STARTING\n");
    printf("==================================\n");
    printf("My full address (Child): **%s**\n", my_addr);
    printf("Confirmed Parent Address: **%s**\n", parent_addr);

    if (wiringPiSetupGpio() != 0) {
        fprintf(stderr, "ERROR: wiringPiSetupGpio failed.\n");
        return 1;
    }
    printf("[INFO] WiringPi BCM Setup successful.\n");

    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) {
        fprintf(stderr, "FATAL ERROR: wiringPiSPISetup failed.\n");
        return 2;
    }

    i2c_fd = i2c_open();
    if (i2c_fd < 0) return 3;
    if (bno055_init_ndof() < 0) return 3;

    // ★キャリブ復元（hall_bno2と同じ）
    bno055_restore_calibration_from_file(CALIB_FILE);

    // ★heading_offset 読み込み（※元コードの「ローカル変数に読んで使ってない」バグを解消）
    if (load_heading_offset(OFFSET_FILE, &g_heading_offset) != 0) {
        fprintf(stderr, "ERROR: Heading offset file not loaded. Aborting.\n");
        close(i2c_fd);
        return 1;
    }
    printf("Heading offset loaded: %.2f deg (using %s)\n", g_heading_offset, OFFSET_FILE);

    double baseline[NUM_CH] = {0};
    printf("Baseline sampling...\n");
    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) baseline[ch] += read_adc_voltage(ch);
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) baseline[ch] /= BASELINE_SAMPLES;
    printf("Baseline established.\n");

    if (BLE_send_ready(my_addr) < 0) {
        fprintf(stderr, "Failed to send READY advertising.\n");
        close(i2c_fd);
        return 1;
    }

    char *count_str = getenv(ENV_CHILD_COUNT);
    if (count_str) {
        g_total_child_nodes = atoi(count_str);
        if (g_total_child_nodes <= 0) g_total_child_nodes = 1;
    }
    printf("[CONFIG] Expected total child nodes (Cycles): %d\n", g_total_child_nodes);

    child_map_init();
    printf("Begin continuous monitoring...\n");
    g_scan_phase = 0;

    int node_cycle_count = 0;

    // 初期フェーズ: P-MAG ON
    printf("\n[PHASE START] 初期センサー検知専念 (P-MAG ON) (%d秒)\n", P_MAG_TIME_SEC);
    monitor_hall_sensor_and_report(baseline, my_addr, parent_addr, P_MAG_TIME_SEC);

    printf("[PHASE] クールダウン (%d秒)\n", COOLDOWN_SEC);
    sleep((unsigned int)COOLDOWN_SEC);

    // P-SCAN: レポート送信
    printf("[PHASE START] P-SCAN: レポート送信専念 (%d秒)\n", P_SCAN_TIME_SEC);
    send_queued_reports(my_addr, parent_addr, P_SCAN_TIME_SEC);

    while (node_cycle_count < g_total_child_nodes) {
        printf("\n[PHASE START] C-MT: MT通知監視専念 (%d秒) [Cycle %d/%d]\n",
               C_MT_TIME_SEC, node_cycle_count + 1, g_total_child_nodes);
        int mt_received = BLE_scan_for_MT(g_my_addr, C_MT_TIME_SEC);

        printf("[PHASE START] C-ACT: 駆動 or センサー専念 (%d秒)\n", C_ACT_TIME_SEC);
        if (mt_received) {
            printf("[MAG] MT受信確認。電磁石をすぐにONにします...\n");

            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 1;
            pthread_mutex_unlock(&mag_state_mutex);

            printf("[MAG] 電磁石 ON (BCM %d) (%d秒)\n", BCM_GPIO_PIN, C_ACT_TIME_SEC);
            if (gpio_init_and_on() == 0) {
                sleep((unsigned int)C_ACT_TIME_SEC);
                gpio_off_and_unexport();
            }

            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 0;
            pthread_mutex_unlock(&mag_state_mutex);

            printf("[SENSOR] 駆動後、磁気安定のため3秒待機...\n");
            sleep(3);
        } else {
            monitor_hall_sensor_and_report(baseline, my_addr, parent_addr, C_ACT_TIME_SEC);
        }

        printf("[PHASE START] C-TX: レポート送信専念 (%d秒)\n", C_TX_TIME_SEC);
        send_queued_reports(my_addr, parent_addr, C_TX_TIME_SEC);

        printf("[PHASE] クールダウン (%d秒)\n", COOLDOWN_SEC);
        sleep((unsigned int)COOLDOWN_SEC);

        g_scan_phase = 0;
        node_cycle_count++;
    }

    printf("\n[SEQUENCE END] 全ノードのMTサイクルが完了しました。MAP配布受信フェーズへ移行します。\n");
    printf("\n[PHASE START] MAP 配布受信モード (MK/MAP_ENDを受信)\n");
    g_scan_phase = 1;

    while (!map_end_received) {
        BLE_scan_for_targets(g_my_addr, 2000, g_scan_phase);
        usleep(50000);
    }

    printf("[SYNC] MAP_END受信。終了タイミングを合わせるため5秒待機します...\n");
    sleep(5);

    printf("\n\n===== マッピング終了通知(MAP_END)を受信しました。子機プログラムを終了します。 =====\n");
    child_dump_map_summary(parent_addr);

    if (i2c_fd != -1) close(i2c_fd);
    gpio_off_and_unexport();

    printf("CHILD PROGRAM END\n");
    return 0;
}
