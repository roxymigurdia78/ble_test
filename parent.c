// parent.c (回転対応: hall_bno2方式 / キャリブ復元対応 / CH3<->CH5 入替 / 他機能は維持)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <math.h>
#include <stdint.h>

// ファイルI/O, I2C制御に必要なヘッダー
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>

// BLE HCI
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

// センサーI/O
#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

#define PARENT_ADDR_ARG argv[1]
#define LOG_FILE_PATH   "parent_reception_log.txt"
#define MAP_FILE_PATH   "mapping.txt"
#define KEY_LIST_FILE   "parent_key_list.txt"
#define MAX_DATA_LEN    64
#define MAX_NODES       32

// プロトコル時間定義
#define P_MAG_TIME_SEC   20
#define P_SCAN_TIME_SEC  10
#define C_MT_TIME_SEC    10
#define C_ACT_TIME_SEC   20
#define C_TX_TIME_SEC    10
#define COOLDOWN_SEC      5

// ==========================================================
// センサー関連 定義とグローバル変数
// ==========================================================
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7

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

#define DIFF_THRESHOLD        0.008
#define STABLE_COUNT_REQUIRED 10
#define BASELINE_SAMPLES      50

static int   i2c_fd = -1;
static float g_heading_offset = 0.0f;
static double g_baseline[NUM_CH] = {0};

static volatile int g_parent_mag_active = 0;
static volatile int g_sensor_phase = 0; // 0=受信/報告待ち, 1=検知専念

// 「確定瞬間にHeadingを読まない」ため、直前の安定Headingを保持
static float g_last_heading_north = NAN;

// 親機電磁石
const int BCM_GPIO_PIN = 20;

// ==========================================================
// キーリスト/状態管理 構造体
// ==========================================================
typedef struct {
    int  key;
    char addr[18];
    int x, y, z;
    int index;
} KeyEntry;

static KeyEntry g_nodes[MAX_NODES];
static int g_node_count = 0;

static char g_parent_addr[18] = "";
static pthread_mutex_t mag_mutex = PTHREAD_MUTEX_INITIALIZER;
static char current_mag_target[18] = "";

static pthread_mutex_t ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  ready_cond  = PTHREAD_COND_INITIALIZER;
static int g_child_ready[MAX_NODES] = {0};
static int g_ready_count = 0;
static int g_total_children = 0;

typedef struct {
    char target_addr[18];
    char detected_addr[18];
    char surface[16];
} ReportEntry;

#define MAX_REPORTS 1024
static ReportEntry g_reports[MAX_REPORTS];
static int g_report_count = 0;
static pthread_mutex_t report_mutex = PTHREAD_MUTEX_INITIALIZER;

static int g_report_log_printed[MAX_NODES] = {0};

// ==========================================================
// プロトタイプ宣言
// ==========================================================
void *parent_sensor_monitor(void *arg);
void *BLE_receive_data_server(void *arg);
int  load_key_list(const char *path);
void print_key_list(void);
void *mag_sequence_thread(void *arg);
int  coordinate_mapping(void);
int  BLE_send_map_result(int map_result);

static int  i2c_open(void);
static int  bno055_init_ndof(void);
static int  bno055_set_mode(uint8_t mode);
static int  i2c_write8(uint8_t reg, uint8_t value);
static int  i2c_read_len(uint8_t reg, uint8_t *buf, int len);
static int  i2c_write_len_reg(uint8_t reg, const uint8_t *data, int len);

static int  load_heading_offset(const char *path, float *offset);
static float read_bno055_heading_north(float heading_offset);
static void print_calib_status(void);
static int  bno055_restore_calibration_from_file(const char *path);

double read_adc_voltage(int ch);

void register_child_ready(const char *child_addr);
void process_received_data(const char *received_data);

static int readInt(const char *path);
static int gpio_init_and_on(void);
static void gpio_off_and_unexport(void);
static void compress_addr_to_hex12(const char *addr, char *out, size_t out_size);
static void msleep(int ms);

// 回転マッピング（hall_bno2方式 + CH3<->CH5入替）
static const char *surface_from_channel_and_heading(int ch, float heading_north);

// ==========================================================
// ヘルパー
// ==========================================================
static void msleep(int ms) { usleep(ms * 1000); }

static int readInt(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int v;
    if (fscanf(f, "%d", &v) != 1) { fclose(f); return -1; }
    fclose(f);
    return v;
}
static int fileExists(const char *path) { struct stat st; return (stat(path, &st) == 0); }

static int gpio_init_and_on(void) {
    pinMode(BCM_GPIO_PIN, OUTPUT);
    digitalWrite(BCM_GPIO_PIN, HIGH);
    printf("[GPIO] Parent Electromagnet ON (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
    return 0;
}
static void gpio_off_and_unexport(void) {
    digitalWrite(BCM_GPIO_PIN, LOW);
    printf("[GPIO] Parent Electromagnet OFF (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
}

// ==========================================================
// I2C / BNO055
// ==========================================================
static int i2c_open(void) {
    int fd = open(I2C_DEV_PATH, O_RDWR);
    if (fd < 0) { perror("open(/dev/i2c-1)"); return -1; }
    if (ioctl(fd, I2C_SLAVE, BNO055_ADDRESS) < 0) { perror("ioctl(I2C_SLAVE)"); close(fd); return -1; }
    return fd;
}

static int i2c_write8(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(i2c_fd, buf, 2) != 2) { perror("i2c_write8"); return -1; }
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

static int i2c_read_len(uint8_t reg, uint8_t *buf, int len) {
    if (write(i2c_fd, &reg, 1) != 1) { perror("i2c_read_len:write(reg)"); return -1; }
    if (read(i2c_fd, buf, len) != len) { perror("i2c_read_len:read"); return -1; }
    return 0;
}

static int bno055_set_mode(uint8_t mode) {
    if (i2c_write8(BNO055_OPR_MODE_ADDR, mode) < 0) return -1;
    msleep(30);
    return 0;
}

static int bno055_init_ndof(void) {
    uint8_t id;
    if (i2c_read_len(BNO055_CHIP_ID_ADDR, &id, 1) < 0) { fprintf(stderr, "Failed to read chip ID\n"); return -1; }
    if (id != BNO055_ID_EXPECTED) { fprintf(stderr, "Unexpected BNO055 ID: 0x%02X\n", id); return -1; }
    printf("BNO055 detected! ID=0x%02X\n", id);

    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) { fprintf(stderr, "Failed to set NDOF mode\n"); return -1; }
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
    if (!fileExists(path)) {
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

    // CONFIGモードで書き込み
    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0) return -1;
    msleep(50);

    if (i2c_write_len_reg(BNO055_CALIB_START, calib, BNO055_CALIB_LEN) < 0) {
        fprintf(stderr, "Failed to write calib to BNO055.\n");
        return -1;
    }
    msleep(30);

    // NDOFへ戻す
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

// hall_bno2方式: 45度境界で0/90/180/270へ割当（四象限）
// + CH3<->CH5入替: CH3=RIGHT, CH5=LEFT
static const char *surface_from_channel_and_heading(int ch, float heading_north) {
    if (ch == 1) return "TOP";
    if (ch == 6) return "BOTTOM";

    // 回転インデックス (0..3) を HeadingNorth から決める
    int rotation_index = 0;
    if (!isnan(heading_north)) {
        // 45度境界で四象限
        if (heading_north >= 45.0f  && heading_north < 135.0f) rotation_index = 1;
        else if (heading_north >= 135.0f && heading_north < 225.0f) rotation_index = 2;
        else if (heading_north >= 225.0f && heading_north < 315.0f) rotation_index = 3;
        else rotation_index = 0;
    }

    // 基本割当（rotation_index=0のときの面）
    // CH2=FRONT, CH3=RIGHT(★), CH4=BACK, CH5=LEFT(★)
    int base_index = -1; // 0:FRONT 1:LEFT 2:BACK 3:RIGHT
    if (ch == 2) base_index = 0;         // FRONT
    else if (ch == 3) base_index = 3;    // RIGHT  (swap)
    else if (ch == 4) base_index = 2;    // BACK
    else if (ch == 5) base_index = 1;    // LEFT   (swap)

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
// Key utilities
// ==========================================================
int get_key_by_addr(const char *addr) {
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, addr) == 0) return g_nodes[i].key;
    }
    return -1;
}
int get_index_by_addr(const char *addr) {
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, addr) == 0) return i;
    }
    return -1;
}

// ==========================================================
// BLE受信処理
// ==========================================================
void process_received_data(const char *received_data) {
    char data_copy[MAX_DATA_LEN];
    strncpy(data_copy, received_data, MAX_DATA_LEN);
    data_copy[MAX_DATA_LEN - 1] = '\0';

    char *child_addr = strtok(data_copy, ",");
    char *surface    = strtok(NULL, ",");

    if (child_addr && surface) {
        int child_key = get_key_by_addr(child_addr);
        int child_index = get_index_by_addr(child_addr);

        if (child_index != -1 && g_report_log_printed[child_index] == 0) {
            printf("Surface-report from %s (Key: %d) => %s (Target: %s)\n",
                   child_addr, child_key, surface,
                   current_mag_target[0] != '\0' ? current_mag_target : "None");
            g_report_log_printed[child_index] = 1;
        }

        pthread_mutex_lock(&report_mutex);
        if (g_report_count < MAX_REPORTS && current_mag_target[0] != '\0') {
            strncpy(g_reports[g_report_count].target_addr,   current_mag_target, 18);
            strncpy(g_reports[g_report_count].detected_addr, child_addr,        18);
            strncpy(g_reports[g_report_count].surface,       surface,           16);
            g_reports[g_report_count].target_addr[17]   = '\0';
            g_reports[g_report_count].detected_addr[17] = '\0';
            g_reports[g_report_count].surface[15]       = '\0';
            g_report_count++;
        }
        pthread_mutex_unlock(&report_mutex);
    }
}

void register_child_ready(const char *child_addr) {
    pthread_mutex_lock(&ready_mutex);
    if (g_total_children <= 0) { pthread_mutex_unlock(&ready_mutex); return; }
    if (g_parent_addr[0] != '\0' && strcmp(child_addr, g_parent_addr) == 0) { pthread_mutex_unlock(&ready_mutex); return; }

    int idx = get_index_by_addr(child_addr);
    if (idx < 0 || idx >= MAX_NODES) { pthread_mutex_unlock(&ready_mutex); return; }

    if (!g_child_ready[idx]) {
        g_child_ready[idx] = 1;
        g_ready_count++;
        printf("送ってきた個体のアドレス：%s 準備OK！（%d/%d）\n", child_addr, g_ready_count, g_total_children);
        if (g_ready_count >= g_total_children) pthread_cond_broadcast(&ready_cond);
    }
    pthread_mutex_unlock(&ready_mutex);
}

// ==========================================================
// 親機のホールセンサー常時監視スレッド
// ここを hall_bno2 方式の回転対応に統一
// ==========================================================
void *parent_sensor_monitor(void *arg) {
    (void)arg;
    printf("[SENSOR] 親機センサー監視スレッド開始。\n");

    static char last_surface[16] = "";
    static int same_count = 0;

    while (1) {
        if (g_sensor_phase != 1) { usleep(100000); continue; }
        if (g_parent_mag_active)  { usleep(100000); continue; }

        // 先にHeadingを更新（確定瞬間に読まないため）
        float hn = read_bno055_heading_north(g_heading_offset);
        if (!isnan(hn)) g_last_heading_north = hn;

        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) window[ch] = read_adc_voltage(ch);

        double maxDiff = 0.0;
        int maxCh = -1;
        for (int ch = 1; ch < NUM_CH; ch++) {
            if (g_baseline[ch] < 1.0 || window[ch] < 1.0) continue; // hall_bno2同様
            double diff = fabs(window[ch] - g_baseline[ch]);
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

                same_count = 0;
                last_surface[0] = '\0';
            }
        }

        usleep(10000);
    }
    return NULL;
}

// ==========================================================
// BLE 受信スレッド
// ==========================================================
void *BLE_receive_data_server(void *arg) {
    (void)arg;
    printf("[COMM] BLE親機サーバー起動。子機データ受信待機中...\n");

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("hci_get_route"); return NULL; }
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("hci_open_dev"); return NULL; }

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
    while (1) {
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
            char addr[18];
            ba2str(&info->bdaddr, addr);

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

            if (name[0] != '\0') {
                char *p = strstr(name, "SURFACE:");
                if (p) {
                    p += strlen("SURFACE:");
                    char surface[32];
                    int si = 0;
                    while (*p != '\0' && *p != '|' && si < 31) surface[si++] = *p++;
                    surface[si] = '\0';

                    char combined[MAX_DATA_LEN];
                    snprintf(combined, sizeof(combined), "%s,%s", addr, surface);
                    process_received_data(combined);
                }
                if (strncmp(name, "READY", 5) == 0) register_child_ready(addr);
            }

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    close(sock);
    return NULL;
}

// ==========================================================
// KeyEntry ソート比較関数
// ==========================================================
static int compare_key_entries(const void *a, const void *b) {
    const KeyEntry *entry_a = (const KeyEntry *)a;
    const KeyEntry *entry_b = (const KeyEntry *)b;
    return entry_a->key - entry_b->key;
}

// ==========================================================
// load_key_list / print
// ==========================================================
int load_key_list(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) { perror("キーリスト読み込み失敗"); return -1; }

    g_node_count = 0;
    while (g_node_count < MAX_NODES) {
        int key;
        char addr[18];
        if (fscanf(fp, "%d %17s", &key, addr) != 2) break;
        g_nodes[g_node_count].key = key;
        strncpy(g_nodes[g_node_count].addr, addr, 18);
        g_nodes[g_node_count].x = g_nodes[g_node_count].y = g_nodes[g_node_count].z = -1;
        g_nodes[g_node_count].index = g_node_count;
        g_node_count++;
    }
    fclose(fp);
    return g_node_count;
}

void print_key_list(void) {
    printf("\n--- Parent Key List ---\n");
    for (int i = 0; i < g_node_count; i++) {
        printf("  [%d] key=%d addr=%s\n", i, g_nodes[i].key, g_nodes[i].addr);
    }
    printf("-----------------------\n");
}

// ==========================================================
// mag_sequence_thread（元ロジック維持）
// ==========================================================
void *mag_sequence_thread(void *arg) {
    (void)arg;
    if (g_node_count <= 0) return NULL;

    qsort(g_nodes, g_node_count, sizeof(KeyEntry), compare_key_entries);
    printf("[MAG] Key list sorted by Key value (Ascending).\n");

    if (g_total_children > 0) {
        printf("[MAG] %d 台の子機のREADYを待機...\n", g_total_children);
        pthread_mutex_lock(&ready_mutex);
        while (g_ready_count < g_total_children) pthread_cond_wait(&ready_cond, &ready_mutex);
        pthread_mutex_unlock(&ready_mutex);
        printf("[MAG] 全子機準備完了。10秒待機してからシーケンスを開始します。\n");
        sleep(10);
        printf("[MAG] 待機完了。シーケンス開始。\n");
    }

    int parent_idx = -1;
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, g_parent_addr) == 0) parent_idx = i;
    }

    int dev_id = hci_get_route(NULL); if (dev_id < 0) return NULL;
    int sock_adv = hci_open_dev(dev_id); if (sock_adv < 0) return NULL;

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(500 * 1.6);
    adv_params_cp.min_interval = htobs(interval);
    adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype = 0x00;
    adv_params_cp.chan_map = 0x07;
    hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    // (1) 初期フェーズ: 親機電磁石
    printf("\n[PHASE START] 初期フェーズ: 親機電磁石駆動 (P-MAG ON)\n");
    pthread_mutex_lock(&mag_mutex);
    strncpy(current_mag_target, g_parent_addr, 18);
    g_parent_mag_active = 1;
    g_sensor_phase = 1;
    pthread_mutex_unlock(&mag_mutex);

    if (gpio_init_and_on() == 0) {
        printf("[MAG] 親機電磁石 ON (%d秒)\n", P_MAG_TIME_SEC);
        sleep((unsigned int)P_MAG_TIME_SEC);
    }
    gpio_off_and_unexport();

    printf("[MAG] 親機駆動終了。クールダウン(%d秒)。\n", COOLDOWN_SEC);
    g_parent_mag_active = 0;
    g_sensor_phase = 0;
    sleep((unsigned int)COOLDOWN_SEC);

    memset(g_report_log_printed, 0, sizeof(g_report_log_printed));

    printf("[PHASE START] 親機レポート受信 (P-SCAN) (%d秒)\n", P_SCAN_TIME_SEC);
    sleep((unsigned int)P_SCAN_TIME_SEC);

    // (2) 各ノード
    for (int idx = 0; idx < g_node_count; idx++) {
        const char *target_addr = g_nodes[idx].addr;
        int key = g_nodes[idx].key;
        if (idx == parent_idx) continue;

        pthread_mutex_lock(&mag_mutex);
        strncpy(current_mag_target, target_addr, 18);
        pthread_mutex_unlock(&mag_mutex);

        // A. C-MT
        printf("\n[PHASE START] 子機(Key=%d) MT通知フェーズ (C-MT) (%d秒)\n", key, C_MT_TIME_SEC);
        uint8_t adv_data[31];
        memset(adv_data, 0, sizeof(adv_data));
        int len = 0;

        adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

        char name_field[32];
        snprintf(name_field, sizeof(name_field), "MT:%s", target_addr);
        int name_len = (int)strlen(name_field); if (name_len > 26) name_len = 26;
        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09;
        memcpy(&adv_data[len], name_field, name_len); len += name_len;

        struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_cp;
        adv_cp.length = (uint8_t)len;
        memcpy(adv_cp.data, adv_data, len);

        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_cp);
        uint8_t enable = 0x01;
        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

        sleep((unsigned int)C_MT_TIME_SEC);

        uint8_t disable = 0x00;
        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable);
        printf("[MAG] MT通知 OFF。\n");

        memset(g_report_log_printed, 0, sizeof(g_report_log_printed));

        // B. C-ACT: 親機センサー検知専念
        printf("[PHASE START] 親機センサー検知フェーズ (C-ACT) (%d秒)\n", C_ACT_TIME_SEC);
        g_sensor_phase = 1;
        sleep((unsigned int)C_ACT_TIME_SEC);
        g_sensor_phase = 0;

        // C. C-TX
        printf("[PHASE START] 子機レポート受信 (C-TX) (%d秒)\n", C_TX_TIME_SEC);
        sleep((unsigned int)C_TX_TIME_SEC);

        // D. cooldown
        printf("[MAG] クールダウン(%d秒)。次のターゲットへ移行。\n", COOLDOWN_SEC);
        pthread_mutex_lock(&mag_mutex);
        current_mag_target[0] = '\0';
        pthread_mutex_unlock(&mag_mutex);
        sleep((unsigned int)COOLDOWN_SEC);
    }

    pthread_mutex_lock(&mag_mutex);
    current_mag_target[0] = '\0';
    g_sensor_phase = 0;
    pthread_mutex_unlock(&mag_mutex);

    close(sock_adv);
    printf("[MAG] 全シーケンス完了。\n");
    return NULL;
}

// ==========================================================
// parent.c 修正用コード (X=-1 対応版)
// ==========================================================

// ★未確定を表す値を -1 から -999 に変更 (重要！)
#define UNMAPPED_VAL -999

// 指定した座標にすでに確定したノードがいるかチェック
static int is_occupied(int x, int y, int z) {
    for (int i = 0; i < g_node_count; i++) {
        // ★修正: UNMAPPED_VAL (-999) 以外なら「誰かいる」とみなす
        if (g_nodes[i].x == UNMAPPED_VAL) continue;
        
        if (g_nodes[i].x == x && g_nodes[i].y == y && g_nodes[i].z == z) {
            return 1; // 誰かいる (Occupied)
        }
    }
    return 0; // 空いている (Empty)
}

int coordinate_mapping(void) {
    printf("\n\n===== マッピング処理開始 (初期値-999版) =====\n");

    pthread_mutex_lock(&report_mutex);
    printf("[MAP INFO] Total reports collected: %d\n", g_report_count);
    pthread_mutex_unlock(&report_mutex);

    FILE *fp = fopen(MAP_FILE_PATH, "w");
    if (!fp) { perror("Map file error"); return 1; }
    fprintf(fp, "# Cube Mapping Log\n");

    // 1. 初期化
    int parent_index = -1;
    for (int i = 0; i < g_node_count; i++) {
        // ★修正: 初期値を -999 に設定
        g_nodes[i].x = UNMAPPED_VAL; 
        g_nodes[i].y = UNMAPPED_VAL; 
        g_nodes[i].z = UNMAPPED_VAL;
        
        if (strcmp(g_nodes[i].addr, g_parent_addr) == 0) {
            g_nodes[i].x = 0; g_nodes[i].y = 0; g_nodes[i].z = 0;
            parent_index = i;
        }
    }

    if (parent_index == -1) {
        fprintf(stderr, "[MAP] 親機情報なし。失敗。\n");
        fclose(fp);
        return 1;
    }

    // 2. 反復マッピング
    int mapped_count = 1;
    int loop_limit = g_node_count * 3; // 少し多めに回す
    int changed = 1;

    while (changed && loop_limit > 0) {
        changed = 0;
        loop_limit--;

        pthread_mutex_lock(&report_mutex);
        for (int r = 0; r < g_report_count; r++) {
            const char *target_addr   = g_reports[r].target_addr;
            const char *detected_addr = g_reports[r].detected_addr;
            const char *surf          = g_reports[r].surface;

            int src_idx = get_index_by_addr(target_addr);
            int det_idx = get_index_by_addr(detected_addr);

            if (src_idx == -1 || det_idx == -1) continue;

            // ★修正: UNMAPPED_VAL で判定
            int src_known = (g_nodes[src_idx].x != UNMAPPED_VAL);
            int det_known = (g_nodes[det_idx].x != UNMAPPED_VAL);

            // 両方確定済みならスキップ
            if (src_known && det_known) continue;
            // 両方不明なら計算できないのでスキップ
            if (!src_known && !det_known) continue;

            int nx = 0, ny = 0, nz = 0;
            int candidate_idx = -1;

            // ケースA: 発信源(Known) -> 受信者(Unknown)
            if (src_known) {
                nx = g_nodes[src_idx].x;
                ny = g_nodes[src_idx].y;
                nz = g_nodes[src_idx].z;
                
                if      (strcmp(surf, "LEFT") == 0)   nx += 1;
                else if (strcmp(surf, "RIGHT") == 0)  nx -= 1;
                else if (strcmp(surf, "FRONT") == 0)  ny -= 1;
                else if (strcmp(surf, "BACK") == 0)   ny += 1;
                else if (strcmp(surf, "TOP") == 0)    nz -= 1;
                else if (strcmp(surf, "BOTTOM") == 0) nz += 1;
                
                candidate_idx = det_idx;
            }
            // ケースB: 発信源(Unknown) -> 受信者(Known) (逆算)
            else if (det_known) {
                nx = g_nodes[det_idx].x;
                ny = g_nodes[det_idx].y;
                nz = g_nodes[det_idx].z;

                if      (strcmp(surf, "LEFT") == 0)   nx -= 1;
                else if (strcmp(surf, "RIGHT") == 0)  nx += 1;
                else if (strcmp(surf, "FRONT") == 0)  ny += 1;
                else if (strcmp(surf, "BACK") == 0)   ny -= 1;
                else if (strcmp(surf, "TOP") == 0)    nz += 1;
                else if (strcmp(surf, "BOTTOM") == 0) nz -= 1;

                candidate_idx = src_idx;
            }

            // 排他制御: 計算した場所に誰もいなければ採用
            if (!is_occupied(nx, ny, nz)) {
                g_nodes[candidate_idx].x = nx;
                g_nodes[candidate_idx].y = ny;
                g_nodes[candidate_idx].z = nz;
                mapped_count++;
                changed = 1;
                
                printf("[MAP] Mapped Key:%d to (%d,%d,%d) via %s\n", 
                       g_nodes[candidate_idx].key, nx, ny, nz, surf);
            }
        }
        pthread_mutex_unlock(&report_mutex);
    }

    // 3. 結果出力
    fprintf(fp, "\n# Final Cube Coordinates\n");
    for (int i = 0; i < g_node_count; i++) {
        fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n",
                g_nodes[i].addr, g_nodes[i].key, g_nodes[i].x, g_nodes[i].y, g_nodes[i].z);
    }

    fprintf(fp, "\n# Raw Reports\n");
    pthread_mutex_lock(&report_mutex);
    for (int r = 0; r < g_report_count; r++) {
        fprintf(fp, "%s | %s | %s\n", g_reports[r].target_addr, g_reports[r].detected_addr, g_reports[r].surface);
    }
    pthread_mutex_unlock(&report_mutex);

    g_report_count = 0;
    fclose(fp);

    printf("✅ [SUCCESS] 全ノードマッピング完了 (%d/%d台 確定)。\n", mapped_count, g_node_count);
    return 0;
}

static void compress_addr_to_hex12(const char *addr, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    if (!addr) { snprintf(out, out_size, "000000000000"); return; }
    unsigned int b[6];
    if (sscanf(addr, "%2x:%2x:%2x:%2x:%2x:%2x",
               &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]) == 6) {
        snprintf(out, out_size, "%02X%02X%02X%02X%02X%02X",
                 b[0], b[1], b[2], b[3], b[4], b[5]);
    } else {
        int j = 0;
        for (int i = 0; addr[i] != '\0' && j < (int)out_size - 1; i++) {
            if (addr[i] != ':') out[j++] = addr[i];
        }
        out[j] = '\0';
    }
}

int BLE_send_map_result(int map_result) {
    printf("[COMM] Sending mapping frames & MAP_END advertise (Result: %d)\n", map_result);
    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route (MAP_END)"); return -1; }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev (MAP_END)"); return -1; }

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(100 * 1.6);
    adv_params_cp.min_interval = htobs(interval);
    adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype = 0x00;
    adv_params_cp.chan_map = 0x07;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    uint8_t adv_data[31];
    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_cp;
    uint8_t enable;

    for (int i = 0; i < g_node_count; i++) {
        int key = g_nodes[i].key;
        int x = g_nodes[i].x, y = g_nodes[i].y, z = g_nodes[i].z;
        char addr_hex[13];
        compress_addr_to_hex12(g_nodes[i].addr, addr_hex, sizeof(addr_hex));
        char name_field[32];
        snprintf(name_field, sizeof(name_field), "#%d:%s:%d,%d,%d", key, addr_hex, x, y, z);

        int len = 0;
        memset(adv_data, 0, sizeof(adv_data));
        adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

        int name_len = (int)strlen(name_field);
        if (name_len > 26) name_len = 26;
        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09;
        memcpy(&adv_data[len], name_field, name_len); len += name_len;

        adv_cp.length = (uint8_t)len;
        memcpy(adv_cp.data, adv_data, len);
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_cp);

        enable = 0x01;
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
        printf("[COMM] MAP FRAME ADV: %s (5 seconds)\n", name_field);
        sleep(5);

        enable = 0x00;
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
        usleep(200000);
    }

    char end_field[32];
    snprintf(end_field, sizeof(end_field), "MAP_END:%s", map_result == 0 ? "SUCCESS" : "FAILURE");

    int len = 0;
    memset(adv_data, 0, sizeof(adv_data));
    adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

    int name_len = (int)strlen(end_field);
    if (name_len > 26) name_len = 26;
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(&adv_data[len], end_field, name_len); len += name_len;

    adv_cp.length = (uint8_t)len;
    memcpy(adv_cp.data, adv_data, len);

    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_cp);
    enable = 0x01;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

    printf("[COMM] MAP END advertising start (5 seconds)...\n");
    sleep(5);

    enable = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

    close(sock);
    return 0;
}

// ==========================================================
// main
// ==========================================================
int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <parent_full_address>\n", argv[0]);
        return 1;
    }
    const char *parent_addr = PARENT_ADDR_ARG;
    strncpy(g_parent_addr, parent_addr, 18);
    g_parent_addr[17] = '\0';

    printf("\n=== PARENT PROGRAM START ===\n");

    if (wiringPiSetupGpio() != 0) {
        fprintf(stderr, "FATAL ERROR: wiringPiSetupGpio failed. Cannot initialize GPIO.\n");
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

    // ★Heading offset 読み込み
    if (load_heading_offset(OFFSET_FILE, &g_heading_offset) != 0) {
        fprintf(stderr, "ERROR: Heading offset file not loaded. Aborting.\n");
        close(i2c_fd);
        return 1;
    }
    printf("Heading offset loaded: %.2f deg (using %s)\n", g_heading_offset, OFFSET_FILE);

    pthread_t rx_thread, sensor_th, mag_thread;
    pthread_create(&rx_thread, NULL, BLE_receive_data_server, NULL);
    pthread_create(&sensor_th, NULL, parent_sensor_monitor, NULL);

    printf("Parent Baseline sampling...\n");
    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) g_baseline[ch] += read_adc_voltage(ch);
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) g_baseline[ch] /= BASELINE_SAMPLES;

    int loaded = load_key_list(KEY_LIST_FILE);
    if (loaded > 0) {
        print_key_list();
        memset(g_child_ready, 0, sizeof(g_child_ready));
        g_ready_count = 0;
        g_total_children = 0;
        for (int i = 0; i < g_node_count; i++) {
            if (strcmp(g_nodes[i].addr, parent_addr) != 0) g_total_children++;
        }
    }

    int map_result = 1;
    if (loaded > 0) {
        pthread_create(&mag_thread, NULL, mag_sequence_thread, NULL);
        pthread_join(mag_thread, NULL);
        printf("シーケンス完了。マッピング計算...\n");
        map_result = coordinate_mapping();
    } else {
        printf("[WARN] キーリストなし。\n");
        map_result = 1;
    }

    BLE_send_map_result(map_result);

    gpio_off_and_unexport();
    if (i2c_fd != -1) close(i2c_fd);

    printf("親機終了 (Exit Code: %d)\n", map_result);
    return map_result;
}
