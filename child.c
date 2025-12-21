// child.c (時間分割プロトコル適用, WiringPi GPIO 制御版)

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

// main.c からの引数: argv[1] = 自分のアドレス, argv[2] = 親のアドレス
#define CHILD_ADDR_ARG  argv[1]
#define PARENT_ADDR_ARG argv[2]

#define CHILD_MAP_FILE_PATH "mapping.txt"

// プロトコル時間定義 (親機と一致させる)
#define P_MAG_TIME_SEC   30  
#define P_SCAN_TIME_SEC  10
#define C_MT_TIME_SEC    10
#define C_ACT_TIME_SEC   20
#define C_TX_TIME_SEC    10
#define COOLDOWN_SEC      5

// 子機の総台数を環境変数から取得するための定義
#define ENV_CHILD_COUNT "TOTAL_CHILD_NODES"
static int g_total_child_nodes = 1; // 1台の子機をデフォルトとする

// 自分のアドレス（スキャンスレッドから参照）
static char g_my_addr[18] = "(unknown)";
static volatile int electromagnet_requested = 0;
static volatile int electromagnet_active    = 0;
static pthread_mutex_t mag_state_mutex = PTHREAD_MUTEX_INITIALIZER;
static volatile int map_end_received = 0;
static volatile int g_scan_phase = 0;
// 0=MT待ち, 1=MAP 配布受信

// GPIO
const int BCM_GPIO_PIN = 20; // BCMピン番号を使用
// g_linux_gpio は WiringPi 移行により不要
// ==========================================================
// 構造体定義とグローバル変数 (競合エラー修正済み)
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
// BNO055/ADC 定義
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7
#define DIFF_THRESHOLD        0.010 
#define STABLE_COUNT_REQUIRED 10 
#define BASELINE_SAMPLES      50

#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_OPR_MODE_ADDR   0x3D
#define OPERATION_MODE_NDOF    0x0C
#define EULER_UNIT             16.0f
const char *OFFSET_FILE  = "bno055_heading_offset.txt";
static int i2c_fd = -1;
static float g_heading_offset = 0.0f;
double g_baseline[NUM_CH] = {0};
// レポートキュー構造体
#define MAX_REPORT_QUEUE 20 
typedef struct {
    char surface[16];
} ReportQueueEntry;

static ReportQueueEntry g_report_queue[MAX_REPORT_QUEUE];
static volatile int g_report_queue_count = 0;
static pthread_mutex_t report_queue_mutex = PTHREAD_MUTEX_INITIALIZER;


// ==========================================================
// プロトタイプ宣言
// ==========================================================
// static int readInt(const char *path); // 削除
// static int find_gpio_base(void); // 削除
// static int writeFileSilent(const char *path, const char *value); // 削除
// static int writeFile(const char *path, const char *value); // 削除
// static int fileExists(const char *path); // 削除
static int i2c_open(void);
static int bno055_init_ndof(void);
double read_adc_voltage(int ch);
float read_bno055_heading(int fd, float heading_offset);
int BLE_send_ready(const char *my_addr);
int BLE_send_surface_data(const char *my_addr, const char *parent_addr, const char *surface_name);
static void child_map_init(void);
static void expand_compact_addr12(const char *compact, char *out, size_t out_size);
static void child_store_map_frame(int key, const char *addr, int x, int y, int z);
static void child_dump_map_summary(const char *parent_addr);
int BLE_scan_for_targets(const char *target_addr, int timeout_ms, int phase);
int BLE_scan_for_MT(const char *target_addr, int timeout_sec);
void monitor_hall_sensor_and_report(const double *baseline, const char *my_addr, const char *parent_addr, int duration_sec);
static int gpio_init_and_on(void);
static void gpio_off_and_unexport(void);
static int setup_adv_parameters(int sock, uint16_t interval_ms);
static int i2c_write8(uint8_t reg, uint8_t value);
static int i2c_read_len(uint8_t reg, uint8_t *buf, int len);
static int bno055_set_mode(uint8_t mode);
static void msleep(int ms);
void send_queued_reports(const char *my_addr, const char *parent_addr, int duration_sec);
// ==========================================================
// GPIO 制御ヘルパー (Sysfs関数は削除し、WiringPiで置き換え)
// ==========================================================

// NOTE: Sysfsの不安定なファイルI/O関数は削除しました。
// WiringPiの関数を使用するため、これらのヘルパーは不要です。

static int gpio_init_and_on(void) {
    // WiringPi の BCM モードでピンをセットアップ
    // WiringPiSetup() は main で呼ばれているため、ピンモード設定から開始
    pinMode(BCM_GPIO_PIN, OUTPUT);
    digitalWrite(BCM_GPIO_PIN, HIGH);
    printf("[GPIO] 子機電磁石 ON (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
    return 0;
}

static void gpio_off_and_unexport(void) {
    digitalWrite(BCM_GPIO_PIN, LOW);
    printf("[GPIO] 子機電磁石 OFF (BCM %d) using WiringPi.\n", BCM_GPIO_PIN);
    // WiringPiSetup() が BCM モードなので、終了時に自動的にクリーンアップされます。
}

// ==========================================================
// WiringPi 移行により削除された Sysfs Helper のダミー関数 (コンパイルエラー回避のため)
// ==========================================================
// NOTE: これらの関数が他の場所で参照されている可能性があるため、ここでは残しますが、
//       新しいプログラムでは WiringPi API に置き換えることでこれらの不安定な関数は不要になります。
static int readInt(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) return -1;
    int v;
    if (fscanf(f, "%d", &v) != 1) { fclose(f); return -1; }
    fclose(f);
    return v;
}
static int writeFile(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) return -1;
    if (fprintf(f, "%s", value) < 0) { fclose(f); return -1; }
    fclose(f);
    return 0;
}
static int find_gpio_base(void) { return -1; }
static int writeFileSilent(const char *path, const char *value) { return 0; }
static int fileExists(const char *path) { return 1; }
// ==========================================================


// ==========================================================
// BLE送信ユーティリティ
// ==========================================================
static int setup_adv_parameters(int sock, uint16_t interval_ms) {
    le_set_advertising_parameters_cp adv_params_cp;
memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(interval_ms * 1.6);
    adv_params_cp.min_interval = htobs(interval); adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype = 0x00;
adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07; adv_params_cp.filter = 0x00;

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
    if (dev_id < 0) { perror("[BLE] hci_get_route"); return -1;
}
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev"); return -1;
}
    if (setup_adv_parameters(sock, 100) < 0) { close(sock); return -1; }

    uint8_t adv_data[31];
memset(adv_data, 0, sizeof(adv_data)); int len = 0;
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
close(sock); return -1;
    }

    for (int i = 0; i < 3; i++) { 
        uint8_t enable = 0x01;
if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                         1, &enable) < 0) {
            perror("[BLE] Failed to enable advertising");
close(sock); return -1;
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
    printf("[COMM] Sending READY advertise.\n");
    int dev_id = hci_get_route(NULL);
if (dev_id < 0) { perror("[BLE] hci_get_route (READY)"); return -1; }
    int sock = hci_open_dev(dev_id);
if (sock < 0) { perror("[BLE] hci_open_dev (READY)"); return -1;
}
    if (setup_adv_parameters(sock, 100) < 0) { close(sock); return -1; }

    uint8_t adv_data[31];
memset(adv_data, 0, sizeof(adv_data)); int len = 0;
    adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;
const char *name_field = "READY"; int name_len = (int)strlen(name_field);
    adv_data[len++] = (uint8_t)(name_len + 1); adv_data[len++] = 0x09;
memcpy(adv_data + len, name_field, name_len); len += name_len;
    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;
    adv_data_cp_struct.length = (uint8_t)len;
memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
    memcpy(adv_data_cp_struct.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp_struct) < 0) {
        perror("[BLE] Failed to set advertising data (READY)");
close(sock); return -1;
    }

    time_t start_time = time(NULL); uint8_t enable = 0x01; uint8_t disable = 0x00;
while (time(NULL) - start_time < 10) {
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable) < 0) {
            perror("[BLE] Failed to enable advertising (READY)");
break;
        }
        usleep(100000);
if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable) < 0) {
             perror("[BLE] Failed to disable advertising (READY)");
break;
        }
        usleep(50000); 
    }
    printf("[COMM] READY advertising finished.\n");
    close(sock);
    return 0;
}

// ==========================================================
// BNO055/ADC
// ==========================================================
static void msleep(int ms) { usleep(ms * 1000);
}

static int i2c_open(void) {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) { perror("open(/dev/i2c-1)"); return -1;
}
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) { perror("ioctl(I2C_SLAVE)"); close(i2c_fd); i2c_fd = -1; return -1;
}
    return i2c_fd;
}

static int bno055_set_mode(uint8_t mode) {
    uint8_t buf[2] = { BNO055_OPR_MODE_ADDR, mode };
if (write(i2c_fd, buf, 2) != 2) return -1;
    msleep(30);
    return 0;
}

static int bno055_init_ndof(void) {
    uint8_t id;
uint8_t reg = BNO055_CHIP_ID_ADDR;
    if (write(i2c_fd, &reg, 1) != 1) return -1;
    if (read(i2c_fd, &id, 1) != 1) return -1;
if (id != BNO055_ID_EXPECTED) return -1;
    printf("BNO055 detected! ID=0x%02X\n", id);
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) return -1;
    msleep(50);
printf("BNO055 initialized in NDOF mode.\n");
    return 0;
}

float read_bno055_heading(int fd, float heading_offset) {
    (void)fd;
uint8_t reg = BNO055_EUL_HEADING_LSB;
    uint8_t buf[2];
    if (write(i2c_fd, &reg, 1) != 1) return NAN;
if (read(i2c_fd, buf, 2) != 2) return NAN;
    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
float heading_raw   = raw_heading / EULER_UNIT;
    float heading_north = heading_raw - heading_offset;
while (heading_north < 0.0f)    heading_north += 360.0f;
    while (heading_north >= 360.0f) heading_north -= 360.0f;
    return heading_north;
}

int load_heading_offset(const char *path, float *offset) {
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
if (fscanf(fp, "%f", offset) != 1) { fclose(fp); return -1; }
    fclose(fp);
    return 0;
}

double read_adc_voltage(int ch) {
    unsigned char data[3];
    data[0] = 1; data[1] = (8 + ch) << 4;
data[2] = 0;
    wiringPiSPIDataRW(SPI_CH, data, 3);
    int value = ((data[1] & 3) << 8) | data[2];
double voltage = (double)value * 3.3 / 1023.0;
    return voltage;
}

// ==========================================================
// 子機側マッピング保存構造体
// ==========================================================
static void child_map_init(void) {
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        g_child_map[i].valid = 0;
g_child_map[i].key = 0; g_child_map[i].addr[0] = '\0'; g_child_map[i].x = g_child_map[i].y = g_child_map[i].z = 0;
    }
}

static void expand_compact_addr12(const char *compact, char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
if (!compact) { snprintf(out, out_size, "UNKNOWN"); return; }
    size_t len = strlen(compact);
if (len < 12) { snprintf(out, out_size, "%s", compact); return; }
    char buf[18]; int j = 0;
for (int i = 0; i < 12 && j < 17; i += 2) {
        buf[j++] = compact[i];
if (j >= 17) break; buf[j++] = compact[i + 1];
if (i != 10 && j < 17) { buf[j++] = ':';
}
    }
    buf[j] = '\0'; snprintf(out, out_size, "%s", buf);
}

static void child_store_map_frame(int key,
                                  const char *addr,
                                  int x, int y, int z)
{
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if 
(g_child_map[i].valid && g_child_map[i].key == key) {
            if (g_child_map[i].x == x && g_child_map[i].y == y && g_child_map[i].z == z) { return;
}
            else { g_child_map[i].x = x; g_child_map[i].y = y;
g_child_map[i].z = z;
                if (addr && addr[0] != '\0') { strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr)); g_child_map[i].addr[sizeof(g_child_map[i].addr) - 1] = '\0';
}
                printf("[MAP-RECV] key=%d addr=%s -> (%d,%d,%d) (UPDATED)\n",
                       key, g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN", x, y, z);
return;
            }
        }
    }
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (!g_child_map[i].valid) {
            g_child_map[i].valid = 1;
g_child_map[i].key = key; g_child_map[i].x = x; g_child_map[i].y = y; g_child_map[i].z = z;
if (addr && addr[0] != '\0') { strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr)); g_child_map[i].addr[sizeof(g_child_map[i].addr) - 1] = '\0';
} else { g_child_map[i].addr[0] = '\0'; }
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
                   g_child_map[i].key, g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN", g_child_map[i].x, g_child_map[i].y, g_child_map[i].z);
}
    }
    printf("============================================\n");

    FILE *fp = fopen(CHILD_MAP_FILE_PATH, "w"); if (!fp) return;
fprintf(fp, "# Cube Mapping Log\n"); fprintf(fp, "\n# Final Cube Coordinates\n");
for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid) {
            fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n",
                    g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                    g_child_map[i].key, g_child_map[i].x, g_child_map[i].y, g_child_map[i].z);
}
    }
    fprintf(fp, "\n# Raw Reports\n"); fclose(fp);
}

// ==========================================================
// BLE スキャン: MAP_END/MK 受信専用
// ==========================================================
int BLE_scan_for_targets(const char *target_addr, int timeout_ms, int phase) {
    int dev_id = hci_get_route(NULL);
if (dev_id < 0) return 0;
    int sock = hci_open_dev(dev_id); if (sock < 0) return 0;
    struct hci_filter nf; hci_filter_clear(&nf);
hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf); setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));
    le_set_scan_parameters_cp scan_params_cp; memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01; scan_params_cp.interval = htobs(0x0010);
scan_params_cp.window   = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00; scan_params_cp.filter   = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);
uint8_t enable = 0x01; uint8_t filter_dup = 0x00; uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
unsigned char buf[HCI_MAX_EVENT_SIZE];
    struct timeval tv = { .tv_sec  = timeout_ms / 1000, .tv_usec = (timeout_ms % 1000) * 1000 };
int ret_val = 0;

    while (ret_val == 0) {
        fd_set fds; FD_ZERO(&fds);
FD_SET(sock, &fds);
        struct timeval current_tv = tv;
        int r = select(sock + 1, &fds, NULL, NULL, &current_tv);
if (r <= 0) break;
        int len = read(sock, buf, sizeof(buf));
if (len < 0) { if (errno == EINTR) continue; break;
}
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
uint8_t *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;
uint8_t reports = meta->data[0]; uint8_t *offset = meta->data + 1;
for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
char name[128] = ""; int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
if (field_len == 0 || pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];
if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
if (name_len > (int)sizeof(name)-1) name_len = (int)sizeof(name)-1;
                    memcpy(name, &info->data[pos + 2], name_len); name[name_len] = '\0';
}
                pos += field_len + 1;
}

            if (name[0] != '\0') {
                if (phase == 1) {
                    const char *p = NULL;
if (strncmp(name, "MK:", 3) == 0) { p = name + 3;
} else if (strncmp(name, "MAPK:", 5) == 0) { p = name + 5;
}

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
                                expand_compact_addr12(compact, full_addr, sizeof(full_addr)); p = next_colon + 1;
                            } else { snprintf(full_addr, sizeof(full_addr), "UNKNOWN");
}
                            long x = strtol(p, &endp, 10);
if (endp && *endp == ',') {
                                p = endp + 1;
long y = strtol(p, &endp, 10);
                                if (endp && *endp == ',') { p = endp + 1;
long z = strtol(p, &endp, 10);
                                    child_store_map_frame((int)key, full_addr, (int)x, (int)y, (int)z);
}
                            }
                        }
                    }
                    if (strncmp(name, "MAP_END:", 8) == 0) {
  
printf("\n[SCAN] フェーズ2: 親機からのマッピング送信完了通知を受信: %s\n", name);
ret_val = 2; map_end_received = 1; break;
                    }
                }
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
}
        if (ret_val != 0) break;
}

    uint8_t disable2 = 0x00; cmd[0] = disable2; cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
    close(sock);
    return ret_val;
}

// ==========================================================
// MT通知監視専用関数 (MT通知のみに専念, 10秒間監視を継続)
// ==========================================================
int BLE_scan_for_MT(const char *target_addr, int timeout_sec) {
    int dev_id = hci_get_route(NULL);
if (dev_id < 0) return 0;
    int sock = hci_open_dev(dev_id); if (sock < 0) return 0;
    
    struct hci_filter nf; hci_filter_clear(&nf);
hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf); setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));
    le_set_scan_parameters_cp scan_params_cp; memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01; scan_params_cp.interval = htobs(0x0010);
scan_params_cp.window   = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00; scan_params_cp.filter   = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);
uint8_t enable = 0x01; uint8_t filter_dup = 0x00; uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
unsigned char buf[HCI_MAX_EVENT_SIZE]; time_t start_time = time(NULL);

    int mt_found = 0; // MT発見フラグ

    while (time(NULL) - start_time < timeout_sec) {
        struct timeval tv = { .tv_sec = 0, .tv_usec = 50000 };
fd_set fds; FD_ZERO(&fds); FD_SET(sock, &fds);

        if (select(sock + 1, &fds, NULL, NULL, &tv) <= 0) continue;
int len = read(sock, buf, sizeof(buf));
        if (len < 0) { if (errno == EINTR) continue; break;
}
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
uint8_t *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;
uint8_t reports = meta->data[0]; uint8_t *offset = meta->data + 1;
for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
char name[128] = ""; int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
if (field_len == 0 || pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];
if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
if (name_len > 127) name_len = 127;
                    memcpy(name, &info->data[pos + 2], name_len); name[name_len] = '\0';
}
                pos += field_len + 1;
}

            if (strncmp(name, "MT:", 3) == 0 && strcmp(name + 3, target_addr) == 0) {
                // MTを受信してもすぐに終了せず、フラグを立てる
                mt_found = 1; 
                printf("[INFO] MT notification received! Waiting for phase end...\n");
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
}
    }

    uint8_t disable2 = 0x00; cmd[0] = disable2; cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
    close(sock);
    return mt_found; // フラグを返す
}

// ==========================================================
// ホールセンサー検知専念関数 (レポートの単一化と回数制限を適用)
// ==========================================================
void monitor_hall_sensor_and_report(const double *baseline, const char *my_addr, const char *parent_addr, int duration_sec) {
    
    printf("[SENSOR] ホールセンサー検知に専念 (%d秒)...\n", duration_sec);
time_t start_time = time(NULL);
    static char last_surface[16] = "";
    static int  same_count = 0;
    
    const int MAX_REPORTS_PER_PHASE = 10; // 10回までレポートをキューに格納
    int reports_in_this_phase = 0;

while (time(NULL) - start_time < duration_sec) {

        // 10回検知したら残りの時間は待機してループを抜ける
        if (reports_in_this_phase >= MAX_REPORTS_PER_PHASE) {
             printf("[SENSOR] 検知上限 (%d回) に達しました。残り時間を待機します。\n", MAX_REPORTS_PER_PHASE);
             time_t elapsed_time = time(NULL) - start_time;
             if (duration_sec > elapsed_time) {
                 sleep((unsigned int)(duration_sec - elapsed_time));
             }
             break; // ループを抜ける
        }


        // --- センサー検知ロジック (中略) ---
        double window[NUM_CH];
for (int ch = 1; ch < NUM_CH; ch++) {
            window[ch] = read_adc_voltage(ch);
}

        double maxDiff = 0.0;
        int maxCh = -1;
for (int ch = 1; ch < NUM_CH; ch++) {
            if (baseline[ch] == 0.0) continue;
double diff = fabs(window[ch] - baseline[ch]);
            if (diff > maxDiff) {
                maxDiff = diff;
maxCh = ch;
            }
        }

        int is_candidate = 0;
if (maxCh != -1 && maxDiff >= DIFF_THRESHOLD) {
            is_candidate = 1;
}

        if (is_candidate) {
            float heading_north = read_bno055_heading(i2c_fd, g_heading_offset);
const char* detected_surface = "UNKNOWN";

            if (!isnan(heading_north)) {
                
                if (maxCh == 1)      detected_surface = "TOP";
else if (maxCh == 6) detected_surface = "BOTTOM";
                    else {
                        int rotation_index = 0;
if (heading_north >= 45.0f  && heading_north < 135.0f) rotation_index = 1;
if (heading_north >= 135.0f && heading_north < 225.0f) rotation_index = 2;
if (heading_north >= 225.0f && heading_north < 315.0f) rotation_index = 3;
                        else rotation_index = 0;

                        int initial_index = -1;
if (maxCh == 2)      initial_index = 0;
                        else if (maxCh == 3) initial_index = 1;
if (maxCh == 4) initial_index = 2;
                        else if (maxCh == 5) initial_index = 3;
if (initial_index != -1) {
                            int mapped_index = (initial_index + rotation_index) % 4;
const char* final_names[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
                            detected_surface = final_names[mapped_index];
}
                    }

                char current_surface[16];
strncpy(current_surface, detected_surface, sizeof(current_surface));
                current_surface[sizeof(current_surface)-1] = '\0';

                if (strcmp(last_surface, current_surface) == 0) {
                    same_count++;
} else {
                    strncpy(last_surface, current_surface, sizeof(last_surface));
last_surface[sizeof(last_surface)-1] = '\0';
                    same_count = 1;
                }

                if (same_count >= STABLE_COUNT_REQUIRED) {
                    printf("\n[EVENT CONFIRMED] CH%d detected (Diff: %.3f V) -> Surface: **%s**\n",
                           maxCh, maxDiff, current_surface);
 
                    // C-ACTフェーズ または P-MAG ON フェーズでキューに記憶
                    if (reports_in_this_phase < MAX_REPORTS_PER_PHASE && 
                        (duration_sec == C_ACT_TIME_SEC || duration_sec == P_MAG_TIME_SEC)) {
                         
                         pthread_mutex_lock(&report_queue_mutex);
                         
                         if (g_report_queue_count < MAX_REPORT_QUEUE) {
                             strncpy(g_report_queue[g_report_queue_count].surface, current_surface, 16);
                             g_report_queue_count++;
                             reports_in_this_phase++;
                             printf("[REPORT QUEUED] Surface %s (Total: %d)\n", current_surface, g_report_queue_count);
                         }
                         pthread_mutex_unlock(&report_queue_mutex);
                    } else {
                        // C-ACT/P-MAG以外ではコンソール出力のみ
                        printf("[EVENT] Detected during Passive Phase: %s\n", current_surface);
                    }
                    
                    same_count = 0;
last_surface[0] = '\0';
                    usleep(500000); // イベント後の短いクールダウン
                }
            }
        }
        usleep(10000);
// センサー読み取り間の短い待機 (10ms)
    }
}

// ==========================================================
// レポート送信専念関数 (キューが空になったら残りの時間を待機)
// ==========================================================
void send_queued_reports(const char *my_addr, const char *parent_addr, int duration_sec) {
    time_t start_time = time(NULL);
    printf("[COMM] C-TX Phase: Sending %d queued reports for %d seconds...\n", g_report_queue_count, duration_sec);
    
    // C-TXフェーズの間、定期的にレポートを送信する
    while (time(NULL) - start_time < duration_sec) {
        pthread_mutex_lock(&report_queue_mutex);
        
        if (g_report_queue_count > 0) {
            // キューの先頭のレポートを取り出して送信
            char surface_to_send[16];
            strncpy(surface_to_send, g_report_queue[0].surface, 16);
            surface_to_send[15] = '\0';

            printf("[COMM] Sending queued report: %s\n", surface_to_send);
            BLE_send_surface_data(my_addr, parent_addr, surface_to_send);

            // キューから削除し、残りをシフト (レポートは1つしかないはずだが処理は維持)
            for (int i = 0; i < g_report_queue_count - 1; i++) {
                g_report_queue[i] = g_report_queue[i + 1];
            }
            g_report_queue_count--;
            pthread_mutex_unlock(&report_queue_mutex);
            
            // レポートを送信し終えたら、残りの時間を待機して終了
            if (g_report_queue_count == 0) {
                time_t elapsed_time = time(NULL) - start_time;
                if (duration_sec > elapsed_time) {
                    printf("[COMM] All reports sent. Waiting %ld seconds for C-TX end.\n", duration_sec - elapsed_time);
                    sleep((unsigned int)(duration_sec - elapsed_time));
                }
                goto end_tx_phase; // ループを抜ける
            }

        } else {
            pthread_mutex_unlock(&report_queue_mutex);
            // キューが空の場合、残りの時間を待機して終了する
            time_t elapsed_time = time(NULL) - start_time;
            if (duration_sec > elapsed_time) {
                 printf("[COMM] No reports in queue. Waiting %ld seconds for C-TX end.\n", duration_sec - elapsed_time);
                 sleep((unsigned int)(duration_sec - elapsed_time));
            }
            break; // ループを抜ける
        }
    }
    
end_tx_phase:; // goto のターゲット
    
    pthread_mutex_lock(&report_queue_mutex);
    if (g_report_queue_count > 0) {
        printf("[WARN] C-TX ended with %d reports still in queue.\n", g_report_queue_count);
    }
    pthread_mutex_unlock(&report_queue_mutex);
}


// ==========================================================
// main: 時間分割ロジック適用 (サイクル数制御を導入)
// ==========================================================
int main(int argc, char *argv[])
{
setvbuf(stdout, NULL, _IONBF, 0);
    if (argc < 3) {
        fprintf(stderr,
                "Usage: ./child <my_full_address> <parent_full_address>\n");
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

    // ★ 修正: WiringPiSetup() を使用して BCM ピン制御を初期化
    if (wiringPiSetupGpio() != 0) { // BCMピン番号を使用するよう変更
        fprintf(stderr, "ERROR: wiringPiSetupGpio failed.\n");
return 1;
    }
    printf("[INFO] WiringPi BCM Setup successful.\n");


    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) {
        fprintf(stderr, "FATAL ERROR: wiringPiSPISetup failed. Check SPI enablement and permissions.\n");
return 2; // SPIエラー専用コード
    }
    i2c_fd = i2c_open();
if (i2c_fd < 0 || bno055_init_ndof() < 0) {
        return 3;
// I2C/BNOエラー専用コード
    }

    // ★ 修正: Sysfs GPIO アドレス解決ロジックは WiringPi 移行により削除
    printf("GPIO Pin used for Magnet: BCM %d\n", BCM_GPIO_PIN);


    float heading_offset = 0.0f;
if (load_heading_offset(OFFSET_FILE, &heading_offset) != 0) {
        printf("ERROR: Heading offset file not loaded. Aborting.\n");
close(i2c_fd);
        return 1;
    }

    double baseline[NUM_CH] = {0};
    printf("Baseline sampling...\n");
for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) {
            baseline[ch] += read_adc_voltage(ch);
}
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) {
        baseline[ch] /= BASELINE_SAMPLES;
}
    printf("Baseline established.\n");

    if (BLE_send_ready(my_addr) < 0) {
        fprintf(stderr, "Failed to send READY advertising.\n");
close(i2c_fd);
        return 1;
    }

    // 子機台数を環境変数から取得し、サイクル数を設定
    char *count_str = getenv(ENV_CHILD_COUNT);
    if (count_str) {
        g_total_child_nodes = atoi(count_str);
        if (g_total_child_nodes <= 0) g_total_child_nodes = 1;
    }
    printf("[CONFIG] Expected total child nodes (Cycles): %d\n", g_total_child_nodes);


    child_map_init();
    printf("Begin continuous monitoring...\n");
    g_scan_phase = 0;
    
    int node_cycle_count = 0; // サイクルカウンターを導入
    
// =====================================================
    // P-MAG ON / P-SCAN サイクル (シーケンス開始前の初期フェーズ)
    // =====================================================
    
    // P-MAG ON: 親機が電磁石ON。子機は検知に専念 (レポートをキューに格納)
    printf("\n[PHASE START] 初期センサー検知専念 (P-MAG ON) (%d秒)\n", P_MAG_TIME_SEC);
monitor_hall_sensor_and_report(baseline, my_addr, parent_addr, P_MAG_TIME_SEC);

    printf("[PHASE] クールダウン (%d秒)\n", COOLDOWN_SEC);
    sleep((unsigned int)COOLDOWN_SEC);

    // P-SCAN (10秒): 親機からのレポート受信時間。子機はレポート送信に専念
    printf("[PHASE START] P-SCAN: レポート送信専念 (%d秒)\n", P_SCAN_TIME_SEC);
    send_queued_reports(my_addr, parent_addr, P_SCAN_TIME_SEC); 
    
    // =====================================================
    // メインループ: C-MT / C-ACT サイクル (接続台数分実行)
    // =====================================================
    while (node_cycle_count < g_total_child_nodes)
    {
        // 1. C-MT: MT通知監視 (10秒)
        printf("\n[PHASE START] C-MT: MT通知監視専念 (%d秒) [Cycle %d/%d]\n", 
               C_MT_TIME_SEC, node_cycle_count + 1, g_total_child_nodes);
        // BLE_scan_for_MT は10秒間監視を継続する。
        int mt_received = BLE_scan_for_MT(g_my_addr, C_MT_TIME_SEC); 

        // 2. C-ACT: 駆動 or センサー専念 (20秒)
        printf("[PHASE START] C-ACT: 駆動 or センサー専念 (%d秒)\n", C_ACT_TIME_SEC);
if (mt_received) {
            // MTを受信した場合: 電磁石を駆動
            
            // BLE_scan_for_MT が10秒消費したため、待機は不要。すぐにON
            printf("[MAG] MT受信確認。電磁石をすぐにONにします...\n");

            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 1;
            pthread_mutex_unlock(&mag_state_mutex);

            printf("[MAG] 電磁石 ON (BCM %d) (%d秒)\n", BCM_GPIO_PIN, C_ACT_TIME_SEC);
            if (gpio_init_and_on() == 0) { // WiringPi を使用した駆動
                sleep((unsigned int)C_ACT_TIME_SEC);
                gpio_off_and_unexport();
            } else {
                fprintf(stderr, "[MAG ERROR] Failed to activate magnet using WiringPi. Skipping drive time.\n");
            }

            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 0;
            pthread_mutex_unlock(&mag_state_mutex);
// 電磁石駆動後は残留磁気対策として強制クールダウン
            printf("[SENSOR] 駆動後、磁気安定のため3秒待機...\n");
            sleep(3);
} else {
            // MTを受信しなかった場合: ホールセンサーを監視 (検知結果をキューに記憶)
            monitor_hall_sensor_and_report(baseline, my_addr, parent_addr, C_ACT_TIME_SEC);
}
        
        // 3. C-TX: レポート送信 (10秒)
        printf("[PHASE START] C-TX: レポート送信専念 (%d秒)\n", C_TX_TIME_SEC);
        send_queued_reports(my_addr, parent_addr, C_TX_TIME_SEC);
        
        // 4. C-CD: クールダウン (5秒)
        printf("[PHASE] クールダウン (%d秒)\n", COOLDOWN_SEC);
        sleep((unsigned int)COOLDOWN_SEC);
g_scan_phase = 0; // 次のMT監視フェーズに戻る

        node_cycle_count++; // サイクルをカウントアップ
    }
    
    // ループ終了後、マッピング配布受信フェーズに移行
    printf("\n[SEQUENCE END] 全ノードのMTサイクルが完了しました。MAP配布受信フェーズへ移行します。\n");

    // MAP 配布受信フェーズ
    printf("\n[PHASE START] MAP 配布受信モード (MK/MAP_ENDを受信)\n");
g_scan_phase = 1;
    while (!map_end_received) {
        BLE_scan_for_targets(g_my_addr, 50, g_scan_phase); 
        usleep(50000);
}
    
    printf("\n\n===== マッピング終了通知(MAP_END)を受信しました。子機プログラムを終了します。 =====\n");
    child_dump_map_summary(parent_addr);
    if (i2c_fd != -1) close(i2c_fd);
    gpio_off_and_unexport();
printf("CHILD PROGRAM END\n");
    return 0;
}
