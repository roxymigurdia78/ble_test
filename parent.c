// parent.c (修正版: Index方式マッピング & ロジック修正 & 安全対策強化)
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
#include <math.h> 

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
#define MAP_FILE_PATH   "cube_map_log.txt"
#define KEY_LIST_FILE   "parent_key_list.txt"
#define MAX_DATA_LEN    64
#define MAX_NODES       32

// ==========================================================
// センサー関連 定義とグローバル変数
// ==========================================================
// BNO055 / ADC 定数
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7 
#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_OPR_MODE_ADDR   0x3D
#define OPERATION_MODE_NDOF    0x0C
#define EULER_UNIT             16.0f
const char *OFFSET_FILE  = "bno055_heading_offset.txt";

// ホールセンサーのしきい値
#define DIFF_THRESHOLD        0.020
#define SECOND_MARGIN         0.0     
#define STABLE_COUNT_REQUIRED 1       

#define BASELINE_SAMPLES 50

// グローバル変数
static int i2c_fd = -1;
static float g_heading_offset = 0.0f;
static double g_baseline[NUM_CH] = {0};

// 親機の電磁石駆動状態を監視するためのフラグ
static volatile int g_parent_mag_active = 0;

// ★★★ GPIO 制御定義 ★★★
const int BCM_GPIO_PIN = 20; // BCM 20 を使用
static int g_linux_gpio = -1;

// ==========================================================
// ★★★ GPIO 制御ヘルパー関数 ★★★
// ==========================================================

static int writeFile(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return -1; }
    if (fprintf(f, "%s", value) < 0) { perror("fprintf"); fclose(f); return -1; }
    fclose(f);
    return 0;
}

static int writeFileSilent(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) { return -1; }
    if (fprintf(f, "%s", value) < 0) { fclose(f); return -1; }
    fclose(f);
    return 0;
}

static int readInt(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    int v;
    if (fscanf(f, "%d", &v) != 1) { fprintf(stderr, "failed to read int from %s\n", path); fclose(f); return -1; }
    fclose(f);
    return v;
}

static int fileExists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

// ★ 電磁石 ON 処理
static int gpio_init_and_on(void) {
    char num_str[16];
    char path_dir[128];
    char path_val[128];
    
    if (g_linux_gpio == -1) {
        fprintf(stderr, "ERROR: GPIO pin not initialized.\n");
        return -1;
    }

    snprintf(num_str, sizeof(num_str), "%d", g_linux_gpio);
    snprintf(path_dir, sizeof(path_dir), "/sys/class/gpio/gpio%d/direction", g_linux_gpio);
    snprintf(path_val, sizeof(path_val), "/sys/class/gpio/gpio%d/value", g_linux_gpio);
    
    // クリーンアップ
    writeFileSilent("/sys/class/gpio/unexport", num_str);
    usleep(100000);
    
    // Export
    if (writeFile("/sys/class/gpio/export", num_str) < 0) {
         fprintf(stderr, "ERROR: Failed to export GPIO %d. (Need sudo?)\n", g_linux_gpio);
         return -1;
    }
    usleep(100000); 

    // Direction Out
    if (writeFile(path_dir, "out") < 0) {
        fprintf(stderr, "ERROR: Failed to set direction to 'out'.\n");
        return -1;
    }

    // ON (1)
    if (writeFile(path_val, "1") < 0) {
        fprintf(stderr, "ERROR: Failed to write '1' to value.\n");
        return -1;
    }
    printf("[GPIO] Parent Electromagnet ON (GPIO %d)\n", g_linux_gpio);
    return 0;
}

// ★ 電磁石 OFF 処理
static void gpio_off_and_unexport(void) {
    char num_str[16];
    char path_val[128];
    if (g_linux_gpio == -1) return;

    snprintf(num_str, sizeof(num_str), "%d", g_linux_gpio);
    snprintf(path_val, sizeof(path_val), "/sys/class/gpio/gpio%d/value", g_linux_gpio);
    if (fileExists(path_val)) {
        // OFF (0)
        writeFileSilent(path_val, "0");
        // Unexport
        writeFileSilent("/sys/class/gpio/unexport", num_str);
        printf("[GPIO] Parent Electromagnet OFF & Unexport (GPIO %d) -> OFF信号送信完了\n", g_linux_gpio);
    }
}

// ==========================================================
// キーリスト/状態管理 構造体
// ==========================================================
typedef struct {
    int  key;
    char addr[18];
    // マッピング用: 座標
    int x, y, z; 
    // 親機のキーリスト上のインデックス (0から始まる)
    int index;
} KeyEntry;

static KeyEntry g_nodes[MAX_NODES];
static int g_node_count = 0;

// 親機アドレス
static char g_parent_addr[18] = "";
// 電磁石タイムの制御用
static pthread_mutex_t mag_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  mag_cond  = PTHREAD_COND_INITIALIZER;
static char current_mag_target[18] = "";
static int  mag_end_flag = 0;

// READY(準備完了) 用
static pthread_mutex_t ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  ready_cond  = PTHREAD_COND_INITIALIZER;
static int g_child_ready[MAX_NODES] = {0};
static int g_ready_count = 0;
static int g_total_children = 0;

// マッピング用データ構造 (レポート)
typedef struct {
    char target_addr[18];
    char detected_addr[18];
    char surface[16];
} ReportEntry;
#define MAX_REPORTS 256
static ReportEntry g_reports[MAX_REPORTS];
static int g_report_count = 0;
static pthread_mutex_t report_mutex = PTHREAD_MUTEX_INITIALIZER;


// ==========================================================
// ★★★ BFSマッピング用構造体・定義 ★★★
// ==========================================================
#define UNKNOWN 0
#define QUEUE_MAX 128
#define INF 999999

// 6方向の定義
typedef enum {PX=0, NX, PY, NY, PZ, NZ} DIR;

// ノード構造体 (Indexベース)
typedef struct {
    int id;           // ノードID (未使用、またはKeyを格納)
    int neighbor[6];  // 各方向の接続先Index (-1=なし)
    int x, y, z;      // 位置
    int fixed;        // 座標確定フラグ
} MappingNode;

static MappingNode mapping_nodes[MAX_NODES]; // Index 0～MAX_NODES-1

// キュー（BFS用）
static int q[QUEUE_MAX];
static int qh = 0, qt = 0;

// ==========================================================
// 関数プロトタイプ宣言 
// ==========================================================
void coordinate_mapping(void); 
int  get_key_by_addr(const char *addr);
int  get_index_by_addr(const char *addr);
void register_mag_end_from_child(const char *child_addr);
void register_child_ready(const char *child_addr);
void process_received_data(const char *received_data);
void *parent_sensor_monitor(void *arg);
int load_key_list(const char *path);
void print_key_list(void);

// ==========================================================
// センサー関連ヘルパー関数 
// ==========================================================
static void msleep(int ms) { usleep(ms * 1000); }

static int i2c_open(void) {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) { perror("open(/dev/i2c-1)"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) { 
        perror("ioctl(I2C_SLAVE)"); close(i2c_fd); i2c_fd = -1; return -1;
    }
    return 0;
}

static int i2c_write8(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(i2c_fd, buf, 2) != 2) { perror("i2c_write8"); return -1; }
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
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) { fprintf(stderr, "Failed to set NDOF mode\n"); return -1; }
    msleep(50);
    printf("BNO055 initialized in NDOF mode.\n");
    return 0;
}

float read_bno055_heading(int fd, float heading_offset) {
    (void)fd; 
    uint8_t buf[2];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 2) < 0) return NAN;
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
    data[0] = 1;
    data[1] = (8 + ch) << 4;
    data[2] = 0;
    wiringPiSPIDataRW(SPI_CH, data, 3);
    int value = ((data[1] & 3) << 8) | data[2];
    return (double)value * 3.3 / 1023.0;
}

// ==========================================================
// 共通ヘルパー関数
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

void write_log(const char *child_addr, const char *surface) {
    FILE *fp = fopen(LOG_FILE_PATH, "a");
    if (!fp) return;
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char time_str[26];
    strftime(time_str, 26, "%Y-%m-%d %H:%M:%S", tm_info);
    fprintf(fp, "[%s] [%s] DETECTED: %s\n", time_str, child_addr, surface);
    fclose(fp);
}

void register_mag_end_from_child(const char *child_addr) {
    pthread_mutex_lock(&mag_mutex);
    if (current_mag_target[0] != '\0' && strcmp(current_mag_target, child_addr) == 0) {
        mag_end_flag = 1;
        pthread_cond_signal(&mag_cond);
    }
    pthread_mutex_unlock(&mag_mutex);
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

void process_received_data(const char *received_data) {
    char data_copy[MAX_DATA_LEN];
    strncpy(data_copy, received_data, MAX_DATA_LEN);
    data_copy[MAX_DATA_LEN - 1] = '\0';
    
    char *child_addr = strtok(data_copy, ",");
    char *surface    = strtok(NULL, ",");

    if (child_addr && surface) {
        int child_key = get_key_by_addr(child_addr);
        printf("Surface-report from %s (Key: %d) => %s (Target: %s)\n", 
               child_addr, child_key, surface, current_mag_target[0] != '\0' ? current_mag_target : "None");
        write_log(child_addr, surface);

        pthread_mutex_lock(&report_mutex);
        if (g_report_count < MAX_REPORTS && current_mag_target[0] != '\0') {
            strncpy(g_reports[g_report_count].target_addr, current_mag_target, 18);
            strncpy(g_reports[g_report_count].detected_addr, child_addr, 18);
            strncpy(g_reports[g_report_count].surface, surface, 16);
            g_reports[g_report_count].target_addr[17] = '\0';
            g_reports[g_report_count].detected_addr[17] = '\0';
            g_reports[g_report_count].surface[15] = '\0';
            g_report_count++;
        }
        pthread_mutex_unlock(&report_mutex);
    } else {
        fprintf(stderr, "[ERROR] 受信データの形式が不正です: %s\n", received_data ? received_data : "(null)");
    }
}

// ==========================================================
// 親機のホールセンサー常時監視スレッド
// ==========================================================
void *parent_sensor_monitor(void *arg) {
    (void)arg;
    printf("[SENSOR] 親機センサー監視スレッド開始。電磁石停止後にレポートを開始します。\n");
    static char last_surface[16] = "";
    static int  same_count = 0;
    
    while (1) {
        if (g_parent_mag_active) { usleep(100000); continue; }
        if (current_mag_target[0] == '\0') { usleep(100000); continue; }

        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) window[ch] = read_adc_voltage(ch);

        double maxDiff = 0.0, secondDiff = 0.0;
        int maxCh = -1, secondCh = -1;

        for (int ch = 1; ch < NUM_CH; ch++) {
            if (g_baseline[ch] < 1.0 || window[ch] < 1.0) continue;
            double diff = fabs(window[ch] - g_baseline[ch]);
            if (diff > maxDiff) {
                secondDiff = maxDiff; secondCh = maxCh;
                maxDiff = diff; maxCh = ch;
            } else if (diff > secondDiff) {
                secondDiff = diff; secondCh = ch;
            }
        }
        
        int is_candidate = 0;
        if (maxCh != -1 && maxDiff >= DIFF_THRESHOLD) is_candidate = 1;

        if (is_candidate) {
            float heading_north = read_bno055_heading(i2c_fd, g_heading_offset);
            const char* detected_surface = "UNKNOWN";

            if (!isnan(heading_north)) {
                if (maxCh == 1)      detected_surface = "TOP";
                else if (maxCh == 6) detected_surface = "BOTTOM";
                else {
                    int rotation_index = 0;
                    if (heading_north >= 45.0f  && heading_north < 135.0f) rotation_index = 1;
                    else if (heading_north >= 135.0f && heading_north < 225.0f) rotation_index = 2;
                    else if (heading_north >= 225.0f && heading_north < 315.0f) rotation_index = 3;
                    else rotation_index = 0;
                    
                    int initial_index = -1;
                    if (maxCh == 2)      initial_index = 0; 
                    else if (maxCh == 3) initial_index = 1;
                    else if (maxCh == 4) initial_index = 2; 
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

                if (strcmp(last_surface, current_surface) == 0) same_count++;
                else {
                    strncpy(last_surface, current_surface, sizeof(last_surface));
                    last_surface[sizeof(last_surface)-1] = '\0';
                    same_count = 1;
                }

                if (same_count >= STABLE_COUNT_REQUIRED) {
                    printf("\n[PARENT EVENT CONFIRMED] CH%d detected (Diff: %.3f V) -> Surface: **%s**\n",
                           maxCh, maxDiff, current_surface);
                    char combined[MAX_DATA_LEN];
                    snprintf(combined, sizeof(combined), "%s,%s", g_parent_addr, current_surface);
                    process_received_data(combined); 
                    same_count = 0;
                    last_surface[0] = '\0';
                    sleep(2);
                }
            }
        }
        usleep(100000);
    }
    return NULL;
}

// ==========================================================
// BLE 受信スレッド (常時スキャン)
// ==========================================================
void *BLE_receive_data_server(void *arg) {
    (void)arg;
    printf("[COMM] BLE親機サーバーを起動しました。\n");

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route に失敗しました"); return NULL; }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev に失敗しました"); return NULL; }
    
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        perror("[BLE] HCIフィルタ設定に失敗しました"); close(sock); return NULL;
    }

    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type            = 0x01;
    scan_params_cp.interval        = htobs(0x0010);
    scan_params_cp.window          = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00;
    scan_params_cp.filter          = 0x00;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp) < 0) {
        perror("[BLE] スキャンパラメータ設定に失敗しました"); close(sock); return NULL;
    }

    uint8_t enable = 0x01, filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd) < 0) {
        perror("[BLE] スキャン有効化に失敗しました"); close(sock); return NULL;
    }

    printf("[COMM] BLEスキャンを開始しました。\n");
    unsigned char buf[HCI_MAX_EVENT_SIZE];

    while (1) {
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            perror("[BLE] read でエラーが発生しました"); break;
        }
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
                if (field_len == 0) break;
                if (pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];
                if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
                    if (name_len > (int)sizeof(name) - 1) name_len = (int)sizeof(name) - 1;
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
                    while (*p != '\0' && *p != '|' && si < (int)sizeof(surface)-1) {
                        surface[si++] = *p++;
                    }
                    surface[si] = '\0';
                    char combined[MAX_DATA_LEN];
                    snprintf(combined, sizeof(combined), "%s,%s", addr, surface);
                    process_received_data(combined);
                }
                if (strncmp(name, "ME", 2) == 0) register_mag_end_from_child(addr);
                if (strncmp(name, "READY", 5) == 0) register_child_ready(addr);
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    enable = 0x00; cmd[0] = enable; cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);
    close(sock);
    return NULL;
}

// ==========================================================
// キーリスト読み込み
// ==========================================================
int load_key_list(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) { perror("キーリストファイルを開けませんでした"); return -1; }
    g_node_count = 0;
    while (g_node_count < MAX_NODES) {
        int key; char addr[18];
        if (fscanf(fp, "%d %17s", &key, addr) != 2) break;
        g_nodes[g_node_count].key = key;
        strncpy(g_nodes[g_node_count].addr, addr, sizeof(g_nodes[g_node_count].addr));
        g_nodes[g_node_count].addr[sizeof(g_nodes[g_node_count].addr) - 1] = '\0';
        g_nodes[g_node_count].x = g_nodes[g_node_count].y = g_nodes[g_node_count].z = -1;
        g_nodes[g_node_count].index = g_node_count;
        g_node_count++;
    }
    fclose(fp);
    return g_node_count;
}

void print_key_list(void) {
    printf("\n--- Parent Key List (ascending) ---\n");
    for (int i = 0; i < g_node_count; i++) {
        printf("  [%d] key=%d addr=%s\n", i, g_nodes[i].key, g_nodes[i].addr);
    }
    printf("-----------------------------------\n");
}

// ==========================================================
// mag_sequence_thread: 電磁石タイムシーケンス
// ==========================================================
void *mag_sequence_thread(void *arg) {
    (void)arg;
    if (g_node_count <= 0) return NULL;

    if (g_total_children > 0) {
        printf("[MAG] %d 台の子機からの READY を待ちます...\n", g_total_children);
        pthread_mutex_lock(&ready_mutex);
        while (g_ready_count < g_total_children) {
            pthread_cond_wait(&ready_cond, &ready_mutex);
        }
        pthread_mutex_unlock(&ready_mutex);
        printf("[MAG] 全ての子機が準備完了しました。\n");
    } else {
        printf("[MAG] 子機がいないため、準備待ちなしで電磁石タイムを開始します。\n");
    }

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) return NULL;
    int sock_adv = hci_open_dev(dev_id);
    if (sock_adv < 0) return NULL;

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(500 * 1.6);
    adv_params_cp.min_interval = htobs(interval);
    adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype = 0x00;
    adv_params_cp.chan_map = 0x07;
    if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[MAG] アドバタイズパラメータ設定に失敗しました"); close(sock_adv); return NULL;
    }

    for (int idx = 0; idx < g_node_count; idx++) {
        const char *target_addr = g_nodes[idx].addr;
        int key = g_nodes[idx].key;

        // --- 親機自身のフェーズ ---
        if (strcmp(target_addr, g_parent_addr) == 0) {
            printf("[MAG] key=%d addr=%s は親機自身です。\n", key, target_addr);
            pthread_mutex_lock(&mag_mutex);
            strncpy(current_mag_target, target_addr, sizeof(current_mag_target));
            current_mag_target[sizeof(current_mag_target) - 1] = '\0';
            mag_end_flag = 0; 
            g_parent_mag_active = 1;
            pthread_mutex_unlock(&mag_mutex);

            printf("[MAG] 親機の電磁石を駆動中 (20秒間)...\n");
            
            // 電磁石ON
            if (gpio_init_and_on() != 0) {
                // 失敗時リカバリ
                pthread_mutex_lock(&mag_mutex);
                current_mag_target[0] = '\0';
                g_parent_mag_active = 0;
                pthread_mutex_unlock(&mag_mutex);
                
                // ★失敗時も必ずOFFを呼ぶ
                gpio_off_and_unexport();
                continue;
            }
            
            sleep(20);

            // 終了処理
            pthread_mutex_lock(&mag_mutex);
            current_mag_target[0] = '\0';
            g_parent_mag_active = 0;
            pthread_mutex_unlock(&mag_mutex);
            
            printf("[MAG] 親機の電磁石を停止します (Signal OFF)。\n");
            // ★確実にOFFにする
            gpio_off_and_unexport();
            
            printf("[MAG] 親機駆動終了。クールダウン5秒。\n");
            sleep(5);
            continue; 
        }

        // --- 子機フェーズ ---
        pthread_mutex_lock(&mag_mutex);
        strncpy(current_mag_target, target_addr, sizeof(current_mag_target));
        current_mag_target[sizeof(current_mag_target) - 1] = '\0';
        mag_end_flag = 0;
        pthread_mutex_unlock(&mag_mutex);

        uint8_t adv_data[31];
        memset(adv_data, 0, sizeof(adv_data));
        int len = 0;
        adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

        char name_field[32];
        snprintf(name_field, sizeof(name_field), "MT:%s", target_addr);
        int name_len = (int)strlen(name_field);
        if (name_len > 26) name_len = 26;
        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09; 
        memcpy(&adv_data[len], name_field, name_len);
        len += name_len;

        struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;
        adv_data_cp_struct.length = (uint8_t)len;
        memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
        memcpy(adv_data_cp_struct.data, adv_data, len);

        if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_data_cp_struct) < 0) continue;

        uint8_t enable = 0x01;
        if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable) < 0) continue;

        printf("[MAG] key=%d addr=%s に対して 20 秒間 MT アドバタイズ開始\n", key, target_addr);
        sleep(20); 

        enable = 0x00;
        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

        printf("[MAG] addr=%s からの ME 通知を待機...\n", target_addr);
        pthread_mutex_lock(&mag_mutex);
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 40; 
        
        while (!mag_end_flag) {
            if (pthread_cond_timedwait(&mag_cond, &mag_mutex, &ts) == ETIMEDOUT) {
                fprintf(stderr, "[MAG] 警告: ME通知タイムアウト (addr=%s)。\n", target_addr);
                break;
            }
        }
        pthread_mutex_unlock(&mag_mutex);
        if (mag_end_flag) printf("[MAG] ME 通知を受信しました。\n");

        printf("[MAG] クールダウン5秒。\n");
        sleep(5);
    }

    pthread_mutex_lock(&mag_mutex);
    current_mag_target[0] = '\0';
    mag_end_flag = 0;
    pthread_mutex_unlock(&mag_mutex);

    close(sock_adv);
    printf("[MAG] 全ての子機への電磁石タイム通知が完了しました。\n");
    
    coordinate_mapping(); 
    return NULL;
}

// ==========================================================
// ★★★ BFS マッピング関連 ヘルパー関数 ★★★
// ==========================================================

int opposite_dir(int d){
    switch(d){
        case PX: return NX;
        case NX: return PX;
        case PY: return NY;
        case NY: return PY;
        case PZ: return NZ;
        case NZ: return PZ;
    }
    return -1;
}

void enqueue(int v){
    q[qt] = v;
    qt = (qt + 1) % QUEUE_MAX;
}
int dequeue(){
    int v = q[qh];
    qh = (qh + 1) % QUEUE_MAX;
    return v;
}
int queue_empty(){
    return qh == qt;
}

void dir_to_delta(int d, int *dx, int *dy, int *dz){
    *dx = *dy = *dz = 0;
    if(d == PX) *dx = +1;
    if(d == NX) *dx = -1;
    if(d == PY) *dy = +1;
    if(d == NY) *dy = -1;
    if(d == PZ) *dz = +1;
    if(d == NZ) *dz = -1;
}

// 接続設定: 双方向に設定する (Indexベース)
void set_neighbor(int id, DIR d, int target){
    if (id < 0 || id >= MAX_NODES || target < 0 || target >= MAX_NODES) return;
    
    mapping_nodes[id].neighbor[d] = target;
    if(target != -1) {
        int opp = opposite_dir(d);
        mapping_nodes[target].neighbor[opp] = id;
    }
}

// レポートの面情報文字列を DIR enum に変換
DIR surface_to_dir(const char *surface) {
    if (strcmp(surface, "RIGHT") == 0)  return PX; // +X (RIGHT)
    if (strcmp(surface, "LEFT") == 0)   return NX; // -X (LEFT)
    if (strcmp(surface, "FRONT") == 0)  return PY; // +Y (FRONT)
    if (strcmp(surface, "BACK") == 0)   return NY; // -Y (BACK)
    if (strcmp(surface, "TOP") == 0)    return PZ; // +Z (TOP)
    if (strcmp(surface, "BOTTOM") == 0) return NZ; // -Z (BOTTOM)
    return -1; 
}

void init_mapping_nodes(const char *parent_addr) {
    for(int i=0; i < MAX_NODES; i++){
        mapping_nodes[i].id = -1; // Indexを使用するためIDは補助
        mapping_nodes[i].x = mapping_nodes[i].y = mapping_nodes[i].z = INF;
        mapping_nodes[i].fixed = 0;
        for(int d=0; d<6; d++) mapping_nodes[i].neighbor[d] = -1;
    }

    // 親機ノードを探し、原点(0,0,0)に固定
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, parent_addr) == 0) {
            // Index 'i' を使う
            mapping_nodes[i].x = 0;
            mapping_nodes[i].y = 0;
            mapping_nodes[i].z = 0;
            mapping_nodes[i].fixed = 1;
            printf("[MAP-INIT] Parent Fixed at Index %d (Key: %d)\n", i, g_nodes[i].key);
            break;
        }
    }
}

void update_positions(void) {
    qh = qt = 0;
    // 位置が確定しているノードのみ開始点（親機のみ）
    for(int i = 0; i < MAX_NODES; i++){
        if(mapping_nodes[i].fixed){
            enqueue(i);
        }
    }

    while(!queue_empty()){
        int cur_idx = dequeue(); // Index
        for(int d = 0; d < 6; d++){
            int nb_idx = mapping_nodes[cur_idx].neighbor[d];
            if(nb_idx == -1) continue; // -1 チェック

            int dx, dy, dz;
            dir_to_delta(d, &dx, &dy, &dz);

            int nx = mapping_nodes[cur_idx].x + dx;
            int ny = mapping_nodes[cur_idx].y + dy;
            int nz = mapping_nodes[cur_idx].z + dz;

            if(mapping_nodes[nb_idx].fixed == 0){
                mapping_nodes[nb_idx].x = nx;
                mapping_nodes[nb_idx].y = ny;
                mapping_nodes[nb_idx].z = nz;
                mapping_nodes[nb_idx].fixed = 1;
                enqueue(nb_idx);
                printf("[MAP-BFS] Mapped Index %d at (%d, %d, %d) from Index %d (Dir: %d)\n",
                       nb_idx, nx, ny, nz, cur_idx, d);
            }
            else{
                if(mapping_nodes[nb_idx].x != nx ||
                   mapping_nodes[nb_idx].y != ny ||
                   mapping_nodes[nb_idx].z != nz){
                     printf("[MAP-WARNING] Conflict detected for Index %d. Existing (%d,%d,%d) vs New (%d,%d,%d). Ignored.\n",
                           nb_idx, mapping_nodes[nb_idx].x, mapping_nodes[nb_idx].y, mapping_nodes[nb_idx].z, nx, ny, nz);
                }
            }
        }
    }
}

// ==========================================================
// マッピング処理 (BFS版 + Index対応)
// ==========================================================
void coordinate_mapping(void) {
    printf("\n\n===== マッピング処理開始 (BFSベース / Index方式) =====\n");
    FILE *fp = fopen(MAP_FILE_PATH, "w");
    if (!fp) { perror("マップファイルを開けませんでした"); return; }
    fprintf(fp, "# Cube Mapping Log\n");

    // 1. ノード初期化と親機の固定
    init_mapping_nodes(g_parent_addr); 
    
    // 2. レポートからグラフ構築
    pthread_mutex_lock(&report_mutex);
    for (int r = 0; r < g_report_count; r++) {
        const char *target_addr = g_reports[r].target_addr;
        const char *detected_addr = g_reports[r].detected_addr;
        const char *surface = g_reports[r].surface;

        // KeyではなくIndexを取得する
        int target_idx = get_index_by_addr(target_addr);
        int detected_idx = get_index_by_addr(detected_addr);
        DIR dir = surface_to_dir(surface);

        if (target_idx != -1 && detected_idx != -1 && dir != -1) {
            // ★論理修正: 検出側(Detected)の Dir 側に Target がある
            set_neighbor(detected_idx, dir, target_idx);
            
            // ログ用
            int t_key = g_nodes[target_idx].key;
            int d_key = g_nodes[detected_idx].key;
            printf("[MAP-Graph] Index %d(Key:%d) -> Dir %d -> Index %d(Key:%d)\n", 
                   detected_idx, d_key, dir, target_idx, t_key);
        } else {
            fprintf(stderr, "[MAP-WARN] Invalid Report skipped: T=%s, D=%s\n", target_addr, detected_addr);
        }
    }
    pthread_mutex_unlock(&report_mutex);
    
    // 3. BFS実行
    update_positions(); 

    // 4. 結果出力 (ファイル & 画面)
    fprintf(fp, "\n# Final Cube Coordinates\n");
    printf("\n--- 最終座標結果 ---\n");
    
    int mapped_count = 0;
    for (int i = 0; i < g_node_count; i++) {
        // Index i を使用
        if (mapping_nodes[i].fixed) {
            g_nodes[i].x = mapping_nodes[i].x;
            g_nodes[i].y = mapping_nodes[i].y;
            g_nodes[i].z = mapping_nodes[i].z;
            mapped_count++;
        } else {
            g_nodes[i].x = g_nodes[i].y = g_nodes[i].z = -1;
        }
        
        fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n", 
                g_nodes[i].addr, g_nodes[i].key, g_nodes[i].x, g_nodes[i].y, g_nodes[i].z);
        
        // ★画面へそのまま出力
        printf("[%s] Key: %d, Coords: (%d, %d, %d)\n", 
                g_nodes[i].addr, g_nodes[i].key, g_nodes[i].x, g_nodes[i].y, g_nodes[i].z);
    }
    printf("[MAP] マッピングされたノード数: %d / %d\n", mapped_count, g_node_count);
    
    fprintf(fp, "\n# Raw Reports (Target | Detected | Surface)\n");
    pthread_mutex_lock(&report_mutex);
    for (int r = 0; r < g_report_count; r++) {
        fprintf(fp, "%s | %s | %s\n", g_reports[r].target_addr, g_reports[r].detected_addr, g_reports[r].surface);
    }
    pthread_mutex_unlock(&report_mutex);

    fclose(fp);
    printf("===== マッピング処理完了。結果は %s に出力されました。 =====\n", MAP_FILE_PATH);
}


// ==========================================================
// parent.c main 関数
// ==========================================================
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <parent_full_address>\n", argv[0]);
        return 1;
    }
    const char *parent_addr = PARENT_ADDR_ARG;
    strncpy(g_parent_addr, parent_addr, sizeof(g_parent_addr));
    g_parent_addr[sizeof(g_parent_addr)-1] = '\0';

    printf("\n==================================\n");
    printf("👑 親機プログラム開始 (PARENT PROGRAM STARTING)\n");
    printf("==================================\n");
    printf("確定した親機アドレス: %s\n", parent_addr);

    if (wiringPiSetup() != 0) { fprintf(stderr, "wiringPiSetup failed.\n"); return 1; }
    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) { fprintf(stderr, "wiringPiSPISetup failed.\n"); return 1; }
    if (i2c_open() < 0 || bno055_init_ndof() < 0) { fprintf(stderr, "BNO055 init failed on parent.\n"); return 1; }
    
    // GPIO Base 設定
    int base = readInt("/sys/class/gpio/gpiochip512/base");
    if (base < 0) {
        fprintf(stderr, "ERROR: Failed to read GPIO base. Check /sys/class/gpio/gpiochip*/base.\n");
        close(i2c_fd); return 1;
    }
    g_linux_gpio = base + BCM_GPIO_PIN;
    printf("Resolved Parent GPIO Pin: %d (BCM %d)\n", g_linux_gpio, BCM_GPIO_PIN);

    if (load_heading_offset(OFFSET_FILE, &g_heading_offset) != 0) {
        printf("ERROR: Heading offset file not loaded. Aborting.\n"); close(i2c_fd); return 1;
    }

    printf("Parent Baseline sampling...\n");
    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) g_baseline[ch] += read_adc_voltage(ch);
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) g_baseline[ch] /= BASELINE_SAMPLES;
    printf("Parent Baseline established.\n");

    int loaded = load_key_list(KEY_LIST_FILE);
    if (loaded > 0) {
        print_key_list(); 
        memset(g_child_ready, 0, sizeof(g_child_ready));
        g_ready_count = 0;
        g_total_children = 0;
        for (int i = 0; i < g_node_count; i++) {
            if (strcmp(g_nodes[i].addr, parent_addr) != 0) g_total_children++;
        }
        printf("\n--- 電磁石タイム ---\n");
        printf("子機の準備完了(READY)を %d 台分待ちます。\n", g_total_children);
    } else {
        printf("[WARN] %s が読み込めなかったため、キー情報なしで起動します。\n", KEY_LIST_FILE);
    }

    pthread_t rx_thread;
    if (pthread_create(&rx_thread, NULL, BLE_receive_data_server, NULL) != 0) {
        perror("BLE受信スレッドの生成に失敗しました"); return 1;
    }
    
    pthread_t sensor_th;
    if (pthread_create(&sensor_th, NULL, parent_sensor_monitor, NULL) != 0) {
        perror("センサー監視スレッドの生成に失敗しました"); return 1;
    }

    pthread_t mag_thread;
    if (loaded > 0) {
        if (pthread_create(&mag_thread, NULL, mag_sequence_thread, NULL) != 0) {
            perror("電磁石タイムシーケンススレッドの生成に失敗しました");
        }
    }

    printf("親機メイン処理を実行中...\n");
    
    if (loaded > 0) {
        pthread_join(mag_thread, NULL);
    }

    printf("電磁石タイムシーケンスが完了しました。受信スレッドは継続します (Ctrl+Cで終了)。\n");
    
    // ★プログラム終了時も必ずOFF信号を送る
    gpio_off_and_unexport(); 

    return 0;
}
