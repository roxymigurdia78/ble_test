// parent.c (エラー修正済み)

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
#define MAP_FILE_PATH   "mapping.txt"
#define KEY_LIST_FILE   "parent_key_list.txt"
#define MAX_DATA_LEN    64
#define MAX_NODES       32

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
#define OPERATION_MODE_NDOF    0x0C
#define EULER_UNIT             16.0f
const char *OFFSET_FILE  = "bno055_heading_offset.txt";

#define DIFF_THRESHOLD        0.020
#define SECOND_MARGIN         0.0
#define STABLE_COUNT_REQUIRED 1

#define BASELINE_SAMPLES 50

static int i2c_fd = -1;
static float g_heading_offset = 0.0f;
static double g_baseline[NUM_CH] = {0};

static volatile int g_parent_mag_active = 0;

const int BCM_GPIO_PIN = 20;
static int g_linux_gpio = -1;

// ==========================================================
// GPIO 制御ヘルパー関数 (変更なし)
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

    writeFileSilent("/sys/class/gpio/unexport", num_str);
    usleep(100000);

    if (writeFile("/sys/class/gpio/export", num_str) < 0) {
         fprintf(stderr, "ERROR: Failed to export GPIO %d.\n", g_linux_gpio);
         return -1;
    }
    usleep(100000);

    if (writeFile(path_dir, "out") < 0) {
        fprintf(stderr, "ERROR: Failed to set direction to 'out'.\n");
        return -1;
    }

    if (writeFile(path_val, "1") < 0) {
        fprintf(stderr, "ERROR: Failed to write '1' to value.\n");
        return -1;
    }
    printf("[GPIO] Parent Electromagnet ON (GPIO %d)\n", g_linux_gpio);
    return 0;
}

static void gpio_off_and_unexport(void) {
    char num_str[16];
    char path_val[128];

    if (g_linux_gpio == -1) return;

    snprintf(num_str, sizeof(num_str), "%d", g_linux_gpio);
    snprintf(path_val, sizeof(path_val), "/sys/class/gpio/gpio%d/value", g_linux_gpio);

    if (fileExists(path_val)) {
        writeFileSilent(path_val, "0");
        writeFileSilent("/sys/class/gpio/unexport", num_str);
        printf("[GPIO] Parent Electromagnet OFF & Unexport (GPIO %d)\n", g_linux_gpio);
    }
}

// ==========================================================
// キーリスト/状態管理 構造体 (変更なし)
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

#define MAX_REPORTS 256
static ReportEntry g_reports[MAX_REPORTS];
static int g_report_count = 0;
static pthread_mutex_t report_mutex = PTHREAD_MUTEX_INITIALIZER;

// プロトタイプ
int  coordinate_mapping(void);
int  get_key_by_addr(const char *addr);
int  get_index_by_addr(const char *addr);
void register_child_ready(const char *child_addr);
void process_received_data(const char *received_data);
void *parent_sensor_monitor(void *arg);
int  load_key_list(const char *path);
void print_key_list(void);
int  BLE_send_map_result(int map_result);

// ==========================================================
// センサー関連ヘルパー (変更なし)
// ==========================================================
static void msleep(int ms) { usleep(ms * 1000); }

static int i2c_open(void) {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) {
        perror("open(/dev/i2c-1)");
        return -1;
    }
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) {
        perror("ioctl(I2C_SLAVE)");
        close(i2c_fd);
        i2c_fd = -1;
        return -1;
    }
    return 0;
}

static int i2c_write8(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = { reg, value };
    if (write(i2c_fd, buf, 2) != 2) {
        perror("i2c_write8");
        return -1;
    }
    return 0;
}

static int i2c_read_len(uint8_t reg, uint8_t *buf, int len) {
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("i2c_read_len:write(reg)");
        return -1;
    }
    if (read(i2c_fd, buf, len) != len) {
        perror("i2c_read_len:read");
        return -1;
    }
    return 0;
}

static int bno055_set_mode(uint8_t mode) {
    if (i2c_write8(BNO055_OPR_MODE_ADDR, mode) < 0) return -1;
    msleep(30);
    return 0;
}

static int bno055_init_ndof(void) {
    uint8_t id;
    if (i2c_read_len(BNO055_CHIP_ID_ADDR, &id, 1) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        return -1;
    }
    if (id != BNO055_ID_EXPECTED) {
        fprintf(stderr, "Unexpected BNO055 ID: 0x%02X\n", id);
        return -1;
    }
    printf("BNO055 detected! ID=0x%02X\n", id);

    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "Failed to set NDOF mode\n");
        return -1;
    }
    msleep(50);
    printf("BNO055 initialized in NDOF mode.\n");
    return 0;
}

float read_bno055_heading(int fd, float heading_offset) {
    (void)fd;
    uint8_t buf[2];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 2) < 0) {
        return NAN;
    }
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
    if (fscanf(fp, "%f", offset) != 1) {
        fclose(fp);
        return -1;
    }
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
    double voltage = (double)value * 3.3 / 1023.0;
    return voltage;
}

// ==========================================================
// 共通ヘルパー (変更なし)
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

void register_child_ready(const char *child_addr) {
    pthread_mutex_lock(&ready_mutex);

    if (g_total_children <= 0) {
        pthread_mutex_unlock(&ready_mutex);
        return;
    }

    if (g_parent_addr[0] != '\0' && strcmp(child_addr, g_parent_addr) == 0) {
        pthread_mutex_unlock(&ready_mutex);
        return;
    }

    int idx = get_index_by_addr(child_addr);
    if (idx < 0 || idx >= MAX_NODES) {
        pthread_mutex_unlock(&ready_mutex);
        return;
    }

    if (!g_child_ready[idx]) {
        g_child_ready[idx] = 1;
        g_ready_count++;
        printf("送ってきた個体のアドレス：%s 準備OK！（%d/%d）\n", child_addr, g_ready_count, g_total_children);

        if (g_ready_count >= g_total_children) {
            pthread_cond_broadcast(&ready_cond);
        }
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
               child_addr, child_key, surface,
               current_mag_target[0] != '\0' ? current_mag_target : "None");

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
    }
}

// ==========================================================
// 親機のホールセンサー常時監視スレッド (変更なし)
// ==========================================================
void *parent_sensor_monitor(void *arg) {
    (void)arg;
    printf("[SENSOR] 親機センサー監視スレッド開始。\n");

    static char last_surface[16] = "";
    static int  same_count = 0;

    while (1) {
        if (g_parent_mag_active) {
            usleep(100000);
            continue;
        }
        if (current_mag_target[0] == '\0') {
             usleep(100000);
             continue;
        }

        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) {
            window[ch] = read_adc_voltage(ch);
        }

        double maxDiff    = 0.0;
        int    maxCh      = -1;

        for (int ch = 1; ch < NUM_CH; ch++) {
            if (g_baseline[ch] < 1.0 || window[ch] < 1.0) continue;
            double diff = fabs(window[ch] - g_baseline[ch]);
            if (diff > maxDiff) {
                maxDiff = diff;
                maxCh   = ch;
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

                if (strcmp(last_surface, current_surface) == 0) {
                    same_count++;
                } else {
                    strncpy(last_surface, current_surface, sizeof(last_surface));
                    last_surface[sizeof(last_surface)-1] = '\0';
                    same_count = 1;
                }

                if (same_count >= STABLE_COUNT_REQUIRED) {
                    printf("\n[PARENT EVENT] CH%d detected -> Surface: **%s**\n", maxCh, current_surface);

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
// BLE 受信スレッド (エラー修正: adv_params_cp 関連を削除)
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
    scan_params_cp.type            = 0x01;
    scan_params_cp.interval        = htobs(0x0010);
    scan_params_cp.window          = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00;
    scan_params_cp.filter          = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);
    
    // エラー修正: 以下の古い広告設定コードを削除
    /*
    uint16_t interval = (uint16_t)(500 * 1.6);
    adv_params_cp.min_interval = htobs(interval);
    adv_params_cp.max_interval = htobs(interval);
    adv_params_cp.advtype      = 0x00;
    adv_params_cp.chan_map     = 0x07;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);
    */

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
            char addr[18]; ba2str(&info->bdaddr, addr);
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
// parent_key_list.txt 読み込み (変更なし)
// ==========================================================
int load_key_list(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) { perror("キーリスト読み込み失敗"); return -1; }

    g_node_count = 0;
    while (g_node_count < MAX_NODES) {
        int key; char addr[18];
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
// mag_sequence_thread: 電磁石タイムシーケンス (エラー修正: sock -> sock_adv)
// ==========================================================
#define CHILD_MAG_COOLDOWN_SEC 10 // ★ 25秒から10秒に短縮

void *mag_sequence_thread(void *arg) {
    (void)arg;

    if (g_node_count <= 0) return NULL;

    // READY 待機 (変更なし)
    if (g_total_children > 0) {
        printf("[MAG] %d 台の子機のREADYを待機...\n", g_total_children);
        pthread_mutex_lock(&ready_mutex);
        while (g_ready_count < g_total_children) {
            pthread_cond_wait(&ready_cond, &ready_mutex);
        }
        pthread_mutex_unlock(&ready_mutex);

        // ★ 全子機 READY 受信後、さらに 10秒待機
        printf("[MAG] 全子機準備完了。10秒待機してからシーケンスを開始します。\n");
        sleep(10);
        printf("[MAG] 待機完了。シーケンス開始。\n");
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
    adv_params_cp.advtype      = 0x00;
    adv_params_cp.chan_map     = 0x07;
    hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    for (int idx = 0; idx < g_node_count; idx++) {
        const char *target_addr = g_nodes[idx].addr;
        int key = g_nodes[idx].key;

        // --- 親機自身のフェーズ ---
        if (strcmp(target_addr, g_parent_addr) == 0) {
            printf("[MAG] 親機(key=%d) 電磁石駆動フェーズ\n", key);

            pthread_mutex_lock(&mag_mutex);
            strncpy(current_mag_target, target_addr, 18);
            g_parent_mag_active = 1;
            pthread_mutex_unlock(&mag_mutex);

            if (gpio_init_and_on() != 0) {
                pthread_mutex_lock(&mag_mutex);
                current_mag_target[0] = '\0';
                g_parent_mag_active = 0;
                pthread_mutex_unlock(&mag_mutex);
                continue;
            }
            sleep(20);

            pthread_mutex_lock(&mag_mutex);
            current_mag_target[0] = '\0';
            g_parent_mag_active = 0;
            pthread_mutex_unlock(&mag_mutex);

            gpio_off_and_unexport();
            printf("[MAG] 親機駆動終了。クールダウン5秒。\n");
            sleep(5);
            continue;
        }

        // --- 子機フェーズ (MT広告) ---
        pthread_mutex_lock(&mag_mutex);
        strncpy(current_mag_target, target_addr, 18);
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

        struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_cp;
        adv_cp.length = (uint8_t)len;
        memcpy(adv_cp.data, adv_data, len);

        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_cp);
        uint8_t enable = 0x01;
        // エラー修正: sock -> sock_adv
        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

        printf("[MAG] 子機(key=%d) MT開始 (20秒)\n", key);
        sleep(20);

        enable = 0x00;
        // エラー修正: sock -> sock_adv
        hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
        printf("[MAG] MT広告停止。\n");

        // ★ クールダウン時間を10秒に短縮
        printf("[MAG] 子機の電磁石駆動・クールダウン期間(合計約 %d秒)を待機中...\n", CHILD_MAG_COOLDOWN_SEC);
        sleep(CHILD_MAG_COOLDOWN_SEC);

        pthread_mutex_lock(&mag_mutex);
        current_mag_target[0] = '\0';
        pthread_mutex_unlock(&mag_mutex);

        printf("[MAG] 次のターゲットへ移行。\n");
    }

    pthread_mutex_lock(&mag_mutex);
    current_mag_target[0] = '\0';
    pthread_mutex_unlock(&mag_mutex);

    close(sock_adv);
    printf("[MAG] 全シーケンス完了。\n");

    return NULL;
}

// ==========================================================
// マッピング処理 (変更なし)
// ==========================================================
int coordinate_mapping(void) {
    printf("\n\n===== マッピング処理開始 =====\n");
    FILE *fp = fopen(MAP_FILE_PATH, "w");
    if (!fp) { perror("Map file error"); return 1; }
    fprintf(fp, "# Cube Mapping Log\n");

    int parent_index = -1;
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, g_parent_addr) == 0) {
            g_nodes[i].x = 0; g_nodes[i].y = 0; g_nodes[i].z = 0;
            parent_index = i;
            break;
        }
    }

    if (parent_index == -1) {
        fprintf(stderr, "[MAP] 親機情報なし。失敗。\n");
        fclose(fp);
        return 1;
    }

    int mapped_count = 1;
    while (mapped_count < g_node_count) {
        int new_mapped = 0;

        for (int i = 0; i < g_node_count; i++) {
            if (g_nodes[i].x == -1) continue;
            int cx = g_nodes[i].x, cy = g_nodes[i].y, cz = g_nodes[i].z;

            pthread_mutex_lock(&report_mutex);
            for (int r = 0; r < g_report_count; r++) {
                if (strcmp(g_reports[r].target_addr, g_nodes[i].addr) == 0) {
                    const char *det_addr = g_reports[r].detected_addr;
                    const char *surf = g_reports[r].surface;

                    int det_idx = -1;
                    for (int j = 0; j < g_node_count; j++) {
                        if (strcmp(g_nodes[j].addr, det_addr) == 0) { det_idx = j; break; }
                    }

                    if (det_idx != -1 && g_nodes[det_idx].x == -1) {
                        int nx = cx, ny = cy, nz = cz;
                        if (strcmp(surf, "FRONT") == 0)      ny += 1;
                        else if (strcmp(surf, "BACK") == 0)  ny -= 1;
                        else if (strcmp(surf, "LEFT") == 0)  nx -= 1;
                        else if (strcmp(surf, "RIGHT") == 0) nx += 1;
                        else if (strcmp(surf, "TOP") == 0)   nz += 1;
                        else if (strcmp(surf, "BOTTOM") == 0)nz -= 1;

                        g_nodes[det_idx].x = nx; g_nodes[det_idx].y = ny; g_nodes[det_idx].z = nz;
                        new_mapped++; mapped_count++;
                        printf("[MAP] Mapped %s at (%d, %d, %d) from %s (%s)\n",
                               det_addr, nx, ny, nz, g_nodes[i].addr, surf);
                    }
                }
            }
            pthread_mutex_unlock(&report_mutex);
        }
        if (new_mapped == 0 && mapped_count < g_node_count) {
            printf("[MAP] これ以上マッピング不可。\n");
            break;
        }
    }

    fprintf(fp, "\n# Final Cube Coordinates\n");
    for (int i = 0; i < g_node_count; i++) {
        fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n",
                g_nodes[i].addr, g_nodes[i].key, g_nodes[i].x, g_nodes[i].y, g_nodes[i].z);
    }

    fprintf(fp, "\n# Raw Reports\n");
    pthread_mutex_lock(&report_mutex);
    for (int r = 0; r < g_report_count; r++) {
        fprintf(fp, "%s | %s | %s\n",
                g_reports[r].target_addr, g_reports[r].detected_addr, g_reports[r].surface);
    }
    pthread_mutex_unlock(&report_mutex);
    fclose(fp);

    if (mapped_count < g_node_count) {
        printf("❌ [FAILURE] マッピング不完全 (%d/%d)。\n", mapped_count, g_node_count);
        return 1;
    }
    printf("✅ [SUCCESS] 全ノードマッピング完了。\n");
    return 0;
}

// ==========================================================
// MACアドレス圧縮: "AA:BB:CC:DD:EE:FF" → "AABBCCDDEEFF" (変更なし)
// ==========================================================
static void compress_addr_to_hex12(const char *addr, char *out, size_t out_size)
{
    if (!out || out_size == 0) return;
    if (!addr) {
        snprintf(out, out_size, "000000000000");
        return;
    }
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

// ==========================================================
// マッピング結果を配布 (MKフレーム & MAP_END) (変更なし)
// ==========================================================
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
    adv_params_cp.advtype      = 0x00;
    adv_params_cp.chan_map     = 0x07;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    uint8_t adv_data[31];
    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_cp;
    uint8_t enable;

    // 1) MKフレーム
    for (int i = 0; i < g_node_count; i++) {
        int key = g_nodes[i].key;
        int x   = g_nodes[i].x;
        int y   = g_nodes[i].y;
        int z   = g_nodes[i].z;

        char addr_hex[13];
        compress_addr_to_hex12(g_nodes[i].addr, addr_hex, sizeof(addr_hex));

        // 形式: "MK:<key>:<MAC12>:x,y,z"
        char name_field[32];
        snprintf(name_field, sizeof(name_field), "MK:%d:%s:%d,%d,%d",
                 key, addr_hex, x, y, z);

        int len = 0;
        memset(adv_data, 0, sizeof(adv_data));
        adv_data[len++] = 2;
        adv_data[len++] = 0x01;
        adv_data[len++] = 0x06;

        int name_len = (int)strlen(name_field);
        if (name_len > 26) name_len = 26;
        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09;
        memcpy(&adv_data[len], name_field, name_len);
        len += name_len;

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

    // 2) MAP_END
    char name_field[32];
    snprintf(name_field, sizeof(name_field), "MAP_END:%s", map_result == 0 ? "SUCCESS" : "FAILURE");

    int len = 0;
    memset(adv_data, 0, sizeof(adv_data));
    adv_data[len++] = 2;
    adv_data[len++] = 0x01;
    adv_data[len++] = 0x06;

    int name_len = (int)strlen(name_field);
    if (name_len > 26) name_len = 26;
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(&adv_data[len], name_field, name_len);
    len += name_len;

    adv_cp.length = (uint8_t)len;
    memcpy(adv_cp.data, adv_data, len);

    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_cp);
    enable = 0x01;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

    printf("[COMM] MAP END advertising start (10 seconds)...\n");
    sleep(10);

    enable = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
    printf("[COMM] MAP END advertising stop.\n");

    close(sock);
    return 0;
}

// ==========================================================
// main (変更なし)
// ==========================================================
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <parent_full_address>\n", argv[0]);
        return 1;
    }
    const char *parent_addr = PARENT_ADDR_ARG;
    strncpy(g_parent_addr, parent_addr, 18);
    g_parent_addr[17] = '\0';

    printf("\n=== PARENT PROGRAM START ===\n");

    if (wiringPiSetup() != 0) return 1;
    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) return 1;
    if (i2c_open() < 0 || bno055_init_ndof() < 0) return 1;

    int base = readInt("/sys/class/gpio/gpiochip512/base");
    if (base < 0) { close(i2c_fd); return 1; }
    g_linux_gpio = base + BCM_GPIO_PIN;

    if (load_heading_offset(OFFSET_FILE, &g_heading_offset) != 0) {
        fprintf(stderr, "ERROR: Heading offset file not loaded.\n");
        close(i2c_fd);
        return 1;
    }

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

    pthread_t rx_thread, sensor_th, mag_thread;
    pthread_create(&rx_thread, NULL, BLE_receive_data_server, NULL);
    pthread_create(&sensor_th, NULL, parent_sensor_monitor, NULL);

    if (loaded > 0) {
        pthread_create(&mag_thread, NULL, mag_sequence_thread, NULL);
        pthread_join(mag_thread, NULL);
    }

    printf("シーケンス完了。マッピング計算...\n");

    int map_result = 0;
    if (loaded > 0) {
        map_result = coordinate_mapping();
    } else {
        printf("[WARN] キーリストなし。\n");
        map_result = 1;
    }

    BLE_send_map_result(map_result);

    gpio_off_and_unexport();

    printf("親機終了 (Exit Code: %d)\n", map_result);
    return map_result;
}
