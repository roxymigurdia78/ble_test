// child.c
// ★ 修正: ログ出力の削減 (`[EVENT CANDIDATE]`, BLE ADV詳細ログを削除)。
//          センサー検知ロジックは毎回実行に戻す。

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

// 自分のアドレス（スキャンスレッドから参照）
static char g_my_addr[18] = "(unknown)";

// 電磁石タイム要求フラグ
static volatile int electromagnet_requested = 0;
static volatile int electromagnet_active    = 0;
static pthread_mutex_t mag_state_mutex = PTHREAD_MUTEX_INITIALIZER;

// マッピング終了フラグ
static volatile int map_end_received = 0;

// フェーズ: 0=MT待ち, 1=MAP 配布受信
static volatile int g_scan_phase = 0;

// GPIO
const int BCM_GPIO_PIN = 20;
static int g_linux_gpio = -1;

// ==========================================================
// GPIO 制御ヘルパー (変更なし)
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
    if (fscanf(f, "%d", &v) != 1) {
        fprintf(stderr, "failed to read int from %s\n", path);
        fclose(f);
        return -1;
    }
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
         fprintf(stderr, "ERROR: Failed to export GPIO %d. (Need sudo?)\n", g_linux_gpio);
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
    printf("[GPIO] 子機電磁石 ON (GPIO %d)\n", g_linux_gpio);
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
        printf("[GPIO] 子機電磁石 OFF & Unexport (GPIO %d)\n", g_linux_gpio);
    }
}

// ==========================================================
// BLE送信ユーティリティ (ログ出力削減)
// ==========================================================
// 短時間の広告を設定するヘルパー (変更なし)
static int setup_adv_parameters(int sock, uint16_t interval_ms) {
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(interval_ms * 1.6);
    adv_params_cp.min_interval     = htobs(interval);
    adv_params_cp.max_interval     = htobs(interval);
    adv_params_cp.advtype          = 0x00;
    adv_params_cp.own_bdaddr_type  = 0x00;
    adv_params_cp.chan_map         = 0x07;
    adv_params_cp.filter           = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[BLE] Failed to set advertising parameters");
        return -1;
    }
    return 0;
}

// Surface-reportを複数回ブロードキャスト (ログ削減)
int BLE_send_surface_data(const char *my_addr,
                          const char *parent_addr,
                          const char *surface_name)
{
    (void)parent_addr;

    // ★ ログ削減: 広告開始メッセージのみ残す
    printf("[COMM] Attempting BLE ADV send: Surface=%s\n", surface_name);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route"); return -1; }
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev"); return -1; }

    if (setup_adv_parameters(sock, 100) < 0) { // 100ms間隔で設定
        close(sock);
        return -1;
    }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;
    adv_data[len++] = 2;
    adv_data[len++] = 0x01;
    adv_data[len++] = 0x06;

    char name_field[31];
    snprintf(name_field, sizeof(name_field),
             "CubeNode|SURFACE:%s", surface_name);

    int name_len = (int)strlen(name_field);
    if (name_len > 29) name_len = 29;

    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(&adv_data[len], name_field, name_len);
    len += name_len;

    struct {
        uint8_t length;
        uint8_t data[31];
    } __attribute__((packed)) adv_data_cp_struct;

    adv_data_cp_struct.length = (uint8_t)len;
    memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
    memcpy(adv_data_cp_struct.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp_struct) < 0) {
        perror("[BLE] Failed to set advertising data");
        close(sock);
        return -1;
    }

    // 広告を複数回送信 (100msごとに5回)
    for (int i = 0; i < 5; i++) {
        uint8_t enable = 0x01;
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                         1, &enable) < 0) {
            perror("[BLE] Failed to enable advertising");
            close(sock);
            return -1;
        }
        usleep(100000); // 100ms 広告
        uint8_t disable = 0x00;
        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable);
        usleep(50000); // 50ms 停止
    }
    
    // ★ ログ削減: 広告停止メッセージを削除
    close(sock);
    return 0;
}

// READY: 10秒間、複数回アドバタイズ (ログ削減)
int BLE_send_ready(const char *my_addr) {
    // ★ ログ削減: 広告開始メッセージを簡略化
    printf("[COMM] Sending READY advertise.\n");

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) { perror("[BLE] hci_get_route (READY)"); return -1; }
    int sock = hci_open_dev(dev_id);
    if (sock < 0) { perror("[BLE] hci_open_dev (READY)"); return -1; }

    if (setup_adv_parameters(sock, 100) < 0) { // 100ms間隔で設定
        close(sock);
        return -1;
    }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;
    adv_data[len++] = 2;
    adv_data[len++] = 0x01;
    adv_data[len++] = 0x06;

    const char *name_field = "READY";
    int name_len = (int)strlen(name_field);
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09;
    memcpy(adv_data + len, name_field, name_len);
    len += name_len;

    struct {
        uint8_t length;
        uint8_t data[31];
    } __attribute__((packed)) adv_data_cp_struct;

    adv_data_cp_struct.length = (uint8_t)len;
    memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
    memcpy(adv_data_cp_struct.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp_struct) < 0) {
        perror("[BLE] Failed to set advertising data (READY)");
        close(sock);
        return -1;
    }

    // 10秒間、短いサイクルで広告をON/OFF
    time_t start_time = time(NULL);
    uint8_t enable = 0x01;
    uint8_t disable = 0x00;

    while (time(NULL) - start_time < 10) {
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable) < 0) {
            perror("[BLE] Failed to enable advertising (READY)");
            break;
        }
        usleep(100000); // 100ms 広告
        
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &disable) < 0) {
             perror("[BLE] Failed to disable advertising (READY)");
             break;
        }
        usleep(50000); // 50ms 停止
    }

    // ★ ログ削減: 広告終了メッセージを簡略化
    printf("[COMM] READY advertising finished.\n");
    close(sock);
    return 0;
}

// ==========================================================
// BNO055/ADC (変更なし)
// ==========================================================
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7
#define DIFF_THRESHOLD        0.020
#define STABLE_COUNT_REQUIRED 1
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

static void msleep(int ms) { usleep(ms * 1000); }

static int i2c_open(void) {
    i2c_fd = open(I2C_DEV_PATH, O_RDWR);
    if (i2c_fd < 0) { perror("open(/dev/i2c-1)"); return -1; }
    if (ioctl(i2c_fd, I2C_SLAVE, BNO055_ADDRESS) < 0) { perror("ioctl(I2C_SLAVE)"); close(i2c_fd); i2c_fd = -1; return -1; }
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
    data[0] = 1;
    data[1] = (8 + ch) << 4;
    data[2] = 0;
    wiringPiSPIDataRW(SPI_CH, data, 3);
    int value = ((data[1] & 3) << 8) | data[2];
    double voltage = (double)value * 3.3 / 1023.0;
    return voltage;
}

// ==========================================================
// 子機側マッピング保存構造体 (変更なし)
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

static void child_map_init(void) {
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        g_child_map[i].valid = 0;
        g_child_map[i].key   = 0;
        g_child_map[i].addr[0] = '\0';
        g_child_map[i].x = g_child_map[i].y = g_child_map[i].z = 0;
    }
}

// "DCA6329A77A1" → "DC:A6:32:9A:77:A1" (変更なし)
static void expand_compact_addr12(const char *compact, char *out, size_t out_size)
{
    if (!out || out_size == 0) return;

    if (!compact) {
        snprintf(out, out_size, "UNKNOWN");
        return;
    }
    size_t len = strlen(compact);
    if (len < 12) {
        snprintf(out, out_size, "%s", compact);
        return;
    }

    char buf[18];
    int j = 0;
    for (int i = 0; i < 12 && j < 17; i += 2) {
        buf[j++] = compact[i];
        if (j >= 17) break;
        buf[j++] = compact[i + 1];
        if (i != 10 && j < 17) {
            buf[j++] = ':';
        }
    }
    buf[j] = '\0';
    snprintf(out, out_size, "%s", buf);
}

// 新しい key or 座標を初めて見たときだけ printf (変更なし)
static void child_store_map_frame(int key,
                                  const char *addr,
                                  int x, int y, int z)
{
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid && g_child_map[i].key == key) {
            if (g_child_map[i].x == x &&
                g_child_map[i].y == y &&
                g_child_map[i].z == z) {
                return;
            } else {
                g_child_map[i].x = x;
                g_child_map[i].y = y;
                g_child_map[i].z = z;
                if (addr && addr[0] != '\0') {
                    strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr));
                    g_child_map[i].addr[sizeof(g_child_map[i].addr) - 1] = '\0';
                }
                printf("[MAP-RECV] key=%d addr=%s -> (%d,%d,%d) (UPDATED)\n",
                       key,
                       g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                       x, y, z);
                return;
            }
        }
    }

    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (!g_child_map[i].valid) {
            g_child_map[i].valid = 1;
            g_child_map[i].key   = key;
            g_child_map[i].x     = x;
            g_child_map[i].y     = y;
            g_child_map[i].z     = z;
            if (addr && addr[0] != '\0') {
                strncpy(g_child_map[i].addr, addr, sizeof(g_child_map[i].addr));
                g_child_map[i].addr[sizeof(g_child_map[i].addr) - 1] = '\0';
            } else {
                g_child_map[i].addr[0] = '\0';
            }

            printf("[MAP-RECV] key=%d addr=%s -> (%d,%d,%d)\n",
                   key,
                   g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                   x, y, z);
            return;
        }
    }

    fprintf(stderr, "[WARN] Child map table full. Could not store key=%d\n", key);
}

// 子機側マップを親機と同じ形式でファイル保存 (変更なし)
static void child_dump_map_summary(const char *parent_addr) {
    printf("\n===== 受信したマッピング情報 (Child) =====\n");
    printf("Parent: %s\n", parent_addr);
    for (int i = 0; i < CHILD_MAX_MAP; i++) {
        if (g_child_map[i].valid) {
            printf("Key: %d, Addr: %s, Coords: (%d, %d, %d)\n",
                   g_child_map[i].key,
                   g_child_map[i].addr[0] ? g_child_map[i].addr : "UNKNOWN",
                   g_child_map[i].x,
                   g_child_map[i].y,
                   g_child_map[i].z);
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
                    g_child_map[i].key,
                    g_child_map[i].x,
                    g_child_map[i].y,
                    g_child_map[i].z);
        }
    }
    fprintf(fp, "\n# Raw Reports\n");
    fclose(fp);
}

// ==========================================================
// BLE スキャン (変更なし)
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
    scan_params_cp.filter   = 0x00;

    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS,
                 sizeof(scan_params_cp), &scan_params_cp);

    uint8_t enable = 0x01;
    uint8_t filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                 sizeof(cmd), cmd);

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    struct timeval tv = {
        .tv_sec  = timeout_ms / 1000,
        .tv_usec = (timeout_ms % 1000) * 1000
    };

    int ret_val = 0;

    while (ret_val == 0) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);

        struct timeval current_tv = tv;

        int r = select(sock + 1, &fds, NULL, NULL, &current_tv);
        if (r <= 0) break;

        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            break;
        }
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
                    if (name_len > (int)sizeof(name)-1)
                        name_len = (int)sizeof(name)-1;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                }
                pos += field_len + 1;
            }

            if (name[0] != '\0') {
                // フェーズ0: 自分宛 MT
                if (phase == 0 &&
                    strncmp(name, "MT:", 3) == 0 &&
                    strcmp(name + 3, target_addr) == 0) {
                    pthread_mutex_lock(&mag_state_mutex);
                    if (!electromagnet_requested) {
                        electromagnet_requested = 1;
                        printf("\n[SCAN] フェーズ1: 自分宛ての電磁石タイム要求(MT)を検知しました。\n");
                        ret_val = 1;
                    }
                    pthread_mutex_unlock(&mag_state_mutex);
                    break;
                }

                // フェーズ1: MK / MAP_END
                if (phase == 1) {
                    // --- MKフレームのパース ---
                    const char *p = NULL;
                    if (strncmp(name, "MK:", 3) == 0) {
                        p = name + 3;
                    } else if (strncmp(name, "MAPK:", 5) == 0) {
                        // 旧形式互換（念のため）
                        p = name + 5;
                    }

                    if (p) {
                        char *endp;
                        long key = strtol(p, &endp, 10);
                        if (endp && *endp == ':') {
                            p = endp + 1;

                            char full_addr[18];
                            full_addr[0] = '\0';

                            const char *next_colon  = strchr(p, ':');
                            const char *first_comma = strchr(p, ',');

                            if (next_colon && (!first_comma || next_colon < first_comma)) {
                                // 新形式: MK:<key>:<ADDR12>:x,y,z
                                char compact[13];
                                size_t addr_len = (size_t)(next_colon - p);
                                if (addr_len >= sizeof(compact))
                                    addr_len = sizeof(compact) - 1;
                                memcpy(compact, p, addr_len);
                                compact[addr_len] = '\0';

                                expand_compact_addr12(compact, full_addr, sizeof(full_addr));
                                p = next_colon + 1;
                            } else {
                                // 旧: MK:<key>:x,y,z / MAPK:<key>:x,y,z
                                snprintf(full_addr, sizeof(full_addr), "UNKNOWN");
                            }

                            long x = strtol(p, &endp, 10);
                            if (endp && *endp == ',') {
                                p = endp + 1;
                                long y = strtol(p, &endp, 10);
                                if (endp && *endp == ',') {
                                    p = endp + 1;
                                    long z = strtol(p, &endp, 10);
                                    child_store_map_frame((int)key,
                                                          full_addr,
                                                          (int)x, (int)y, (int)z);
                                }
                            }
                        }
                    }

                    // MAP_END
                    if (strncmp(name, "MAP_END:", 8) == 0) {
                        printf("\n[SCAN] フェーズ2: 親機からのマッピング送信完了通知を受信: %s\n", name);
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

// ==========================================================
// main (センサー検知ロジックを修正)
// ==========================================================
int main(int argc, char *argv[])
{
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

    if (wiringPiSetup() != 0) {
        fprintf(stderr, "wiringPiSetup failed.\n");
        return 1;
    }
    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) {
        fprintf(stderr, "wiringPiSPISetup failed.\n");
        return 1;
    }
    if (i2c_open() < 0 || bno055_init_ndof() < 0) {
        return 1;
    }

    int base = readInt("/sys/class/gpio/gpiochip512/base");
    if (base < 0) {
        fprintf(stderr, "ERROR: Failed to read GPIO base address.\n");
        close(i2c_fd);
        return 1;
    }
    g_linux_gpio = base + BCM_GPIO_PIN;
    printf("Resolved Child GPIO Pin: %d (BCM %d)\n", g_linux_gpio, BCM_GPIO_PIN);

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

    // ★ READY を 10秒間、複数回広告
    if (BLE_send_ready(my_addr) < 0) {
        fprintf(stderr, "Failed to send READY advertising.\n");
        close(i2c_fd);
        return 1;
    }

    child_map_init();

    printf("Begin continuous monitoring...\n");

    static char last_surface[16] = "";
    static int  same_count = 0;

    g_scan_phase = 0;
    printf("[PHASE] Starting in Phase 1 (MT monitoring).\n");
    
    // スキャンタイムアウト時間 (50ms)
    int scan_timeout_ms = 50;

    while (!map_end_received)
    {
        // 1. スキャンを実行 (MT検出のため)
        int scan_result = BLE_scan_for_targets(g_my_addr, scan_timeout_ms, g_scan_phase);

        if (scan_result == 2) {
            break;
        }

        // 2. MT/MAP_END 処理
        pthread_mutex_lock(&mag_state_mutex);
        int mag_req    = electromagnet_requested;
        int mag_active = electromagnet_active;
        if (mag_req) {
            electromagnet_requested = 0;
            electromagnet_active    = 1;
        }
        pthread_mutex_unlock(&mag_state_mutex);

        if (mag_req) {
            // --- 電磁石駆動フェーズ (MT命令受信) ---
            printf("\n===== 電磁石タイム START (自分自身を駆動) =====\n");

            if (gpio_init_and_on() != 0) {
                 pthread_mutex_lock(&mag_state_mutex);
                 electromagnet_active = 0;
                 pthread_mutex_unlock(&mag_state_mutex);
                 continue;
            }

            printf("[MAG] 電磁石 ON (20秒)\n");
            sleep(20);

            gpio_off_and_unexport();

            printf("[MAG] クールダウン (5秒) と MAP 配布受信への準備期間。\n");
            sleep(5);

            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 0;
            pthread_mutex_unlock(&mag_state_mutex);

            g_scan_phase = 1;
            printf("[PHASE] Transitioned to Phase 2 (MAP 配布受信モード).\n");

            printf("===== 電磁石タイム END (MAP配布受信モードへ移行) =====\n");
            continue;
        }

        if (g_scan_phase == 1 || mag_active) {
            usleep(100000); 
            continue;
        }

        // 3. 通常センサー監視 (MT待ち中のみ) - 毎回実行
        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) {
            window[ch] = read_adc_voltage(ch);
        }

        double maxDiff    = 0.0;
        int    maxCh      = -1;

        for (int ch = 1; ch < NUM_CH; ch++) {
            if (baseline[ch] < 1.0 || window[ch] < 1.0) continue;
            double diff = fabs(window[ch] - baseline[ch]);
            if (diff > maxDiff) {
                maxDiff = diff;
                maxCh   = ch;
            }
        }

        int is_candidate = 0;
        if (maxCh != -1 && maxDiff >= DIFF_THRESHOLD) {
            is_candidate = 1;
        }

        if (is_candidate)
        {
            float heading_north = read_bno055_heading(i2c_fd, heading_offset);
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
                    // 確定ログは残す
                    printf("\n[EVENT CONFIRMED] CH%d detected (Diff: %.3f V) -> Surface: **%s**\n",
                           maxCh, maxDiff, current_surface);
                    
                    // Surface-report を複数回広告
                    BLE_send_surface_data(my_addr, parent_addr, current_surface);
                    
                    same_count = 0;
                    last_surface[0] = '\0';
                }

            } else {
                fprintf(stderr,
                        "[WARNING] Failed to read BNO055 heading during detection.\n");
            }
        }
        usleep(100000); // センサーチェック後のわずかな待機
    }

    printf("\n\n===== マッピング終了通知(MAP_END)を受信しました。子機プログラムを終了します。 =====\n");

    child_dump_map_summary(parent_addr);

    if (i2c_fd != -1) close(i2c_fd);
    gpio_off_and_unexport();

    printf("CHILD PROGRAM END\n");
    return 0;
}
