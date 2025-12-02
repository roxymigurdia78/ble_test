// child.c
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
#include <sys/stat.h> // coil.c機能のために追加

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

// 自分のアドレス（スキャンスレッドから参照するためグローバルにコピー）
static char g_my_addr[18] = "(unknown)";

// 電磁石タイム要求フラグ (親機からの自分宛 MT を検知)
static volatile int electromagnet_requested = 0;
// 電磁石が駆動中フラグ (隣接キューブのセンサーはこの間のみ反応)
static volatile int electromagnet_active = 0; 
static pthread_mutex_t mag_state_mutex = PTHREAD_MUTEX_INITIALIZER;

// ★★★ coil.c から統合したGPIO制御定義 ★★★
const int BCM_GPIO_PIN = 20; // BCM 20 を使用 (物理ピン38番)
static int g_linux_gpio = -1;

// ==========================================================
// ★★★ GPIO 制御ヘルパー関数 (coil.cから統合) ★★★
// ==========================================================

// 通常の書き込み（エラーは表示）
static int writeFile(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return -1; }
    if (fprintf(f, "%s", value) < 0) { perror("fprintf"); fclose(f); return -1; }
    fclose(f);
    return 0;
}

// エラーを表示しない書き込み（unexport用）
static int writeFileSilent(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) { return -1; }
    if (fprintf(f, "%s", value) < 0) { fclose(f); return -1; }
    fclose(f);
    return 0;
}

// ファイルから int を読み込む (gpiochip の base を読む用)
static int readInt(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
    int v;
    if (fscanf(f, "%d", &v) != 1) { fprintf(stderr, "failed to read int from %s\n", path); fclose(f); return -1; }
    fclose(f);
    return v;
}

// ファイルが存在するかどうか
static int fileExists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0);
}

// ★ 電磁石 ON 処理: export → out 設定 → ON
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

    // 1. 確実に unexport を試みる (クリーンアップ)
    writeFileSilent("/sys/class/gpio/unexport", num_str);
    usleep(100000); 

    // 2. GPIO を export
    if (writeFile("/sys/class/gpio/export", num_str) < 0) {
         fprintf(stderr, "ERROR: Failed to export GPIO %d. (Need sudo?)\n", g_linux_gpio);
         return -1;
    }
    usleep(100000); 

    // 3. 方向を out に設定
    if (writeFile(path_dir, "out") < 0) {
        fprintf(stderr, "ERROR: Failed to set direction to 'out'.\n");
        return -1;
    }

    // 4. ON にする
    if (writeFile(path_val, "1") < 0) {
        fprintf(stderr, "ERROR: Failed to write '1' to value.\n");
        return -1;
    }
    printf("[GPIO] 子機電磁石 ON (GPIO %d)\n", g_linux_gpio);
    return 0;
}

// ★ 電磁石 OFF 処理: OFF → unexport
static void gpio_off_and_unexport(void) {
    char num_str[16];
    char path_val[128];
    
    if (g_linux_gpio == -1) return;

    snprintf(num_str, sizeof(num_str), "%d", g_linux_gpio);
    snprintf(path_val, sizeof(path_val), "/sys/class/gpio/gpio%d/value", g_linux_gpio);

    if (fileExists(path_val)) {
        // 1. OFF にする (エラーは無視)
        writeFileSilent(path_val, "0");
        
        // 2. unexport (エラーは無視)
        writeFileSilent("/sys/class/gpio/unexport", num_str);
        printf("[GPIO] 子機電磁石 OFF & Unexport (GPIO %d)\n", g_linux_gpio);
    }
}
// ==========================================================


// ==========================================================
// 🚨 BLE送信関数（アドバタイズを使って面情報を送る）
// ==========================================================
int BLE_send_surface_data(const char *my_addr,
                          const char *parent_addr,
                          const char *surface_name)
{
    (void)parent_addr; 

    printf("\n[COMM] Attempting BLE ADV send: Child=%s, Surface=%s\n",
           my_addr, surface_name);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[BLE] hci_get_route");
        return -1;
    }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("[BLE] hci_open_dev");
        return -1;
    }

    // --- Advertising Parameters 設定 ---
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    // 500ms程度のインターバル
    uint16_t interval = (uint16_t)(500 * 1.6); // 0.625ms単位
    adv_params_cp.min_interval     = htobs(interval);
    adv_params_cp.max_interval     = htobs(interval);
    adv_params_cp.advtype          = 0x00;    // Connectable undirected
    adv_params_cp.own_bdaddr_type  = 0x00;    // Public
    adv_params_cp.chan_map         = 0x07;
    adv_params_cp.filter           = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[BLE] Failed to set advertising parameters");
        close(sock);
        return -1;
    }

    // --- Advertising Data 構築 ---
    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));

    int len = 0;
    // Flags
    adv_data[len++] = 2;      // 長さ
    adv_data[len++] = 0x01;   // Flags
    adv_data[len++] = 0x06;   // LE General Discoverable + BR/EDR not supported

    // Local Name: "CubeNode|SURFACE:XXXX"
    char name_field[31];
    snprintf(name_field, sizeof(name_field),
             "CubeNode|SURFACE:%s", surface_name);

    int name_len = (int)strlen(name_field);
    if (name_len > 29) name_len = 29; // 31 - 2(Length,type) =29

    adv_data[len++] = (uint8_t)(name_len + 1); // 長さ(タイプ+データ)
    adv_data[len++] = 0x09;                   // Complete Local Name
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

    // --- Advertising Enable ---
    uint8_t enable = 0x01;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("[BLE] Failed to enable advertising");
        close(sock);
        return -1;
    }

    printf("[COMM] BLE advertising started with name='%s'\n", name_field);

    // 少なくとも1秒程度は広告を飛ばす（親が拾う時間）
    sleep(1);

    // --- Advertising Disable ---
    enable = 0x00;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("[BLE] Failed to disable advertising");
        // ここは致命的ではないので続行
    }

    printf("[COMM] BLE advertising stopped.\n");

    close(sock);
    return 0;
}

// ==========================================================
// 電磁石タイム終了を知らせる ADV ("ME") を20秒間送信 (ON時間に合わせる)
// Local Name: "ME"
// ==========================================================
int BLE_send_mag_end(const char *my_addr) {
    printf("[COMM] Sending MAG END advertise from %s\n", my_addr);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[BLE] hci_get_route (ME)");
        return -1;
    }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("[BLE] hci_open_dev (ME)");
        return -1;
    }

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(500 * 1.6);
    adv_params_cp.min_interval     = htobs(interval);
    adv_params_cp.max_interval     = htobs(interval);
    adv_params_cp.advtype          = 0x00;
    adv_params_cp.own_bdaddr_type  = 0x00;
    adv_params_cp.chan_map         = 0x07;
    adv_params_cp.filter           = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[BLE] Failed to set advertising parameters (ME)");
        close(sock);
        return -1;
    }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;

    // Flags
    adv_data[len++] = 2;
    adv_data[len++] = 0x01;
    adv_data[len++] = 0x06;

    const char *name_field = "ME";
    int name_len = (int)strlen(name_field);
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09; // Complete Local Name
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
        perror("[BLE] Failed to set advertising data (ME)");
        close(sock);
        return -1;
    }

    uint8_t enable = 0x01;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("[BLE] Failed to enable advertising (ME)");
        close(sock);
        return -1;
    }

    printf("[COMM] MAG END advertising start (20 seconds)\n");
    sleep(20); // ★ 20秒に延長 (親機のON時間およびタイムアウトに合わせる)

    enable = 0x00;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("[BLE] Failed to disable advertising (ME)");
    }
    printf("[COMM] MAG END advertising stop\n");

    close(sock);
    return 0;
}

// ==========================================================
// 修正: センサー準備完了を知らせる ADV ("READY") を継続的に送信
// Local Name: "READY"
// ==========================================================
int BLE_send_ready(const char *my_addr) {
    printf("[COMM] Sending READY advertise from %s (Persistent)\n", my_addr);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[BLE] hci_get_route (READY)");
        return -1;
    }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("[BLE] hci_open_dev (READY)");
        return -1;
    }

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(500 * 1.6);
    adv_params_cp.min_interval     = htobs(interval);
    adv_params_cp.max_interval     = htobs(interval);
    adv_params_cp.advtype          = 0x00;
    adv_params_cp.own_bdaddr_type  = 0x00;
    adv_params_cp.chan_map         = 0x07;
    adv_params_cp.filter           = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[BLE] Failed to set advertising parameters (READY)");
        close(sock);
        return -1;
    }

    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));
    int len = 0;

    // Flags
    adv_data[len++] = 2;
    adv_data[len++] = 0x01;
    adv_data[len++] = 0x06;

    const char *name_field = "READY";
    int name_len = (int)strlen(name_field);
    adv_data[len++] = (uint8_t)(name_len + 1);
    adv_data[len++] = 0x09; // Complete Local Name
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
        perror("[BLE] Failed to set advertising data (READY)");
        close(sock);
        return -1;
    }

    uint8_t enable = 0x01;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("[BLE] Failed to enable advertising (READY)");
        close(sock);
        return -1;
    }

    printf("[COMM] READY advertising started. (Socket: %d)\n", sock);
    
    // 継続広告のため、sleep と disable を削除。ソケットを開いたまま返す。
    return sock; // ソケットディスクリプタを返す
}

// ==========================================================
// MCP + BNO055 センサーロジック (CH1-CH6対応)
// ==========================================================

#define SPI_CH      0
#define SPI_SPEED   1000000
// 変更: CH1からCH6まで使用するため、配列サイズは7 (インデックス0は未使用、1～6使用)
#define NUM_CH      7 

// ★ しきい値まわり（hall_bno2 に寄せる）
#define DIFF_THRESHOLD        0.020   // 0.02V 以上を検出
#define SECOND_MARGIN         0.0     // 一旦未使用
#define STABLE_COUNT_REQUIRED 1       // 1回で確定させる

#define BASELINE_SAMPLES 50

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

// --- センサー基本関数 ---
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
        fprintf(stderr, "Unexpected BNO055 ID: 0x%02X (expected 0x%02X)\n",
                id, BNO055_ID_EXPECTED);
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
    (void)fd; // i2c_fdをグローバルで使っているので未使用扱い
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

// MCP3008/3208 風 10bit 読み出し
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

// ---------------------------------------------------------
// 子機側: 親の "MT:<自分のアドレス>" アドバタイズを監視するスレッド
// ---------------------------------------------------------
void *mag_scan_thread(void *arg) {
    (void)arg;

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[SCAN] hci_get_route");
        return NULL;
    }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("[SCAN] hci_open_dev");
        return NULL;
    }

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        perror("[SCAN] HCI filter");
        close(sock);
        return NULL;
    }

    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01;               // Active scan
    scan_params_cp.interval = htobs(0x0010);
    scan_params_cp.window   = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00;    // Public
    scan_params_cp.filter   = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS,
                     sizeof(scan_params_cp), &scan_params_cp) < 0) {
        perror("[SCAN] set scan params");
        close(sock);
        return NULL;
    }

    uint8_t enable = 0x01;
    uint8_t filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("[SCAN] enable scan");
        close(sock);
        return NULL;
    }

    unsigned char buf[HCI_MAX_EVENT_SIZE];

    printf("[SCAN] 子機スキャンスレッド開始 (MT: 自分宛ての電磁石タイム要求を監視)\n");

    while (1) {
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            perror("[SCAN] read");
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
                if (field_len == 0) break;
                if (pos + field_len >= info->length) break;

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
                // Local Name が "MT:<addr>" 形式かチェック
                if (strncmp(name, "MT:", 3) == 0) {
                    const char *addr_part = name + 3;
                    pthread_mutex_lock(&mag_state_mutex);
                    if (strcmp(addr_part, g_my_addr) == 0 && !electromagnet_requested) {
                        electromagnet_requested = 1;
                        printf("\n[MAG] 自分宛ての電磁石タイム要求(MT)を検知しました。\n");
                        
                        // ★ 要求を検知したらスキャンを停止し、スレッドを終了する
                        enable = 0x00;
                        cmd[0] = enable;
                        cmd[1] = 0x00;
                        hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                                     sizeof(cmd), cmd);
                        close(sock);
                        pthread_mutex_unlock(&mag_state_mutex);
                        return NULL; // スレッド終了
                    }
                    pthread_mutex_unlock(&mag_state_mutex);
                }
            }

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    // 通常は到達しない
    enable = 0x00;
    cmd[0] = enable;
    cmd[1] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                 sizeof(cmd), cmd);

    close(sock);
    return NULL;
}

// ---------------------------------------------------------
// 子機のメイン関数
// ---------------------------------------------------------
int main(int argc, char *argv[])
{
    if (argc < 3) {
        fprintf(stderr,
                "Usage: ./child <my_full_address> <parent_full_address>\n");
        return 1;
    }
    const char *my_addr     = CHILD_ADDR_ARG;
    const char *parent_addr = PARENT_ADDR_ARG;
    
    // グローバルコピー（スキャンスレッドが使う）
    strncpy(g_my_addr, my_addr, sizeof(g_my_addr));
    g_my_addr[sizeof(g_my_addr) - 1] = '\0';
    
    printf("\n==================================\n");
    printf("🌱 CHILD PROGRAM STARTING\n");
    printf("==================================\n");
    printf("My full address (Child): **%s**\n", my_addr);
    printf("Confirmed Parent Address: **%s**\n", parent_addr);
    
    // I/O初期化
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
    
    // ★★★ GPIO Base アドレス読み込み ★★★
    // NOTE: gpiochip512 のパスは環境によって異なる可能性があります。
    int base = readInt("/sys/class/gpio/gpiochip512/base"); 
    if (base < 0) {
        fprintf(stderr, "ERROR: Failed to read GPIO base address. Check /sys/class/gpio/gpiochip*/base. Aborting.\n");
        close(i2c_fd);
        return 1;
    }
    g_linux_gpio = base + BCM_GPIO_PIN;
    printf("Resolved Child GPIO Pin: %d (BCM %d)\n", g_linux_gpio, BCM_GPIO_PIN);
    // --------------------------------

    float heading_offset = 0.0f;
    if (load_heading_offset(OFFSET_FILE, &heading_offset) != 0) {
        printf("ERROR: Heading offset file not loaded. Aborting.\n");
        close(i2c_fd);
        return 1;
    }

    // ベースライン計測 (CH1-CH6)
    double baseline[NUM_CH] = {0}; 
    printf("Baseline sampling...\n");
    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) { // CH1からCH6までをループ
            baseline[ch] += read_adc_voltage(ch);
        }
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) {
        baseline[ch] /= BASELINE_SAMPLES;
    }
    printf("Baseline established.\n");

    // ★ 準備完了(READY)を親機に通知 (常時広告ソケットを取得)
    int ready_sock = BLE_send_ready(my_addr);
    if (ready_sock < 0) {
        fprintf(stderr, "Failed to start persistent READY advertising.\n");
        close(i2c_fd);
        return 1;
    }

    // 親の MT アドバタイズを監視するスレッド開始
    pthread_t scan_th;
    if (pthread_create(&scan_th, NULL, mag_scan_thread, NULL) != 0) {
        perror("mag_scan_thread の生成に失敗しました");
        // 生成失敗しても、電磁石タイム機能なしで動作を継続
    }

    printf("Begin continuous monitoring...\n");

    static char last_surface[16] = "";
    static int  same_count = 0;
    
    while (1)
    {
        // --- 電磁石タイム要求が来ていないかチェック ---
        pthread_mutex_lock(&mag_state_mutex);
        int mag_req = electromagnet_requested;
        int mag_active = electromagnet_active;
        if (mag_req) {
            electromagnet_requested = 0; // 要求を消費
            electromagnet_active = 1;    // 電磁石を駆動
        }
        pthread_mutex_unlock(&mag_state_mutex);

        if (mag_req) {
            // **電磁石タイム本体 (自分自身を駆動)**
            printf("\n===== 電磁石タイム START (自分自身を駆動) =====\n");
            
            // ★ READY広告の一時停止
            uint8_t enable = 0x00;
            if (hci_send_cmd(ready_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &enable) < 0) {
                perror("[COMM] Failed to disable READY advertising before MAG START");
            }
            printf("[COMM] READY advertising temporarily stopped.\n");

            // ★ 1. ON コマンドを実行を関数呼び出しに置き換え
            if (gpio_init_and_on() != 0) {
                 // ON失敗時のリカバリ
                 pthread_mutex_lock(&mag_state_mutex);
                 electromagnet_active = 0; 
                 pthread_mutex_unlock(&mag_state_mutex);

                 // READY再開
                 enable = 0x01;
                 hci_send_cmd(ready_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);
                 continue; 
            }
            
            // 2. 終了を親に通知 (20秒間 "ME" をアドバタイズ)
            BLE_send_mag_end(my_addr);

            // ★ 3. OFF コマンドを実行を関数呼び出しに置き換え
            gpio_off_and_unexport();
            
            pthread_mutex_lock(&mag_state_mutex);
            electromagnet_active = 0; // 電磁石駆動終了
            pthread_mutex_unlock(&mag_state_mutex);
            
            // ★ READY広告の再開
            enable = 0x01;
            if (hci_send_cmd(ready_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &enable) < 0) {
                perror("[COMM] Failed to re-enable READY advertising");
            }
            printf("[COMM] READY advertising re-enabled.\n");


            printf("===== 電磁石タイム END (ME送信完了) =====\n");
            // 連続送信防止用の sleep(2)
            sleep(2); 
            continue;
        }

        // ここから先は従来のセンサー監視ロジック（電磁石駆動中はスキップ）
        if (mag_active) {
            // 親からの ME 受信を待っている間はセンサーをポーリングしない
            usleep(100000);
            continue;
        }

        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) { // CH1からCH6までをループ
            window[ch] = read_adc_voltage(ch);
        }

        double maxDiff    = 0.0;
        double secondDiff = 0.0;
        int    maxCh      = -1;
        int    secondCh   = -1;

        for (int ch = 1; ch < NUM_CH; ch++) { // CH1からCH6までをループ
            // 1.0V未満は無視
            if (baseline[ch] < 1.0 || window[ch] < 1.0) continue;

            double diff = fabs(window[ch] - baseline[ch]);

            if (diff > maxDiff) {
                secondDiff = maxDiff;
                secondCh   = maxCh;
                maxDiff    = diff;
                maxCh      = ch;
            } else if (diff > secondDiff) {
                secondDiff = diff;
                secondCh   = ch;
            }
        }
        
        // ★ 「しきい値」だけで候補かどうか判定（hall_bno2 っぽく緩く）
        int is_candidate = 0;
        if (maxCh != -1 && maxDiff >= DIFF_THRESHOLD) {
            is_candidate = 1;
        }

        if (is_candidate)
        {
            float heading_north = read_bno055_heading(i2c_fd, heading_offset);
            const char* detected_surface = "UNKNOWN";

            if (!isnan(heading_north)) {
                // TOP / BOTTOM 判定
                if (maxCh == 1)      detected_surface = "TOP";   // CH1 = 上
                else if (maxCh == 6) detected_surface = "BOTTOM";// CH6 = 下
                else {
                    // 側面 (CH2, CH3, CH4, CH5) の処理
                    
                    // 1. Headingに基づいて回転セクションを決定
                    int rotation_index = 0; 
                    
                    if (heading_north >= 45.0f  && heading_north < 135.0f) rotation_index = 1; // 東向き (90度回転)
                    else if (heading_north >= 135.0f && heading_north < 225.0f) rotation_index = 2; // 南向き (180度回転)
                    else if (heading_north >= 225.0f && heading_north < 315.0f) rotation_index = 3; // 西向き (270度回転)
                    else rotation_index = 0; // 北向き (0度回転)
                    
                    // 2. CHの物理的な初期位置から、回転後の真の向きを決定
                    // CH2=FRONT(0), CH3=LEFT(1), CH4=BACK(2), CH5=RIGHT(3)
                    int initial_index = -1;
                    if (maxCh == 2)      initial_index = 0; // 物理FRONT
                    else if (maxCh == 3) initial_index = 1; // 物理LEFT
                    else if (maxCh == 4) initial_index = 2; // 物理BACK
                    else if (maxCh == 5) initial_index = 3; // 物理RIGHT
                    
                    if (initial_index != -1) {
                        // 回転後のインデックスを計算: (初期位置 + 回転量) % 4
                        int mapped_index = (initial_index + rotation_index) % 4;
                        const char* final_names[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
                        detected_surface = final_names[mapped_index];
                    }
                }

                // ★ 同じ面が連続しているかをチェック（ただし STABLE_COUNT_REQUIRED=1）
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

                printf("\n[EVENT CANDIDATE] CH%d Diff: %.3f V -> Surface: %s (same_count=%d)\n",
                       maxCh, maxDiff, current_surface, same_count);

                // ★ 連続 STABLE_COUNT_REQUIRED 回同じ面が出たら「確定」
                if (same_count >= STABLE_COUNT_REQUIRED) {
                    printf("\n[EVENT CONFIRMED] CH%d detected (Diff: %.3f V) -> Surface: **%s**\n",
                           maxCh, maxDiff, current_surface);

                    BLE_send_surface_data(my_addr, parent_addr, current_surface);

                    // 連続検出をリセット
                    same_count = 0;
                    last_surface[0] = '\0';

                    sleep(2); // 連続送信防止
                }

            } else {
                fprintf(stderr,
                        "[WARNING] Failed to read BNO055 heading during detection.\n");
            }
        }

        usleep(100000);
    }
    
    close(ready_sock); // ★ プログラム終了時にソケットを閉じる
    close(i2c_fd);
    
    // ★ プログラム終了時に念のためOFF/アンエクスポートを保証
    gpio_off_and_unexport(); 
    
    return 0;
}
