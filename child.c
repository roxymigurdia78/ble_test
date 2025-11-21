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

// ==========================================================
// 🚨 BLE送信関数（アドバタイズを使って面情報を送る）
// ==========================================================
int BLE_send_surface_data(const char *my_addr,
                          const char *parent_addr,
                          const char *surface_name)
{
    printf("\n[COMM] Attempting BLE ADV send: Child=%s, Parent=%s, Surface=%s\n",
           my_addr, parent_addr, surface_name);

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
// MCP + BNO055 センサーロジック (CH1-CH6対応)
// ==========================================================

#define SPI_CH      0
#define SPI_SPEED   1000000
// 変更: CH1からCH6まで使用するため、配列サイズは7 (インデックス0は未使用、1～6使用)
#define NUM_CH      7 

// ★ しきい値まわり
#define DIFF_THRESHOLD        0.07   
#define SECOND_MARGIN         0.02   
#define STABLE_COUNT_REQUIRED 3      

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

    printf("Begin continuous monitoring...\n");

    static char last_surface[16] = "";
    static int  same_count = 0;
    
    while (1)
    {
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
        
        // ★ 「しきい値」と「2位との差」で候補かどうか判定
        int is_candidate = 0;
        if (maxCh != -1 &&
            maxDiff >= DIFF_THRESHOLD &&
            (maxDiff - secondDiff) >= SECOND_MARGIN) {
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
                    // 北(0度)基準: 315.0f〜45.0f
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

                // ★ 同じ面が連続しているかをチェック
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
    
    close(i2c_fd);
    return 0;
}
