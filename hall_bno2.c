/************************************************************
 * MCP3008 + BNO055 反応面検知（1.0V 未満の CH は完全無視）
 * BNO055 Heading に基づき、検知されたCHの向きを動的にマッピング (90度回転)
 * チャンネル番号をCH0->CH1, CH1->CH2, ..., CH5->CH6 に変更
 ************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <string.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>
#include <wiringPiI2C.h>

#define SPI_CH      0
#define SPI_SPEED  1000000
// チャンネル数を7に変更 (CH1からCH6を使用)
#define NUM_CH      7
#define THRESHOLD  0.020
#define BASELINE_SAMPLES 50

// BNO055 定義
#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_OPR_MODE_ADDR   0x3D
#define OPERATION_MODE_NDOF    0x0C
#define EULER_UNIT             16.0f

// ファイル名
const char *OFFSET_FILE  = "bno055_heading_offset.txt";

static int i2c_fd = -1;

// ================= I2C/BNO055 基本関数 (前回修正から維持) =================

static void msleep(int ms) {
    usleep(ms * 1000);
}

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
    if (i2c_write8(BNO055_OPR_MODE_ADDR, mode) < 0)
        return -1;
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

// ---------------------------------------------------------
// BNO055 ヘディング（北を0°）取得
// ---------------------------------------------------------
float read_bno055_heading(int fd, float heading_offset)
{
    uint8_t buf[2];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 2) < 0) {
        return NAN;
    }

    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
    float heading_raw = raw_heading / EULER_UNIT;

    float heading_north = heading_raw - heading_offset;
    while (heading_north < 0.0f)   heading_north += 360.0f;
    while (heading_north >= 360.0f) heading_north -= 360.0f;

    return heading_north;
}

// ---------------------------------------------------------
// heading offset 読み込み
// ---------------------------------------------------------
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

// ================= MCP3008 基本関数 (前回修正から維持) =================

//---------------------------------------------------------
// MCP3008（SPI） 電圧読み取り
//---------------------------------------------------------
double read_adc_voltage(int ch)
{
    unsigned char data[3];
    data[0] = 1;
    data[1] = (8 + ch) << 4;
    data[2] = 0;

    wiringPiSPIDataRW(SPI_CH, data, 3);

    int value = ((data[1] & 3) << 8) | data[2];
    double voltage = (double)value * 3.3 / 1023.0;

    return voltage;
}

//---------------------------------------------------------
// メイン
//---------------------------------------------------------
int main()
{
    wiringPiSetup();
    wiringPiSPISetup(SPI_CH, SPI_SPEED);

    if (i2c_open() < 0) return 1;
    if (bno055_init_ndof() < 0) return 1;

    float heading_offset = 0.0f;
    if (load_heading_offset(OFFSET_FILE, &heading_offset) == 0) {
        printf("Heading offset loaded: %.2f deg (using %s)\n", heading_offset, OFFSET_FILE);
    } else {
        printf("ERROR: Heading offset file '%s' not found or failed to load.\n", OFFSET_FILE);
        printf("Please run the 9ziku calibration procedure first.\n");
        return 1;
    }

    // CH1-CH6 を監視対象に変更
    printf("Starting Hall sensors (CH1-CH6) monitoring...\n");
    printf("Baseline sampling...\n");

    // 配列サイズを NUM_CH (7) に変更。インデックス 0 (CH0) は未使用のまま
    double baseline[NUM_CH] = {0};

    // ---------------- baseline 計測 ----------------
    // CH1からCH6をループ
    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) { // ループを CH1 から開始 (ch=1)
            baseline[ch] += read_adc_voltage(ch);
        }
        usleep(100000);
    }

    // CH1からCH6をループ
    for (int ch = 1; ch < NUM_CH; ch++)
        baseline[ch] /= BASELINE_SAMPLES;

    printf("Baseline established:\n");
    // CH1からCH6を出力
    for (int ch = 1; ch < NUM_CH; ch++)
        printf(" CH%d = %.3f V\n", ch, baseline[ch]);

    printf("Begin monitoring...\n\n");


    // ---------------- メインループ ----------------
    while (1)
    {
        // 配列サイズを NUM_CH (7) に変更
        double window[NUM_CH];

        // CH1からCH6をループ
        for (int ch = 1; ch < NUM_CH; ch++)
            window[ch] = read_adc_voltage(ch);

        double maxDiff = 0.0;
        int maxCh = -1;

        // CH1からCH6をループ
        for (int ch = 1; ch < NUM_CH; ch++)
        {
            // ---------------- 1.0V 以下は完全無視 ----------------
            if (baseline[ch] < 1.0 || window[ch] < 1.0)
                continue;

            double diff = fabs(window[ch] - baseline[ch]);

            if (diff > THRESHOLD && diff > maxDiff) {
                maxDiff = diff;
                maxCh = ch;
            }
        }

        if (maxCh != -1)
        {
            float heading_north = read_bno055_heading(i2c_fd, heading_offset);
            
            if (isnan(heading_north)) {
                fprintf(stderr, "Failed to read BNO055 heading. Skipping output.\n");
            } else {
                printf("CH%d confirmed: diff=%.3f V  Heading=%.1f°\n",
                        maxCh, maxDiff, heading_north);

                // ================= 動的な面割り当てロジック (90°回転ベース) =================
                const char* detected_surface = NULL;

                // CH0 -> CH1 (TOP) に変更
                if (maxCh == 1) {
                    // CH1: TOP
                    detected_surface = "TOP";
                // CH5 -> CH6 (BOTTOM) に変更
                } else if (maxCh == 6) {
                    // CH6: BOTTOM
                    detected_surface = "BOTTOM";
                } else {
                    // 側面 (CH2, CH3, CH4, CH5) の処理
                    
                    // 1. Headingに基づいて回転セクションを決定
                    int rotation_index = 0; // 0: 0度, 1: 90度, 2: 180度, 3: 270度
                    
                    if (heading_north >= 45.0f && heading_north < 135.0f) {
                        rotation_index = 1; // 90度回転 (東)
                    } else if (heading_north >= 135.0f && heading_north < 225.0f) {
                        rotation_index = 2; // 180度回転 (南)
                    } else if (heading_north >= 225.0f && heading_north < 315.0f) {
                        rotation_index = 3; // 270度回転 (西)
                    } else { // 315.0f〜360.0f または 0.0f〜45.0f
                        rotation_index = 0; // 0度回転 (北)
                    }
                    
                    // 2. CHの物理的な初期位置から、回転後の真の向きを決定
                    
                    // CHの静的な物理位置 (0度回転時)
                    // (CH1:TOP, CH2:FRONT, CH3:LEFT, CH4:BACK, CH5:RIGHT, CH6:BOTTOM)
                    
                    // 4つの静的な側面CHのインデックス: {CH2, CH3, CH4, CH5}
                    // CH2=FRONT(0), CH3=LEFT(1), CH4=BACK(2), CH5=RIGHT(3)
                    
                    // 検知されたCHが側面チャンネルのどれかを見つける
                    int initial_index = -1;
                    if (maxCh == 2) initial_index = 0; // FRONT (旧CH1)
                    else if (maxCh == 3) initial_index = 1; // LEFT (旧CH2)
                    else if (maxCh == 4) initial_index = 2; // BACK (旧CH3)
                    else if (maxCh == 5) initial_index = 3; // RIGHT (旧CH4)
                    
                    if (initial_index != -1) {
                        // 回転後のインデックスを計算
                        // マッピング配列のインデックス: 0:FRONT, 1:LEFT, 2:BACK, 3:RIGHT
                        int mapped_index = (initial_index + rotation_index) % 4;
                        
                        // 回転後のインデックスから、真の向きの文字列を取得
                        const char* final_names[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
                        detected_surface = final_names[mapped_index];
                    } else {
                        // ありえないケースだが念のため
                         detected_surface = "ERROR_SIDE";
                    }
                }
                
                printf("Detected Surface: %s\n\n", detected_surface);
                
                // 検知後の待機時間 (2秒) を維持
                sleep(2); 
            }
        }

        usleep(100000);
    }
    
    close(i2c_fd);
    return 0;
}
