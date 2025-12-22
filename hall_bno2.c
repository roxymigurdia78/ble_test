// hall_bno2.c
// MCP3008 + BNO055（共有キャリブ bno055_calib.bin を毎回“復元”してから使う版）
//
// 前提：事前に 9ziku.c を一度実行して
//   - bno055_calib.bin
//   - bno055_heading_offset.txt
// を作っておく（以後は再キャリブ不要。hall_bno2 は毎回 “復元” するだけ）
//
// build:
//   gcc hall_bno2.c -o hall_bno2 -lwiringPi -lm
// run:
//   sudo ./hall_bno2
//
// リセットしたいとき（再キャリブしたいとき）:
//   rm bno055_calib.bin bno055_heading_offset.txt
//   sudo ./9ziku
//   sudo ./hall_bno2

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <math.h>
#include <string.h>
#include <errno.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

// ================== MCP3008 ==================
#define SPI_CH      0
#define SPI_SPEED   1000000
#define NUM_CH      7          // CH0..CH6（ただしCH1..CH6を使用）
#define THRESHOLD   0.010
#define BASELINE_SAMPLES 50

// ================== BNO055 ==================
#define I2C_DEV_PATH          "/dev/i2c-1"
#define BNO055_ADDRESS        0x28
#define BNO055_ID_EXPECTED    0xA0

#define BNO055_CHIP_ID_ADDR       0x00
#define BNO055_EUL_HEADING_LSB    0x1A
#define BNO055_CALIB_STAT_ADDR    0x35
#define BNO055_UNIT_SEL_ADDR      0x3B
#define BNO055_PWR_MODE_ADDR      0x3E
#define BNO055_OPR_MODE_ADDR      0x3D
#define BNO055_SYS_TRIGGER        0x3F

#define POWER_MODE_NORMAL         0x00

#define OPERATION_MODE_CONFIG     0x00
#define OPERATION_MODE_NDOF       0x0C

// キャリブレーションデータ
#define BNO055_CALIB_START        0x55
#define BNO055_CALIB_DATA_LEN     22

// 1 LSB = 1/16 deg
#define EULER_UNIT                16.0f

// ファイル
static const char *CALIB_FILE  = "bno055_calib.bin";
static const char *OFFSET_FILE = "bno055_heading_offset.txt";

static int i2c_fd = -1;

static void msleep(int ms) { usleep(ms * 1000); }

// ================= I2C 基本 =================
static int i2c_open_dev(void) {
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

static int i2c_read8(uint8_t reg, uint8_t *value) {
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("i2c_read8:write(reg)");
        return -1;
    }
    if (read(i2c_fd, value, 1) != 1) {
        perror("i2c_read8:read");
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

// ================= BNO055 初期化/復元 =================
static int bno055_set_mode(uint8_t mode) {
    if (i2c_write8(BNO055_OPR_MODE_ADDR, mode) < 0) return -1;
    msleep(30);
    return 0;
}

static int bno055_init_basic(void) {
    uint8_t id = 0;

    if (i2c_read8(BNO055_CHIP_ID_ADDR, &id) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        return -1;
    }
    if (id != BNO055_ID_EXPECTED) {
        fprintf(stderr, "Unexpected BNO055 ID: 0x%02X (expected 0x%02X)\n", id, BNO055_ID_EXPECTED);
        return -1;
    }
    printf("BNO055 detected! ID=0x%02X\n", id);

    // CONFIGへ
    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0) return -1;

    // Power mode
    if (i2c_write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0) return -1;
    msleep(10);

    // Unit: default (deg)
    if (i2c_write8(BNO055_UNIT_SEL_ADDR, 0x00) < 0) return -1;
    msleep(10);

    // Sys trigger: normal
    if (i2c_write8(BNO055_SYS_TRIGGER, 0x00) < 0) return -1;
    msleep(10);

    return 0;
}

static int bno055_restore_calibration_file(const char *path) {
    uint8_t calib[BNO055_CALIB_DATA_LEN];

    FILE *fp = fopen(path, "rb");
    if (!fp) return -1; // ファイルなし
    size_t n = fread(calib, 1, BNO055_CALIB_DATA_LEN, fp);
    fclose(fp);

    if (n != BNO055_CALIB_DATA_LEN) {
        fprintf(stderr, "Calibration file size mismatch: %s\n", path);
        return -1;
    }

    // CONFIGモードで書き込み
    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0) return -1;

    for (int i = 0; i < BNO055_CALIB_DATA_LEN; i++) {
        if (i2c_write8((uint8_t)(BNO055_CALIB_START + i), calib[i]) < 0) {
            fprintf(stderr, "Failed to write calibration register\n");
            return -1;
        }
    }

    printf("Calibration restored from %s\n", path);
    return 0;
}

static void bno055_print_calib_stat(void) {
    uint8_t stat = 0;
    if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) < 0) {
        printf("CALIB_STAT read failed\n");
        return;
    }
    int sys = (stat >> 6) & 0x03;
    int gyr = (stat >> 4) & 0x03;
    int acc = (stat >> 2) & 0x03;
    int mag = (stat >> 0) & 0x03;
    printf("CALIB SYS:%d ACC:%d GYR:%d MAG:%d\n", sys, acc, gyr, mag);
}

// “安定待ち”を短時間だけ行う（再キャリブはしない）
static void bno055_brief_settle_wait(int max_ms) {
    int waited = 0;
    while (waited < max_ms) {
        uint8_t stat = 0;
        if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) == 0) {
            int mag = (stat >> 0) & 0x03;
            // MAGが3なら十分（共有キャリブを入れている前提）
            if (mag == 3) return;
        }
        msleep(50);
        waited += 50;
    }
}

// ================= Heading offset =================
static int load_heading_offset(const char *path, float *offset) {
    FILE *fp = fopen(path, "r");
    if (!fp) return -1;
    if (fscanf(fp, "%f", offset) != 1) {
        fclose(fp);
        return -1;
    }
    fclose(fp);
    return 0;
}

// ================= Euler/Heading =================
static float bno055_read_heading_raw(void) {
    uint8_t buf[2];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 2) < 0) return NAN;
    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
    return (float)raw_heading / EULER_UNIT;
}

static float bno055_heading_north(float heading_offset) {
    float raw = bno055_read_heading_raw();
    if (isnan(raw)) return NAN;

    float north = raw - heading_offset;
    while (north < 0.0f) north += 360.0f;
    while (north >= 360.0f) north -= 360.0f;
    return north;
}

// ================= MCP3008 =================
static double read_adc_voltage(int ch) {
    unsigned char data[3];
    data[0] = 1;
    data[1] = (8 + ch) << 4;
    data[2] = 0;

    if (wiringPiSPIDataRW(SPI_CH, data, 3) == -1) {
        perror("wiringPiSPIDataRW");
        return NAN;
    }

    int value = ((data[1] & 3) << 8) | data[2];
    return (double)value * 3.3 / 1023.0;
}

// ================= main =================
int main(void) {
    // --- wiringPi / SPI ---
    if (wiringPiSetup() != 0) {
        fprintf(stderr, "wiringPiSetup failed\n");
        return 1;
    }
    if (wiringPiSPISetup(SPI_CH, SPI_SPEED) < 0) {
        fprintf(stderr, "wiringPiSPISetup failed\n");
        return 1;
    }

    // --- I2C / BNO055 ---
    if (i2c_open_dev() < 0) return 1;

    if (bno055_init_basic() < 0) {
        fprintf(stderr, "BNO055 init failed\n");
        close(i2c_fd);
        return 1;
    }

    // 共有キャリブ復元（毎回やる。再キャリブはしない）
    if (bno055_restore_calibration_file(CALIB_FILE) != 0) {
        printf("ERROR: Calibration file '%s' not found or invalid.\n", CALIB_FILE);
        printf("Please run 9ziku.c once to generate %s and %s\n", CALIB_FILE, OFFSET_FILE);
        close(i2c_fd);
        return 1;
    }

    // NDOFへ
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "Failed to set NDOF mode\n");
        close(i2c_fd);
        return 1;
    }
    msleep(80);

    // ちょい待ち（安定化）
    bno055_brief_settle_wait(500); // 0.5秒だけ
    bno055_print_calib_stat();

    // offset読み込み
    float heading_offset = 0.0f;
    if (load_heading_offset(OFFSET_FILE, &heading_offset) != 0) {
        printf("ERROR: Heading offset file '%s' not found or failed to load.\n", OFFSET_FILE);
        printf("Please run 9ziku.c once to generate it.\n");
        close(i2c_fd);
        return 1;
    }
    printf("Heading offset loaded: %.2f deg (using %s)\n", heading_offset, OFFSET_FILE);

    // --- Hall baseline ---
    printf("\nStarting Hall sensors (CH1-CH6) monitoring...\n");
    printf("Baseline sampling...\n");

    double baseline[NUM_CH] = {0};

    for (int s = 0; s < BASELINE_SAMPLES; s++) {
        for (int ch = 1; ch < NUM_CH; ch++) {
            double v = read_adc_voltage(ch);
            if (!isnan(v)) baseline[ch] += v;
        }
        usleep(100000);
    }
    for (int ch = 1; ch < NUM_CH; ch++) baseline[ch] /= BASELINE_SAMPLES;

    printf("Baseline established:\n");
    for (int ch = 1; ch < NUM_CH; ch++) {
        printf(" CH%d = %.3f V\n", ch, baseline[ch]);
    }

    printf("\nBegin monitoring...\n");
    printf("  (ignore channels if baseline<1.0V or window<1.0V)\n\n");

    while (1) {
        double window[NUM_CH];
        for (int ch = 1; ch < NUM_CH; ch++) window[ch] = read_adc_voltage(ch);

        double maxDiff = 0.0;
        int maxCh = -1;

        for (int ch = 1; ch < NUM_CH; ch++) {
            if (isnan(window[ch])) continue;

            // 1.0V以下は完全無視
            if (baseline[ch] < 1.0 || window[ch] < 1.0) continue;

            double diff = fabs(window[ch] - baseline[ch]);
            if (diff > THRESHOLD && diff > maxDiff) {
                maxDiff = diff;
                maxCh = ch;
            }
        }

        if (maxCh != -1) {
            float heading = bno055_heading_north(heading_offset);

            if (isnan(heading)) {
                fprintf(stderr, "Failed to read BNO055 heading. Skipping output.\n");
            } else {
                printf("CH%d confirmed: diff=%.3f V  HeadingNorth=%.1f°\n", maxCh, maxDiff, heading);

                const char *detected_surface = NULL;

                // CH1: TOP
                if (maxCh == 1) {
                    detected_surface = "TOP";
                }
                // CH6: BOTTOM
                else if (maxCh == 6) {
                    detected_surface = "BOTTOM";
                }
                // Side: CH2..CH5
                else {
                    // Headingで回転セクション決定（0/90/180/270）
                    int rotation_index = 0; // 0:北, 1:東, 2:南, 3:西

                    if (heading >= 45.0f && heading < 135.0f)      rotation_index = 1;
                    else if (heading >= 135.0f && heading < 225.0f) rotation_index = 2;
                    else if (heading >= 225.0f && heading < 315.0f) rotation_index = 3;
                    else                                            rotation_index = 0;

                    // 物理配置（0度回転時の仮定）
                    // CH2=FRONT(0), CH3=LEFT(1), CH4=BACK(2), CH5=RIGHT(3)
                    int initial_index = -1;
                    if      (maxCh == 2) initial_index = 0;
                    else if (maxCh == 3) initial_index = 1;
                    else if (maxCh == 4) initial_index = 2;
                    else if (maxCh == 5) initial_index = 3;

                    if (initial_index != -1) {
                        int mapped_index = (initial_index + rotation_index) % 4;
                        const char *final_names[] = {"FRONT", "LEFT", "BACK", "RIGHT"};
                        detected_surface = final_names[mapped_index];
                    } else {
                        detected_surface = "ERROR_SIDE";
                    }
                }

                printf("Detected Surface: %s\n\n", detected_surface);

                // 検知後待機（元の挙動維持）
                sleep(2);
            }
        }

        usleep(100000);
    }

    close(i2c_fd);
    return 0;
}
