// hall_9ziku.c
// MCP3208(12bit ADC) + アナログホールIC + BNO055(9軸) 連携プログラム
//
// ・MCP3208 で複数 ch のホールICを監視
// ・磁場変化を検知 → どの ch か表示
// ・検知から 15 秒後に BNO055 の角度を 1 回読み取って表示
//
// ビルド:
//   gcc hall_9ziku.c -o hall_9ziku
// 実行:
//   sudo ./hall_9ziku
//
// 初回だけ：BNO055 のキャリブと北0°の決定を求められます。
// 以降は bno055_calib.bin / bno055_heading_offset.txt を使い回します。

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <string.h>
#include <errno.h>

// ===================== 設定 =====================

// ==== MCP3208（12bit ADC） ====
#define SPI_DEV_PATH       "/dev/spidev0.0"  // SPI0 CE0 を使用（必要なら 0.1 に変更）
#define SPI_MODE_VAL       SPI_MODE_0
#define SPI_BITS_PER_WORD  8
#define SPI_SPEED_HZ       1000000           // 1MHz

#define NUM_HALL_CH        6          // 使用するホールICチャンネル数 (CH0〜CH5)
#define HALL_THRESHOLD     80         // 磁場検知のしきい値 (要調整, 0〜4095)
#define HALL_RELEASE       40         // 解除しきい値 (ヒステリシス用)
#define VREF               3.3        // ADC基準電圧
#define MAX_ADC_12BIT      4095       // MCP3208 は 12bit

// ==== BNO055（9軸） ====
#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0

// BNO055 レジスタ
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_UNIT_SEL_ADDR   0x3B
#define BNO055_PWR_MODE_ADDR   0x3E
#define BNO055_OPR_MODE_ADDR   0x3D
#define BNO055_SYS_TRIGGER     0x3F

#define BNO055_CALIB_START     0x55
#define BNO055_CALIB_DATA_LEN  22

#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C
#define POWER_MODE_NORMAL      0x00

#define EULER_UNIT             16.0f  // 1 LSB = 1/16 deg

// ファイル名
const char *CALIB_FILE  = "bno055_calib.bin";
const char *OFFSET_FILE = "bno055_heading_offset.txt";

// ホールICのスキャン周期
#define HALL_SCAN_INTERVAL_US  50000   // 50ms

// 9軸読み取りまでの待ち時間
#define ORIENT_DELAY_SEC       15      // 15秒

// =================================================
// 共通ユーティリティ
// =================================================

static void msleep(int ms) {
    usleep(ms * 1000);
}

// =================================================
// MCP3208 関連（spidev）
// =================================================

static int spi_fd = -1;

static int spi_open_mcp3208(void) {
    spi_fd = open(SPI_DEV_PATH, O_RDWR);
    if (spi_fd < 0) {
        perror("open(SPI_DEV_PATH)");
        return -1;
    }

    uint8_t mode = SPI_MODE_VAL;
    uint8_t bits = SPI_BITS_PER_WORD;
    uint32_t speed = SPI_SPEED_HZ;

    if (ioctl(spi_fd, SPI_IOC_WR_MODE, &mode) == -1 ||
        ioctl(spi_fd, SPI_IOC_RD_MODE, &mode) == -1) {
        perror("ioctl(SPI_IOC_*_MODE)");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1 ||
        ioctl(spi_fd, SPI_IOC_RD_BITS_PER_WORD, &bits) == -1) {
        perror("ioctl(SPI_IOC_*_BITS_PER_WORD)");
        return -1;
    }
    if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) == -1 ||
        ioctl(spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed) == -1) {
        perror("ioctl(SPI_IOC_*_MAX_SPEED_HZ)");
        return -1;
    }

    return 0;
}

// MCP3208 の指定チャンネルを読み取る（12bit, 0〜4095）
static int read_mcp3208_channel(int channel) {
    if (channel < 0 || channel > 7) return -1;

    uint8_t tx[3];
    uint8_t rx[3];

    // MCP3208:
    // tx[0]: 0000 0110 or 0000 0111 (start=1, single=1, D2)
    // tx[1]: (D1,D0,xxxx xx)
    // tx[2]: dummy
    tx[0] = 0x06 | ((channel & 0x04) >> 2);  // start + single + D2
    tx[1] = (channel & 0x03) << 6;           // D1,D0
    tx[2] = 0x00;

    struct spi_ioc_transfer tr = {
        .tx_buf = (unsigned long)tx,
        .rx_buf = (unsigned long)rx,
        .len = 3,
        .speed_hz = SPI_SPEED_HZ,
        .delay_usecs = 0,
        .bits_per_word = SPI_BITS_PER_WORD,
    };

    if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        perror("SPI_IOC_MESSAGE");
        return -1;
    }

    int value = ((rx[1] & 0x0F) << 8) | rx[2];  // 上位4bit + 下位8bit = 12bit
    return value;
}

// =================================================
// BNO055 関連（I2C）
// =================================================

static int i2c_fd = -1;

static int i2c_open_bno055(void) {
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

static int bno055_set_mode(uint8_t mode) {
    if (i2c_write8(BNO055_OPR_MODE_ADDR, mode) < 0)
        return -1;
    msleep(30);
    return 0;
}

static int bno055_init(void) {
    uint8_t id;
    if (i2c_read8(BNO055_CHIP_ID_ADDR, &id) < 0) {
        fprintf(stderr, "Failed to read chip ID\n");
        return -1;
    }
    if (id != BNO055_ID_EXPECTED) {
        fprintf(stderr, "Unexpected BNO055 ID: 0x%02X (expected 0x%02X)\n",
                id, BNO055_ID_EXPECTED);
        return -1;
    }
    printf("BNO055 detected! ID=0x%02X\n", id);

    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0)
        return -1;

    if (i2c_write8(BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL) < 0)
        return -1;
    msleep(10);

    // 単位設定: デフォルト (deg)
    if (i2c_write8(BNO055_UNIT_SEL_ADDR, 0x00) < 0)
        return -1;
    msleep(10);

    if (i2c_write8(BNO055_SYS_TRIGGER, 0x00) < 0)
        return -1;
    msleep(10);

    return 0;
}

// キャリブレーション保存
static int bno055_save_calibration(const char *path) {
    uint8_t mode_before;
    uint8_t calib[BNO055_CALIB_DATA_LEN];

    if (i2c_read8(BNO055_OPR_MODE_ADDR, &mode_before) < 0)
        return -1;

    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0)
        return -1;

    if (i2c_read_len(BNO055_CALIB_START, calib, BNO055_CALIB_DATA_LEN) < 0) {
        fprintf(stderr, "Failed to read calibration data\n");
        bno055_set_mode(mode_before);
        return -1;
    }

    FILE *fp = fopen(path, "wb");
    if (!fp) {
        perror("fopen(calib save)");
        bno055_set_mode(mode_before);
        return -1;
    }
    if (fwrite(calib, 1, BNO055_CALIB_DATA_LEN, fp) != BNO055_CALIB_DATA_LEN) {
        fprintf(stderr, "Failed to write calibration file\n");
        fclose(fp);
        bno055_set_mode(mode_before);
        return -1;
    }
    fclose(fp);
    printf("Calibration saved to %s\n", path);

    if (bno055_set_mode(mode_before) < 0)
        return -1;

    return 0;
}

// キャリブレーション読込
static int bno055_load_calibration(const char *path) {
    uint8_t mode_before;
    uint8_t calib[BNO055_CALIB_DATA_LEN];

    FILE *fp = fopen(path, "rb");
    if (!fp) {
        return -1; // ファイルがない
    }
    size_t n = fread(calib, 1, BNO055_CALIB_DATA_LEN, fp);
    fclose(fp);
    if (n != BNO055_CALIB_DATA_LEN) {
        fprintf(stderr, "Calibration file size mismatch\n");
        return -1;
    }

    if (i2c_read8(BNO055_OPR_MODE_ADDR, &mode_before) < 0)
        return -1;

    if (bno055_set_mode(OPERATION_MODE_CONFIG) < 0)
        return -1;

    for (int i = 0; i < BNO055_CALIB_DATA_LEN; i++) {
        if (i2c_write8(BNO055_CALIB_START + i, calib[i]) < 0) {
            fprintf(stderr, "Failed to write calibration register\n");
            bno055_set_mode(mode_before);
            return -1;
        }
    }

    printf("Calibration loaded from %s\n", path);

    if (bno055_set_mode(mode_before) < 0)
        return -1;

    return 0;
}

static void bno055_wait_full_calibration(void) {
    printf("Waiting for full calibration...\n");
    printf("Move the sensor slowly in all directions (figure-8 etc.).\n");

    while (1) {
        uint8_t stat;
        if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) < 0) {
            fprintf(stderr, "Failed to read calibration status\n");
            break;
        }

        int sys = (stat >> 6) & 0x03;
        int gyr = (stat >> 4) & 0x03;
        int acc = (stat >> 2) & 0x03;
        int mag = (stat >> 0) & 0x03;

        printf("\rCALIB SYS:%d ACC:%d GYR:%d MAG:%d   ", sys, acc, gyr, mag);
        fflush(stdout);

        if (acc == 3 && gyr == 3 && mag == 3) {
            printf("\nFull calibration achieved!\n");
            break;
        }
        msleep(200);
    }
}

// Euler角読み取り
static int bno055_read_euler(float *heading, float *roll, float *pitch) {
    uint8_t buf[6];
    if (i2c_read_len(BNO055_EUL_HEADING_LSB, buf, 6) < 0) {
        return -1;
    }

    int16_t raw_heading = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t raw_roll    = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t raw_pitch   = (int16_t)((buf[5] << 8) | buf[4]);

    *heading = raw_heading / EULER_UNIT;
    *roll    = raw_roll    / EULER_UNIT;
    *pitch   = raw_pitch   / EULER_UNIT;

    return 0;
}

// Heading オフセット（北=0°用）
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

static int save_heading_offset(const char *path, float offset) {
    FILE *fp = fopen(path, "w");
    if (!fp) {
        perror("fopen(offset save)");
        return -1;
    }
    fprintf(fp, "%.6f\n", offset);
    fclose(fp);
    printf("Heading offset saved to %s (offset=%.2f deg)\n", path, offset);
    return 0;
}

// =================================================
// メイン
// =================================================

int main(void) {
    // --- SPI(MCP3208) 初期化 ---
    if (spi_open_mcp3208() < 0) {
        return 1;
    }

    // --- BNO055(I2C) 初期化 ---
    if (i2c_open_bno055() < 0) {
        close(spi_fd);
        return 1;
    }
    if (bno055_init() < 0) {
        fprintf(stderr, "BNO055 init failed\n");
        close(i2c_fd);
        close(spi_fd);
        return 1;
    }

    // キャリブレーション復元 or 初回キャリブ
    int need_recal = 0;
    if (bno055_load_calibration(CALIB_FILE) == 0) {
        printf("Using stored calibration.\n");
        need_recal = 0;
    } else {
        printf("No valid calibration file. Will create new one.\n");
        need_recal = 1;
    }

    // NDOFモードへ
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "Failed to set NDOF mode\n");
        close(i2c_fd);
        close(spi_fd);
        return 1;
    }
    msleep(50);

    if (need_recal) {
        uint8_t stat;
        if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) == 0) {
            int sys = (stat >> 6) & 0x03;
            int gyr = (stat >> 4) & 0x03;
            int acc = (stat >> 2) & 0x03;
            int mag = (stat >> 0) & 0x03;

            printf("Initial CALIB SYS:%d ACC:%d GYR:%d MAG:%d\n",
                   sys, acc, gyr, mag);

            if (!(acc == 3 && gyr == 3 && mag == 3)) {
                bno055_wait_full_calibration();
            } else {
                printf("Sensor already fully calibrated.\n");
            }
            bno055_save_calibration(CALIB_FILE);
        }
    } else {
        uint8_t stat;
        if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) == 0) {
            int sys = (stat >> 6) & 0x03;
            int gyr = (stat >> 4) & 0x03;
            int acc = (stat >> 2) & 0x03;
            int mag = (stat >> 0) & 0x03;
            printf("CALIB (no re-cal) SYS:%d ACC:%d GYR:%d MAG:%d\n",
                   sys, acc, gyr, mag);
        }
    }

    // Heading オフセット読み込み（なければ今の向きを0°にする）
    float heading_offset = 0.0f;
    if (load_heading_offset(OFFSET_FILE, &heading_offset) == 0) {
        printf("Heading offset loaded: %.2f deg\n", heading_offset);
    } else {
        printf("\nNo heading offset file.\n");
        printf("★ 今から「北を0度に決める」作業をします。\n");
        printf("  センサ(キューブ)の『前』を、実際の北方向に向けてください。\n");
        printf("  向けたら Enter キーを押してください。\n");
        printf("  （スマホのコンパスなどで“北”をざっくりでOK）\n\n");
        getchar(); // Enter待ち

        float heading_raw, roll_dummy, pitch_dummy;
        if (bno055_read_euler(&heading_raw, &roll_dummy, &pitch_dummy) == 0) {
            heading_offset = heading_raw;
            printf("North reference captured. raw heading = %.2f deg\n",
                   heading_raw);
            save_heading_offset(OFFSET_FILE, heading_offset);
        } else {
            fprintf(stderr, "Failed to read heading for offset calibration.\n");
        }
    }

    // --- ホールIC用 ベースライン取得 ---
    int baseline[NUM_HALL_CH];
    int hall_state[NUM_HALL_CH]; // 0:待機, 1:磁場あり

    printf("\nホールICのベースライン計測中...\n");
    for (int ch = 0; ch < NUM_HALL_CH; ch++) {
        long sum = 0;
        int samples = 20;
        for (int i = 0; i < samples; i++) {
            int v = read_mcp3208_channel(ch);
            if (v < 0) {
                fprintf(stderr, "CH%d: ADC read error during baseline\n", ch);
                i--;
                continue;
            }
            sum += v;
            usleep(5000);
        }
        baseline[ch] = (int)(sum / samples);
        hall_state[ch] = 0;
        printf("  CH%d baseline = %d\n", ch, baseline[ch]);
    }
    printf("ベースライン計測完了。\n\n");

    printf("=== ホールIC監視開始 ===\n");
    printf("  ・磁場変化を検知するとチャンネル番号を表示\n");
    printf("  ・検知から %d 秒後に BNO055 の角度を1回読み取って表示\n\n",
           ORIENT_DELAY_SEC);

    while (1) {
        for (int ch = 0; ch < NUM_HALL_CH; ch++) {
            int adc = read_mcp3208_channel(ch);
            if (adc < 0) {
                fprintf(stderr, "CH%d: ADC read error\n", ch);
                continue;
            }
            int delta = adc - baseline[ch];
            if (delta < 0) delta = -delta;

            // 磁場検知（立ち上がり）
            if (hall_state[ch] == 0 && delta > HALL_THRESHOLD) {
                hall_state[ch] = 1;

                double voltage = (double)adc / MAX_ADC_12BIT * VREF;

                printf("\n=== Hall event detected on CH%d ===\n", ch);
                printf("  ADC value = %d / 4095, voltage = %.3f V\n",
                       adc, voltage);
                printf("  baseline = %d, delta = %d (> %d)\n",
                       baseline[ch], delta, HALL_THRESHOLD);
                printf("  -> %d 秒後に BNO055 から角度を読み取ります...\n",
                       ORIENT_DELAY_SEC);

                // 電磁石の影響が落ち着くのを待つ（電磁石は別ラズパイ駆動前提）
                sleep(ORIENT_DELAY_SEC);

                // 9軸から角度読み取り
                float heading_raw, roll, pitch;
                if (bno055_read_euler(&heading_raw, &roll, &pitch) == 0) {
                    float heading_north = heading_raw - heading_offset;
                    while (heading_north < 0.0f)   heading_north += 360.0f;
                    while (heading_north >= 360.0f) heading_north -= 360.0f;

                    printf("  [9-axis] CH%d のイベント時の姿勢:\n", ch);
                    printf("    HeadingNorth = %.2f deg (北=0°換算)\n",
                           heading_north);
                    printf("    HeadingRaw   = %.2f deg\n", heading_raw);
                    printf("    Roll         = %.2f deg\n", roll);
                    printf("    Pitch        = %.2f deg\n\n", pitch);
                } else {
                    fprintf(stderr,
                            "  [9-axis] BNO055 Euler 読み取りに失敗しました。\n");
                }
            }
            // 磁場解除（ヒステリシス）
            else if (hall_state[ch] == 1 && delta < HALL_RELEASE) {
                hall_state[ch] = 0;
            }
        }

        usleep(HALL_SCAN_INTERVAL_US);
    }

    // 通常ここには来ないけど一応
    close(i2c_fd);
    close(spi_fd);
    return 0;
}
