// 9ziku.c
// BNO055 9軸センサ: キャリブファイルがある場合は再キャリブせず、そのまま使用。
// 初回だけキャリブレーション＋北0度オフセット決めを行う。 
//
// ビルド:
//   gcc 9ziku.c -o 9ziku
// 実行:
//   sudo ./9ziku
//
// リセットしたいとき:
//   rm bno055_calib.bin bno055_heading_offset.txt
//   sudo ./9ziku   ← 再キャリブと北0度取り直し

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <string.h>
#include <errno.h>

#define I2C_DEV_PATH       "/dev/i2c-1"
#define BNO055_ADDRESS     0x28
#define BNO055_ID_EXPECTED 0xA0

// --- BNO055 レジスタ ---
#define BNO055_CHIP_ID_ADDR    0x00
#define BNO055_EUL_HEADING_LSB 0x1A  // Heading, Roll, Pitch (LSB〜)
#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_UNIT_SEL_ADDR   0x3B
#define BNO055_PWR_MODE_ADDR   0x3E
#define BNO055_OPR_MODE_ADDR   0x3D
#define BNO055_SYS_TRIGGER     0x3F

// キャリブレーションデータ
#define BNO055_CALIB_START     0x55
#define BNO055_CALIB_DATA_LEN  22

// 動作モード
#define OPERATION_MODE_CONFIG  0x00
#define OPERATION_MODE_NDOF    0x0C  // 磁気・重力・ジャイロ全部使うモード

// 電源モード
#define POWER_MODE_NORMAL      0x00

// 1 LSB = 1/16度
#define EULER_UNIT             16.0f

static int i2c_fd = -1;

// ファイル名
const char *CALIB_FILE   = "bno055_calib.bin";
const char *OFFSET_FILE  = "bno055_heading_offset.txt";

static void msleep(int ms) {
    usleep(ms * 1000);
}

// ================= I2C 基本 =================

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

// ================= BNO055 初期化 =================

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

// ================= キャリブ保存/読込 =================

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

// ================= Euler角 読み出し =================

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

// ================= Headingオフセット 保存/読込 =================

// return 0:OK, -1:なし/失敗
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

// return 0:OK, -1:失敗
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

// ================= main =================

int main(void) {
    if (i2c_open() < 0) {
        return 1;
    }

    if (bno055_init() < 0) {
        fprintf(stderr, "BNO055 init failed\n");
        close(i2c_fd);
        return 1;
    }

    int need_recal = 0;

    // --- キャリブレーション復元 or 新規作成 ---
    if (bno055_load_calibration(CALIB_FILE) == 0) {
        printf("Using stored calibration.\n");
        need_recal = 0;   // ファイルがあるので原則再キャリブしない
    } else {
        printf("No valid calibration file. Will create new one.\n");
        need_recal = 1;   // この起動で一度だけキャリブする
    }

    // NDOFモードへ
    if (bno055_set_mode(OPERATION_MODE_NDOF) < 0) {
        fprintf(stderr, "Failed to set NDOF mode\n");
        close(i2c_fd);
        return 1;
    }
    msleep(50);

    // --- キャリブ状態確認＆必要ならキャリブ ---
    if (need_recal) {
        uint8_t stat;
        if (i2c_read8(BNO055_CALIB_STAT_ADDR, &stat) == 0) {
            int sys = (stat >> 6) & 0x03;
            int gyr = (stat >> 4) & 0x03;
            int acc = (stat >> 2) & 0x03;
            int mag = (stat >> 0) & 0x03;

            printf("Initial CALIB SYS:%d ACC:%d GYR:%d MAG:%d\n",
                   sys, acc, gyr, mag);

            // まだフルじゃなければ一回だけ動かしてキャリブ
            if (!(acc == 3 && gyr == 3 && mag == 3)) {
                bno055_wait_full_calibration();
            } else {
                printf("Sensor already fully calibrated.\n");
            }

            // ★ フル状態になったキャリブを必ず保存する
            bno055_save_calibration(CALIB_FILE);
        }
    } else {
        // 再キャリブは強制しない。状態だけ表示。
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

    // --- Heading オフセット（北=0°に固定） ---
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
            printf("North reference captured. raw heading = %.2f deg\n", heading_raw);
            save_heading_offset(OFFSET_FILE, heading_offset);
        } else {
            fprintf(stderr, "Failed to read heading for offset calibration.\n");
        }
    }

    printf("\nStart reading Euler angles (Ctrl+C to stop)\n");
    printf("  -> 表示: HeadingNorth = 北を0°とした角度 / HeadingRaw = センサ生値\n");

    while (1) {
        float heading_raw, roll, pitch;
        if (bno055_read_euler(&heading_raw, &roll, &pitch) == 0) {

            // オフセット補正（北=0°）
            float heading_north = heading_raw - heading_offset;
            // 0〜360に丸める
            while (heading_north < 0.0f)   heading_north += 360.0f;
            while (heading_north >= 360.0f) heading_north -= 360.0f;

            printf("\rHeadingNorth: %7.2f deg  (raw: %7.2f)  Roll:%7.2f  Pitch:%7.2f  ",
                   heading_north, heading_raw, roll, pitch);
            fflush(stdout);
        } else {
            fprintf(stderr, "\nFailed to read Euler angles\n");
        }
        msleep(100);
    }

    close(i2c_fd);
    return 0;
}
