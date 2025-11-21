#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define BNO055_ADDRESS_A 0x28
#define BNO055_ID        0xA0

#define BNO055_CHIP_ID_ADDR      0x00
#define BNO055_OPR_MODE_ADDR     0x3D
#define BNO055_PWR_MODE_ADDR     0x3E
#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_CALIB_STAT_ADDR   0x35

#define OPERATION_MODE_CONFIG    0x00
#define OPERATION_MODE_NDOF      0x0C

// I2C書き込み（最大3回リトライ）
int write_reg(int file, char reg, char value) {
    char buf[2] = {reg, value};
    for (int i = 0; i < 3; i++) {
        if (write(file, buf, 2) == 2) return 0;
        usleep(50000);
    }
    return -1;
}

// I2C読み出し（最大3回リトライ）
int read_reg(int file, char reg, char *data, int len) {
    for (int i = 0; i < 3; i++) {
        if (write(file, &reg, 1) == 1 && read(file, data, len) == len) return 0;
        usleep(50000);
    }
    return -1;
}

int main() {
    int file;
    char *bus = "/dev/i2c-1";

    if ((file = open(bus, O_RDWR)) < 0) {
        perror("Failed to open I2C bus");
        return 1;
    }
    if (ioctl(file, I2C_SLAVE, BNO055_ADDRESS_A) < 0) {
        perror("Failed to connect to sensor");
        return 1;
    }

    // チップID確認
    char id;
    if (read_reg(file, BNO055_CHIP_ID_ADDR, &id, 1) < 0) {
        perror("Failed to read chip ID");
        return 1;
    }
    if (id != BNO055_ID) {
        printf("BNO055 not detected! ID=0x%X\n", id);
        return 1;
    }
    printf("BNO055 detected! ID=0x%X\n", id);

    // CONFIGモード
    if (write_reg(file, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG) < 0) {
        perror("Failed to set CONFIG mode");
        return 1;
    }
    usleep(50000);

    // NDOFモード
    if (write_reg(file, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF) < 0) {
        perror("Failed to set NDOF mode");
        return 1;
    }
    usleep(2000000); // 2秒待機

    printf("Calibrating sensors...\n");
    printf("Keep GYRO still and move sensor in 8-shape for MAG.\n");

    int sys=0, acc=0, gyro=0, mag=0;

    // キャリブレーション完了までループ
    while (!(sys==3 && acc==3 && gyro==3 && mag==3)) {
        char calib;
        if (read_reg(file, BNO055_CALIB_STAT_ADDR, &calib, 1) < 0) {
            printf("Failed to read calibration status, retrying...\n");
            sleep(1);
            continue;
        }

        sys  = (calib >> 6) & 0x03;
        acc  = (calib >> 4) & 0x03;
        gyro = (calib >> 2) & 0x03;
        mag  = calib & 0x03;

        printf("CALIB SYS:%d ACC:%d GYRO:%d MAG:%d\n", sys, acc, gyro, mag);
        sleep(1);
    }

    printf("Calibration complete! Waiting 2 seconds to stabilize...\n");
    sleep(2);

    printf("Now reading Euler angles...\n");

    while (1) {
        char euler[6];
        if (read_reg(file, BNO055_EULER_H_LSB_ADDR, euler, 6) < 0) {
            printf("I2C read error, retrying...\n");
            sleep(1);
            continue;
        }

        int16_t heading = (int16_t)((euler[1] << 8) | euler[0]);
        int16_t roll    = (int16_t)((euler[3] << 8) | euler[2]);
        int16_t pitch   = (int16_t)((euler[5] << 8) | euler[4]);

        printf("Heading: %.2f°, Roll: %.2f°, Pitch: %.2f°\n",
               heading / 16.0, roll / 16.0, pitch / 16.0);

        sleep(1);
    }

    close(file);
    return 0;
}
