#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#define BNO055_ADDR 0x28
#define CHIP_ID_REG 0x00

int main(void) {
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) { perror("open"); return 1; }

    if (ioctl(fd, I2C_SLAVE, BNO055_ADDR) < 0) { perror("ioctl"); return 1; }

    uint8_t reg = CHIP_ID_REG;
    uint8_t val = 0;

    if (write(fd, &reg, 1) != 1) { perror("write"); return 1; }
    if (read(fd, &val, 1) != 1)  { perror("read");  return 1; }

    printf("CHIP_ID = 0x%02X\n", val);
    close(fd);
    return 0;
}
