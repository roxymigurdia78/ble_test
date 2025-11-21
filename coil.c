#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>

/// ファイルに文字列を書き込む
int writeFile(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) {
        perror(path);
        return -1;
    }
    if (fprintf(f, "%s", value) < 0) {
        perror("fprintf");
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

/// ファイルから int を読み込む（gpiochip の base を読む用）
int readInt(const char *path) {
    FILE *f = fopen(path, "r");
    if (!f) {
        perror(path);
        return -1;
    }
    int v;
    if (fscanf(f, "%d", &v) != 1) {
        fprintf(stderr, "failed to read int from %s\n", path);
        fclose(f);
        return -1;
    }
    fclose(f);
    return v;
}

int main(void) {
    const int bcm_gpio = 20; // 物理ピン38のGPIO番号(BCM)

    // ★ ここ：gpiochip512 の base を読む
    int base = readInt("/sys/class/gpio/gpiochip512/base");
    if (base < 0) {
        fprintf(stderr, "gpiochip base 読み込み失敗\n");
        return 1;
    }

    int linux_gpio = base + bcm_gpio; // sysfs で使う実際の番号
    char num_str[16];
    snprintf(num_str, sizeof(num_str), "%d", linux_gpio);

    char path_dir[128];
    char path_val[128];
    snprintf(path_dir, sizeof(path_dir),
             "/sys/class/gpio/gpio%d/direction", linux_gpio);
    snprintf(path_val, sizeof(path_val),
             "/sys/class/gpio/gpio%d/value", linux_gpio);

    // GPIO を export
    if (writeFile("/sys/class/gpio/export", num_str) < 0) {
        fprintf(stderr, "export に失敗しました\n");
        return 1;
    }

    // ちょっと待つ（ディレクトリが出来るのを待つ）
    usleep(100000);

    // 方向を out に設定
    if (writeFile(path_dir, "out") < 0) {
        fprintf(stderr, "direction 設定に失敗しました\n");
        return 1;
    }

    //printf("BCM GPIO%d (Linux GPIO%d) を 10秒ごとにON/OFF します\n",
           //bcm_gpio, linux_gpio);


    while (1) {
        // ON
        printf("ON\n");
        if (writeFile(path_val, "1") < 0) break;
        sleep(1000000000);

        // OFF
        printf("OFF\n");
        if (writeFile(path_val, "0") < 0) break;
        sleep(10);
    }

    // 終了時に unexport（失敗してもとりあえず続行）
    writeFile("/sys/class/gpio/unexport", num_str);

    return 0;
 }
