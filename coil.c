// coil.c (実行ファイル版)
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/stat.h>

// BCM 20 を使用 (物理ピン38番)
const int BCM_GPIO_PIN = 20;

/// 通常の書き込み（エラーは表示）
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

/// エラーを表示しない書き込み（unexport用）
int writeFileSilent(const char *path, const char *value) {
    FILE *f = fopen(path, "w");
    if (!f) {
        return -1;
    }
    if (fprintf(f, "%s", value) < 0) {
        fclose(f);
        return -1;
    }
    fclose(f);
    return 0;
}

/// ファイルが存在するかどうか
int fileExists(const char *path) {
    struct stat st;
    return (stat(path, &st) == 0);
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

int main(int argc, char *argv[]) {
    // gpiochip512 の base を読む (環境に応じてこのパスは変わる可能性があります)
    int base = readInt("/sys/class/gpio/gpiochip512/base");
    if (base < 0) {
        return 1;
    }

    int linux_gpio = base + BCM_GPIO_PIN; // sysfs で使う実際の番号 (例: 512 + 20 = 532)
    char num_str[16];
    snprintf(num_str, sizeof(num_str), "%d", linux_gpio);

    char path_dir[128];
    char path_val[128];
    snprintf(path_dir, sizeof(path_dir),
             "/sys/class/gpio/gpio%d/direction", linux_gpio);
    snprintf(path_val, sizeof(path_val),
             "/sys/class/gpio/gpio%d/value", linux_gpio);

    // コマンド解釈
    const char *cmd = argc >= 2 ? argv[1] : "start_on"; 

    if (strcmp(cmd, "start_on") == 0) {
        // ★ ON 処理: export → out 設定 → ON 
        
        // 1. 確実に unexport を試みる (クリーンアップ)
        writeFileSilent("/sys/class/gpio/unexport", num_str);
        usleep(100000); 

        // 2. GPIO を export
        if (writeFile("/sys/class/gpio/export", num_str) < 0) {
             fprintf(stderr, "ERROR: Failed to export GPIO %d.\n", linux_gpio);
             return 1;
        }
        usleep(100000); 

        // 3. 方向を out に設定
        if (writeFile(path_dir, "out") < 0) {
            fprintf(stderr, "ERROR: Failed to set direction to 'out'.\n");
            return 1;
        }

        // 4. ON にする
        printf("COIL: ON\n");
        if (writeFile(path_val, "1") < 0) {
            fprintf(stderr, "ERROR: Failed to write '1' to value.\n");
            return 1;
        }
        
    } else if (strcmp(cmd, "stop_off") == 0) {
        // ★ OFF 処理: OFF → unexport 

        if (fileExists(path_val)) {
            // 1. OFF にする
            printf("COIL: OFF\n");
            writeFileSilent(path_val, "0");
            
            // 2. unexport
            writeFileSilent("/sys/class/gpio/unexport", num_str);
        } else {
            // すでに片付いている
        }
    } else {
        fprintf(stderr,
                "Usage: %s [start_on | stop_off | start [seconds]]\n"
                "  no args: start_on (runs start_on by default)\n", argv[0]);
        return 1;
    }

    return 0;
}
