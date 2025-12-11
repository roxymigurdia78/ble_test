#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>
#include <wiringPi.h> // GPIO制御用に追加

// LEDのピン番号 (button_dual.cと同じBCM番号に合わせてください)
// ※以前のコードに基づき 赤:12, 緑:13, 青:19 と仮定しています
#define PIN_LED_RED    12
#define PIN_LED_GREEN  13
#define PIN_LED_BLUE   19

int run_cmd(const char *cmd) {
    int status = system(cmd);
    if (status == -1) return -1;
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

int main(void) {
    printf("=== stop_dual: 停止＆消灯ツール (C言語版) ===\n");

    // 0. GPIO初期化 (消灯処理のため)
    if (wiringPiSetupGpio() < 0) {
        printf("WiringPi Setup failed. ライト消灯スキップ.\n");
    } else {
        pinMode(PIN_LED_RED, OUTPUT);
        pinMode(PIN_LED_GREEN, OUTPUT);
        pinMode(PIN_LED_BLUE, OUTPUT);
    }

    printf("1. systemd サービス停止...\n");
    run_cmd("systemctl stop button_dual.service");

    printf("2. プロセス停止...\n");
    run_cmd("pkill -f button_dual");
    // run_cmd("pkill -f led_control.py"); // Pythonはもう動いていないので削除

    printf("3. Bluetooth停止...\n");
    run_cmd("hciconfig hci0 noleadv");

    printf("4. ライトを消灯(off)...\n");
    // Pythonではなく直接消す
    digitalWrite(PIN_LED_RED, 0);
    digitalWrite(PIN_LED_GREEN, 0);
    digitalWrite(PIN_LED_BLUE, 0);

    printf("✅ 完了\n");
    return 0;
}
