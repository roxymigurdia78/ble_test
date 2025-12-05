#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pigpio.h>

#define BUTTON_PIN 16
// ↓ここにあなたの環境の正しいパスを入れてください
#define TARGET_EXEC "/home/ras4/ble_test/ble_dual"

// フラグ変数（0:押されていない, 1:押された）
volatile int trigger_flag = 0;
volatile uint32_t last_tick = 0;

// 割り込み関数は「旗を立てる」だけにする（軽くて安全）
void button_callback(int gpio, int level, uint32_t tick) {
    if (level == 0) { // 押された
        // ここで簡易デバウンス（300ms）
        if (tick - last_tick > 300000) {
            trigger_flag = 1;
            last_tick = tick;
        }
    }
}

int main() {
    if (gpioInitialise() < 0) return 1;

    // ピン設定
    gpioSetMode(BUTTON_PIN, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN, PI_PUD_UP);
    gpioGlitchFilter(BUTTON_PIN, 1000); // フィルタを少し強め(1000us)に変更

    // 割り込み設定
    gpioSetAlertFunc(BUTTON_PIN, button_callback);

    printf("監視開始: GPIO%d -> %s\n", BUTTON_PIN, TARGET_EXEC);

    while (1) {
        // フラグが立っていたら実行する
        if (trigger_flag == 1) {
            printf("スイッチ検知！ プログラムを起動します...\n");
            
            // 外部プログラム実行
            int ret = system(TARGET_EXEC);
            
            if (ret == -1) {
                printf("起動エラー\n");
            }

            // フラグを戻す
            trigger_flag = 0;
            
            // 連続起動を防ぐため少し待つ（2秒）
            sleep(2);
        }
        
        // CPU負荷を下げるための短い待機
        gpioDelay(50000); // 50ms
    }

    gpioTerminate();
    return 0;
}

