#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <math.h>
#include <wiringPi.h>
#include <softPwm.h>

// GPIO (BCM番号)
#define RED_PIN   12
#define GREEN_PIN 13
#define BLUE_PIN  19

// 明るさ補正 (0-100)
#define SCALE_R 100
#define SCALE_G 100
#define SCALE_B 100

// 終了フラグ
volatile int keepRunning = 1;

// Ctrl+C (SIGINT) を捕捉してLEDを消灯して終了するためのハンドラ
void sig_handler(int signo) {
    if (signo == SIGINT) {
        keepRunning = 0;
    }
}

// HSV to RGB 変換関数
// h, s, v: 0.0〜1.0
// r, g, b: ポインタ経由で 0〜100 の値を返す
void hsv_to_rgb(float h, float s, float v, int *r, int *g, int *b) {
    float r_f, g_f, b_f;
    
    int i = (int)(h * 6);
    float f = h * 6 - i;
    float p = v * (1 - s);
    float q = v * (1 - f * s);
    float t = v * (1 - (1 - f) * s);

    switch (i % 6) {
        case 0: r_f = v; g_f = t; b_f = p; break;
        case 1: r_f = q; g_f = v; b_f = p; break;
        case 2: r_f = p; g_f = v; b_f = t; break;
        case 3: r_f = p; g_f = q; b_f = v; break;
        case 4: r_f = t; g_f = p; b_f = v; break;
        case 5: r_f = v; g_f = p; b_f = q; break;
        default: r_f = 0; g_f = 0; b_f = 0; break;
    }

    // 0-100% にスケール変換し、補正値を適用
    *r = (int)(r_f * 100.0 * SCALE_R / 100.0);
    *g = (int)(g_f * 100.0 * SCALE_G / 100.0);
    *b = (int)(b_f * 100.0 * SCALE_B / 100.0);

    // 最大値制限
    if (*r > 100) *r = 100;
    if (*g > 100) *g = 100;
    if (*b > 100) *b = 100;
}

int main(void) {
    // 信号ハンドラの登録
    signal(SIGINT, sig_handler);

    // GPIO初期化 (BCM番号を使用)
    if (wiringPiSetupGpio() == -1) {
        printf("WiringPi setup failed!\n");
        return 1;
    }

    // ソフトウェアPWMの作成 (ピン, 初期値, レンジ)
    // レンジを100にすることで、0-100%の指定ができるようにする
    softPwmCreate(RED_PIN, 0, 100);
    softPwmCreate(GREEN_PIN, 0, 100);
    softPwmCreate(BLUE_PIN, 0, 100);

    float hue = 0.0;
    int r, g, b;

    printf("Start RGB Loop. Press Ctrl+C to stop.\n");

    while (keepRunning) {
        // HSV -> RGB 変換
        hsv_to_rgb(hue, 1.0, 1.0, &r, &g, &b);

        // PWM値の設定
        softPwmWrite(RED_PIN, r);
        softPwmWrite(GREEN_PIN, g);
        softPwmWrite(BLUE_PIN, b);

        // Hueを更新
        hue += 0.002;
        if (hue >= 1.0) {
            hue -= 1.0;
        }

        // 待機 (ミリ秒) -> Pythonの time.sleep(0.01) は 10ms
        delay(10);
    }

    // 終了処理：LEDを消灯
    softPwmWrite(RED_PIN, 0);
    softPwmWrite(GREEN_PIN, 0);
    softPwmWrite(BLUE_PIN, 0);
    
    // 少し待ってから終了（PWMスレッドへの反映待ち）
    delay(100); 

    printf("\nCleaned up and exited.\n");
    return 0;
}
