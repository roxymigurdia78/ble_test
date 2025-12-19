#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <pthread.h>

// ハードウェア制御ライブラリ
#include <wiringPi.h>
#include <softPwm.h>
#include <softTone.h>

// 楽譜データ (同じディレクトリに music_data.h がある前提)
#include "music_data.h"

// --- ピン設定 ---
#define PIN_SPK     27
#define PIN_LED_R   12
#define PIN_LED_G   13
#define PIN_LED_B   19

// 終了用ボタン
#define PIN_EXIT_BUTTON 16

// センサピン (4つ使用)
const int SENSORS[] = {0, 5, 6, 26};
#define NUM_SENSORS 4

// --- グローバル変数 ---
static volatile int is_playing = 0;
static unsigned long last_trigger_time = 0;

// ---------------------------------------------------------------------------
// ユーティリティ関数
// ---------------------------------------------------------------------------
static void sleep_ms(unsigned long ms) {
    if (ms == 0) return;
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

static void sleep_us(unsigned long us) {
    if (us == 0) return;
    struct timespec req;
    req.tv_sec = us / 1000000;
    req.tv_nsec = (us % 1000000) * 1000L;
    nanosleep(&req, NULL);
}

static inline unsigned long get_time_ms() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (unsigned long)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}

// ---------------------------------------------------------------------------
// LED制御
// ---------------------------------------------------------------------------
void set_led_color(int r, int g, int b) {
    if(r < 0) r = 0; if(r > 100) r = 100;
    if(g < 0) g = 0; if(g > 100) g = 100;
    if(b < 0) b = 0; if(b > 100) b = 100;

    softPwmWrite(PIN_LED_R, r);
    softPwmWrite(PIN_LED_G, g);
    softPwmWrite(PIN_LED_B, b);
}

// --- カラフルなプリセット色定義 ---
typedef struct {
    int r; int g; int b;
    char name[16];
} Color;

// 7色のリスト
const Color PRESET_COLORS[] = {
    {100,   0,   0, "RED"},     // 赤
    {  0, 100,   0, "GREEN"},   // 緑
    {  0,   0, 100, "BLUE"},    // 青
    {100, 100,   0, "YELLOW"},  // 黄 (赤+緑)
    {  0, 100, 100, "CYAN"},    // 水色 (緑+青)
    {100,   0, 100, "MAGENTA"}, // 紫 (赤+青)
    {100, 100, 100, "WHITE"}    // 白 (全部)
};
const int NUM_COLORS = 7;

// LEDスレッド関数: リストからランダムに1色選ぶ
void *led_thread_func(void *arg) {
    // ランダムにインデックスを決定 (0 〜 6)
    int idx = rand() % NUM_COLORS;
    
    Color c = PRESET_COLORS[idx];
    
    // コンソールに選ばれた色を表示
    printf("   [LED] Color Selected: %s (R:%d G:%d B:%d)\n", c.name, c.r, c.g, c.b);

    set_led_color(c.r, c.g, c.b);

    // 再生中はそのまま待機
    while (is_playing) {
        sleep_ms(100); 
    }
    
    // 終了時は消灯
    set_led_color(0,0,0);
    return NULL;
}

// ---------------------------------------------------------------------------
// 音楽再生制御
// ---------------------------------------------------------------------------
static void play_note_for_us(int freq, unsigned long usec) {
    if (freq > 0) {
        softToneWrite(PIN_SPK, freq);
        sleep_us(usec);
        softToneWrite(PIN_SPK, 0);
    } else {
        sleep_us(usec);
    }
}

void play_music(int song_id) {
    if (is_playing) return;
    is_playing = 1;

    // 常にパート1（メインメロディ）
    int part_id = 1; 
    Note *score = get_music_part(song_id, part_id);
    
    if (!score) {
        printf("[ERROR] Score not found for Song %d Part %d\n", song_id, part_id);
        is_playing = 0; 
        return;
    }

    printf("[PLAY] Song:%d (Part:%d)\n", song_id, part_id);

    // LEDスレッド開始
    pthread_t th_led;
    pthread_create(&th_led, NULL, led_thread_func, NULL);
    pthread_detach(th_led);

    // 演奏ループ
    int i = 0;
    const int GAP_US = 10000; 
    
    while (score[i].freq != -1 && is_playing) {
        if (digitalRead(PIN_EXIT_BUTTON) == 0) { is_playing = 0; break; }

        unsigned long total_duration_us = (unsigned long)(score[i].duration_ms * 1000.0f);
        if (score[i].freq > 0) {
            unsigned long play_us = (total_duration_us > GAP_US) ? (total_duration_us - GAP_US) : total_duration_us;
            play_note_for_us((int)score[i].freq, play_us);
            if (play_us < total_duration_us) sleep_us(GAP_US);
        } else {
            sleep_us(total_duration_us);
        }
        i++;
    }

    softToneWrite(PIN_SPK, 0);
    sleep_ms(200);
    is_playing = 0;
    set_led_color(0,0,0);
}

// ---------------------------------------------------------------------------
// 終了処理
// ---------------------------------------------------------------------------
void cleanup_and_exit(int sig) {
    printf("\n[SYSTEM] Shutting down...\n");
    is_playing = 0; 
    
    softPwmWrite(PIN_LED_R, 0); 
    softPwmWrite(PIN_LED_G, 0); 
    softPwmWrite(PIN_LED_B, 0);
    usleep(50000); 

    pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, 0);
    pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, 0);
    pinMode(PIN_LED_B, OUTPUT); digitalWrite(PIN_LED_B, 0);
    
    softToneWrite(PIN_SPK, 0);
    exit(0);
}

// ---------------------------------------------------------------------------
// メイン関数
// ---------------------------------------------------------------------------
int main() {
    printf("=== Simple Effect Standalone (Preset Colors) ===\n");
    printf("Waiting for sensor trigger...\n");
    
    signal(SIGINT, cleanup_and_exit);
    srand(time(NULL)); 

    if (wiringPiSetupGpio() == -1) return 1;

    pinMode(PIN_EXIT_BUTTON, INPUT); pullUpDnControl(PIN_EXIT_BUTTON, PUD_UP);
    for (int i=0; i<NUM_SENSORS; i++) { 
        pinMode(SENSORS[i], INPUT); 
        pullUpDnControl(SENSORS[i], PUD_DOWN); 
    }
    
    softPwmCreate(PIN_LED_R, 0, 100); 
    softPwmCreate(PIN_LED_G, 0, 100); 
    softPwmCreate(PIN_LED_B, 0, 100);
    softToneCreate(PIN_SPK);

    while (1) {
        if (digitalRead(PIN_EXIT_BUTTON) == 0) {
             sleep_ms(20); 
             if (digitalRead(PIN_EXIT_BUTTON) == 0) cleanup_and_exit(0);
        }

        int is_triggered = 0;
        if (!is_playing && (get_time_ms() - last_trigger_time > 3000)) {
            for (int i=0; i<NUM_SENSORS; i++) {
                if (digitalRead(SENSORS[i]) == HIGH) { 
                    is_triggered = 1; 
                    printf("Sensor %d Triggered!\n", SENSORS[i]);
                    break; 
                }
            }
        }

        if (is_triggered) {
            last_trigger_time = get_time_ms();
            int song_id = rand() % 2;
            play_music(song_id);
            sleep_ms(1000);
        }

        sleep_ms(50);
    }

    return 0;
}
