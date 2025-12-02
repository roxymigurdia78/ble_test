#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>

#include <wiringPi.h>

#define BUTTON_PIN      17          // BCM番号
#define HOLD_TIME_MS    1500        // 長押し判定時間（ミリ秒）
#define BLE_DUAL_PATH   "/home/pi/ble_test/ble_dual"  // ble_dual のフルパス

static volatile int running = 1;    // メインループ継続フラグ
static pid_t ble_pid = -1;          // 実行中の ble_dual の PID（なければ -1）

// Ctrl+C で綺麗に終了する用
void handle_sigint(int sig) {
    (void)sig;
    running = 0;
}

// 今覚えている ble_pid が本当に生きているか確認
int is_ble_running(void) {
    if (ble_pid <= 0) return 0;
    // kill(pid, 0) はシグナルは送らず存在チェックだけ
    if (kill(ble_pid, 0) == 0) {
        return 1;   // 生きている
    } else {
        if (errno == ESRCH) {
            return 0;   // プロセスはいない
        }
        // その他のエラー（権限など）は、とりあえず「動いてない扱い」にする
        return 0;
    }
}

// ble_dual を起動（すでに動いていたら起動しない）
void start_ble_dual(void) {
    if (is_ble_running()) {
        printf("[INFO] ble_dual はまだ実行中です。新しく起動しません。\n");
        fflush(stdout);
        return;
    }

    printf("[INFO] ble_dual を起動します...\n");
    fflush(stdout);

    pid_t pid = fork();
    if (pid < 0) {
        perror("[ERROR] fork 失敗");
        return;
    }

    if (pid == 0) {
        // 子プロセス側: ble_dual を exec する
        execl(BLE_DUAL_PATH, BLE_DUAL_PATH, (char *)NULL);

        // ここに来たら exec 失敗
        perror("[ERROR] execl 失敗");
        _exit(1);
    } else {
        // 親プロセス側
        ble_pid = pid;
        printf("[INFO] ble_dual 起動完了 (PID=%d)\n", ble_pid);
        fflush(stdout);
    }
}

// ble_dual を停止（優しく SIGTERM → だめなら SIGKILL）
void stop_ble_dual(void) {
    if (!is_ble_running()) {
        printf("[INFO] ble_dual は起動していません。\n");
        fflush(stdout);
        ble_pid = -1;
        return;
    }

    printf("[INFO] 長押し検出: ble_dual を停止します...\n");
    fflush(stdout);

    // まず SIGTERM で終了要求
    if (kill(ble_pid, SIGTERM) < 0) {
        perror("[ERROR] kill(SIGTERM) 失敗");
    }

    // 最大5秒だけ待ってみる
    int status;
    int waited = 0;
    while (waited < 50) {  // 50 * 100ms = 5秒
        pid_t ret = waitpid(ble_pid, &status, WNOHANG);
        if (ret == ble_pid) {
            printf("[INFO] ble_dual 正常終了.\n");
            fflush(stdout);
            ble_pid = -1;
            return;
        } else if (ret == 0) {
            // まだ生きている
            delay(100);   // 100ms
            waited++;
        } else {
            // waitpid エラー
            perror("[ERROR] waitpid 失敗");
            break;
        }
    }

    // ここまで来たらまだ生きている → SIGKILL
    printf("[WARN] ble_dual が終了しないため SIGKILL します...\n");
    fflush(stdout);

    if (kill(ble_pid, SIGKILL) < 0) {
        perror("[ERROR] kill(SIGKILL) 失敗");
    } else {
        waitpid(ble_pid, &status, 0);
        printf("[INFO] ble_dual 強制終了.\n");
        fflush(stdout);
    }
    ble_pid = -1;
}

int main(void) {
    // SIGINT(Ctrl+C) を捕まえて running=0 にする
    signal(SIGINT, handle_sigint);

    // wiringPi を BCM番号モードで初期化
    if (wiringPiSetupGpio() < 0) {
        fprintf(stderr, "[ERROR] wiringPiSetupGpio 失敗\n");
        return 1;
    }

    // ボタン入力設定（プルアップ有効）
    pinMode(BUTTON_PIN, INPUT);
    pullUpDnControl(BUTTON_PIN, PUD_UP);

    int last_state = digitalRead(BUTTON_PIN);  // 初期状態
    unsigned int press_start_ms = 0;
    int pressed = 0;

    printf("[INFO] ボタン監視を開始します。Ctrl+C で終了。\n");
    fflush(stdout);

    while (running) {
        // ble_dual が勝手に終了していた場合に備えて定期的に状態を更新
        if (!is_ble_running()) {
            ble_pid = -1;
        }

        int state = digitalRead(BUTTON_PIN);

        // 状態が変化したときだけ処理（簡易デバウンス付き）
        if (state != last_state) {
            delay(20);  // チャタリング対策 20ms
            state = digitalRead(BUTTON_PIN);
            if (state != last_state) {
                if (state == LOW) {
                    // ボタンが押された瞬間
                    pressed = 1;
                    press_start_ms = millis();
                    // printf("[DEBUG] ボタン押下\n");
                } else {
                    // ボタンが離された瞬間
                    if (pressed) {
                        unsigned int duration = millis() - press_start_ms;
                        printf("[INFO] ボタン押下時間: %u ms\n", duration);
                        fflush(stdout);

                        if (duration >= HOLD_TIME_MS) {
                            // 長押し → 停止
                            stop_ble_dual();
                        } else {
                            // 短押し → 起動（あるいは「まだ実行中」）
                            start_ble_dual();
                        }
                        pressed = 0;
                    }
                    // printf("[DEBUG] ボタン解放\n");
                }
                last_state = state;
            }
        }

        delay(10);  // ポーリング間隔 10ms
    }

    printf("\n[INFO] 終了処理中...\n");
    // 終了時に ble_dual が生きていれば止めてもいいし、そのままでもOK
    // 必要ならコメントアウト解除：
    // if (is_ble_running()) {
    //     stop_ble_dual();
    // }

    return 0;
}
