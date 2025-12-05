#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>

int run_cmd(const char *cmd) {
    int status = system(cmd);
    if (status == -1) {
        perror("system");
        return -1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return -1;
}

int main(void) {
    printf("=== ble_stop: ble_dual 停止ツール (強化版) ===\n");

    // 現在の ble_dual プロセスを表示（デバッグ用）
    printf("現在の ble_dual プロセス一覧:\n");
    run_cmd("pgrep -af ble_dual || echo \"  (ble_dual は見つかりませんでした)\"");

    // まず ble_dual が動いているか確認
    int running = run_cmd("pgrep -f ble_dual > /dev/null 2>&1");
    if (running != 0) {
        printf("→ ble_dual は実行されていません。（停止する対象なし）\n");
    } else {
        printf("→ ble_dual を pkill -f で停止します...\n");
        int kill_code = run_cmd("pkill -f ble_dual");

        if (kill_code == 0) {
            printf("→ pkill は正常終了しました。\n");
        } else {
            printf("→ pkill の戻り値: %d（既に終了しているか、うまく kill できなかった可能性）\n", kill_code);
        }
    }

    // 念のため、アドバタイズを強制停止
    printf("→ 念のため hci0 のアドバタイズを停止します（hciconfig があれば実行）...\n");
    run_cmd("command -v hciconfig > /dev/null 2>&1 && hciconfig hci0 noleadv");

    // 最終確認
    int final = run_cmd("pgrep -f ble_dual > /dev/null 2>&1");
    if (final != 0) {
        printf("→ 最終確認: ble_dual プロセスは存在しません。\n");
    } else {
        printf("→ 最終確認: まだ ble_dual プロセスが存在します。ps aux | grep ble_dual で確認してください。\n");
    }

    printf("=== ble_stop 完了 ===\n");
    return 0;
}


