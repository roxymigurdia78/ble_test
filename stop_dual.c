#include <stdio.h>
#include <stdlib.h>
#include <sys/wait.h>

int run_cmd(const char *cmd) {
    int status = system(cmd);
    if (status == -1) return -1;
    if (WIFEXITED(status)) return WEXITSTATUS(status);
    return -1;
}

int main(void) {
    printf("=== stop_dual: 停止＆消灯ツール ===\n");

    printf("1. systemd サービス停止...\n");
    run_cmd("systemctl stop button_dual.service");

    printf("2. プロセス停止...\n");
    run_cmd("pkill -f button_dual");
    
    // Pythonスクリプトもkillしておく
    run_cmd("pkill -f led_control.py");

    printf("3. Bluetooth停止...\n");
    run_cmd("hciconfig hci0 noleadv");

    // ★重要: 最後にライトを消すためにPythonを呼ぶ
    printf("4. ライトを消灯(off)...\n");
    run_cmd("sudo python3 /home/ras4/ble_test/led_control.py off");

    printf("✅ 完了\n");
    return 0;
}
