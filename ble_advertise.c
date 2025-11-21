#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

int main() {
    int dev_id = hci_get_route(NULL);
    int sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0) {
        perror("Opening socket");
        exit(1);
    }

    srand(time(NULL));
    int key = rand() % 100; // ランダムキー生成

    // --- アドバタイズパラメータ設定 ---
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    adv_params_cp.min_interval = htobs(0x0800);
    adv_params_cp.max_interval = htobs(0x0800);
    adv_params_cp.advtype = 0x00; // Connectable undirected
    adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07; // ← 修正版：channel_map → chan_map
    adv_params_cp.filter = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("Failed to set advertising parameters");
        exit(1);
    }

    // --- アドバタイズデータ構築 ---
    uint8_t adv_data[31];
    memset(adv_data, 0, sizeof(adv_data));

    const char *name = "Hello BLE";
    int len = 0;
    adv_data[len++] = 2;      // Length
    adv_data[len++] = 0x01;   // Flags
    adv_data[len++] = 0x06;

    adv_data[len++] = strlen(name) + 2; // Length of name field + type + key
    adv_data[len++] = 0x09;             // Complete local name
    strcpy((char *)&adv_data[len], name);
    len += strlen(name);

    adv_data[len++] = (uint8_t)key; // 末尾にキーを追加
    // --- データ送信 ---
    struct {
        uint8_t length;
        uint8_t data[31];
    } __attribute__((packed)) adv_data_cp;

    adv_data_cp.length = len;
    memcpy(adv_data_cp.data, adv_data, len);

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                     len + 1, &adv_data_cp) < 0) {
        perror("Failed to set advertising data");
        exit(1);
    }

    // --- アドバタイズ開始 ---
    uint8_t enable = 0x01;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                     1, &enable) < 0) {
        perror("Failed to enable advertising");
        exit(1);
    }

    printf("Advertising '%s' with key=%d ...\n", name, key);
    printf("Press Ctrl+C to stop.\n");
    sleep(10);

    enable = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                 1, &enable);
    close(sock);

    return 0;
}
