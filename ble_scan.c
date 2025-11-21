#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
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

    // HCI フィルタ設定
    struct hci_filter old_filter;
    socklen_t old_filter_len = sizeof(old_filter);
    getsockopt(sock, SOL_HCI, HCI_FILTER, &old_filter, &old_filter_len);

    struct hci_filter filter;
    hci_filter_clear(&filter);
    hci_filter_set_ptype(HCI_EVENT_PKT, &filter);
    hci_filter_set_event(EVT_LE_META_EVENT, &filter);
    setsockopt(sock, SOL_HCI, HCI_FILTER, &filter, sizeof(filter));

    // スキャンパラメータ設定
    le_set_scan_parameters_cp scan_params;
    memset(&scan_params, 0, sizeof(scan_params));
    scan_params.type = 0x01; // Active scan
    scan_params.interval = htobs(0x10);
    scan_params.window   = htobs(0x10);
    scan_params.own_bdaddr_type = 0x00;
    scan_params.filter = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS,
                     sizeof(scan_params), &scan_params) < 0) {
        perror("Set scan parameters failed");
        exit(1);
    }

    // スキャン開始
    uint8_t scan_enable[2] = {0x01, 0x00}; // enable, filter_duplicates
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(scan_enable), scan_enable) < 0) {
        perror("Enable scan failed");
        exit(1);
    }

    printf("Scanning for 'Hello BLE' devices...\n");

    while (1) {
        unsigned char buf[HCI_MAX_EVENT_SIZE];
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) continue;

        evt_le_meta_event *meta = (evt_le_meta_event *)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t num_reports = meta->data[0];
        void *offset = meta->data + 1;

        for (int i = 0; i < num_reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
            char addr[18];
            ba2str(&info->bdaddr, addr);

            // アドバタイズデータからデバイス名を取得
            char name[31] = {0};
            for (int j = 0; j < info->length;) {
                uint8_t field_len = info->data[j];
                if (field_len == 0) break;
                uint8_t field_type = info->data[j + 1];
                if (field_type == 0x09) { // Complete Local Name
                    int copy_len = field_len - 1;
                    if(copy_len > 30) copy_len = 30;
                    memcpy(name, &info->data[j + 2], copy_len);
                    name[copy_len] = '\0';
                    break;
                }
                j += field_len + 1;
            }

            // 'Hello BLE' のみ表示
            if (strcmp(name, "Hello BLE") == 0) {
                int8_t rssi = *(offset + sizeof(le_advertising_info) + info->length);
                uint8_t key = info->data[info->length - 1];
                printf("Device %s | RSSI: %d | Key: %d\n", addr, rssi, key);
            }

            // 次のレポートに移動
            offset += sizeof(le_advertising_info) + info->length + 1; 
        }
    }

    // スキャン停止・終了処理
    scan_enable[0] = 0x00;
    hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(scan_enable), scan_enable);
    setsockopt(sock, SOL_HCI, HCI_FILTER, &old_filter, sizeof(old_filter));
    close(sock);

    return 0;
}
