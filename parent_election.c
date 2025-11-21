#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define MAX_PEERS 12

// 受信した周囲デバイス情報
struct peer_info {
    bdaddr_t addr;
    uint8_t key;
    int round;
};

int main() {
    srand(time(NULL));

    // 1️⃣ 自分のBD_ADDR取得
    int dev_id = hci_get_route(NULL);
    int sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0) {
        perror("HCI device open failed");
        return 1;
    }

    bdaddr_t self_addr;
    char self_addr_str[18];
    struct hci_dev_info di;
    if (hci_devinfo(dev_id, &di) < 0) {
        perror("Cannot get dev info");
        return 1;
    }
    self_addr = di.bdaddr;
    ba2str(&self_addr, self_addr_str);

    // 2️⃣ 乱数キー生成とランダム遅延
    int key = rand() % 100;
    printf("🔑 Self key: %d\n", key);
    usleep(rand() % 1000000); // 0〜1秒ランダム遅延

    // 3️⃣ アドバタイズパラメータ設定（Connectable, 100ms間隔）
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    adv_params_cp.min_interval = htobs(0x00A0); // 100ms
    adv_params_cp.max_interval = htobs(0x00A0);
    adv_params_cp.advtype = 0x00; // Connectable undirected
    adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07;
    adv_params_cp.filter = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("Failed to set advertising parameters");
    }

    printf("📡 Advertising self info (addr + key + round=0)...\n");

    // 4️⃣ スキャン設定（アクティブ、ノンブロッキング）
    if (hci_le_set_scan_parameters(sock, 0x01, htobs(0x0010), htobs(0x0010), 0x00, 0x00, 1000) < 0) {
        perror("Set scan parameters failed");
    }

    uint8_t scan_enable = 0x01; // スキャン開始
    if (hci_le_set_scan_enable(sock, scan_enable, 0x00, 1000) < 0) {
        perror("Enable scan failed");
    }

    // ノンブロッキングに設定
    fcntl(sock, F_SETFL, O_NONBLOCK);

    struct peer_info peers[MAX_PEERS];
    int num_peers = 0;

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    printf("🔍 Scanning for peers for 10 seconds...\n");
    time_t start = time(NULL);
    while (time(NULL) - start < 10) {  // 10秒間スキャン
        int len = read(sock, buf, sizeof(buf));
        if (len > 0) {
            evt_le_meta_event *meta = (evt_le_meta_event *)(buf + (1 + HCI_EVENT_HDR_SIZE));
            if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

            le_advertising_info *info = (le_advertising_info *)(meta->data + 1);
            char addr[18];
            ba2str(&info->bdaddr, addr);

            // 自分を除外
            if (bacmp(&info->bdaddr, &self_addr) == 0) continue;

            // peerリストに追加（重複なし）
            int exists = 0;
            for (int i = 0; i < num_peers; i++) {
                if (bacmp(&peers[i].addr, &info->bdaddr) == 0) {
                    exists = 1;
                    break;
                }
            }
            if (!exists && num_peers < MAX_PEERS) {
                peers[num_peers].addr = info->bdaddr;
                peers[num_peers].key = info->data[0]; // 簡易版:キーはdata[0]
                peers[num_peers].round = 0;
                num_peers++;
                printf("📡 Detected peer %s, key=%d\n", addr, peers[num_peers-1].key);
            }
        } else {
            usleep(10000); // 10ms休止
        }
    }

    // スキャン終了
    uint8_t scan_disable = 0x00;
    hci_le_set_scan_enable(sock, scan_disable, 0x00, 1000);

    // 5️⃣ 親候補判定（最小キー）
    int min_key = key;
    bdaddr_t parent_addr = self_addr;
    for (int i = 0; i < num_peers; i++) {
        if (peers[i].key < min_key) {
            min_key = peers[i].key;
            parent_addr = peers[i].addr;
        }
    }

    char parent_addr_str[18];
    ba2str(&parent_addr, parent_addr_str);

    if (bacmp(&parent_addr, &self_addr) == 0) {
        printf("👑 I am the parent!\n");
    } else {
        printf("👶 Parent detected: %s\n", parent_addr_str);
    }

    close(sock);
    return 0;
}
