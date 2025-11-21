// parent.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <errno.h>   // errno / EINTR 用

#define PARENT_ADDR_ARG argv[1]
#define LOG_FILE_PATH   "parent_reception_log.txt"
#define MAX_DATA_LEN    64

// ==========================================================
// ログファイルに書き込みを行う関数
// ==========================================================
void write_log(const char *child_addr, const char *surface) {
    FILE *fp = fopen(LOG_FILE_PATH, "a");
    if (fp == NULL) {
        perror("ログファイルを開けませんでした");
        return;
    }
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char time_str[26];
    strftime(time_str, 26, "%Y-%m-%d %H:%M:%S", tm_info);

    // 例: [2025-11-21 18:00:00] [DC:A6:32:9A:77:48] DETECTED: FRONT
    fprintf(fp, "[%s] [%s] DETECTED: %s\n",
            time_str, child_addr, surface);
    fclose(fp);
}

// ==========================================================
// 受信データ文字列を解析し、ログに記録する関数
// (child 側では "<子機MAC>,<面>" を想定)
// ==========================================================
void process_received_data(const char *received_data) {
    char data_copy[MAX_DATA_LEN];
    strncpy(data_copy, received_data, MAX_DATA_LEN);
    data_copy[MAX_DATA_LEN - 1] = '\0';
    
    char *child_addr = strtok(data_copy, ",");
    char *surface    = strtok(NULL, ",");

    if (child_addr && surface) {
        // ★ ble_dual の Parent-report 風に 1 行だけ表示
        //
        // 例:
        //   Surface-report from DC:A6:32:9A:77:48 => FRONT
        printf("Surface-report from %s => %s\n", child_addr, surface);

        // ログファイルにも保存
        write_log(child_addr, surface);
    } else {
        fprintf(stderr,
                "[ERROR] 受信データの形式が不正です: %s\n",
                received_data ? received_data : "(null)");
    }
}

// ==========================================================
// BLE 受信スレッド
//   ・LEスキャンでアドバタイズを受信
//   ・Local Name 内の "SURFACE:XXXX" をパース
//   ・子機MACと面名を process_received_data() に渡す
// ==========================================================
void *BLE_receive_data_server(void *arg) {
    (void)arg;

    printf("[COMM] BLE親機サーバーを起動しました。子機からの面情報を待機します。\n");
    printf("[INFO] ログファイル: %s\n", LOG_FILE_PATH);

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[BLE] hci_get_route に失敗しました");
        return NULL;
    }

    int sock = hci_open_dev(dev_id);
    if (sock < 0) {
        perror("[BLE] hci_open_dev に失敗しました");
        return NULL;
    }

    // HCIフィルタ設定：LE Meta Eventのみ
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        perror("[BLE] HCIフィルタ設定に失敗しました");
        close(sock);
        return NULL;
    }

    // スキャンパラメータ設定
    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type            = 0x01; // Active scan
    scan_params_cp.interval        = htobs(0x0010);
    scan_params_cp.window          = htobs(0x0010);
    scan_params_cp.own_bdaddr_type = 0x00; // Public
    scan_params_cp.filter          = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS,
                     sizeof(scan_params_cp), &scan_params_cp) < 0) {
        perror("[BLE] スキャンパラメータ設定に失敗しました");
        close(sock);
        return NULL;
    }

    // スキャン開始
    uint8_t enable     = 0x01;
    uint8_t filter_dup = 0x00;
    uint8_t cmd[2]     = { enable, filter_dup };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("[BLE] スキャン有効化に失敗しました");
        close(sock);
        return NULL;
    }

    printf("[COMM] BLEスキャンを開始しました。子機からの SURFACE 広告を待っています...\n");

    unsigned char buf[HCI_MAX_EVENT_SIZE];

    while (1) {
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) {
                // シグナル割り込みの場合はリトライ
                continue;
            }
            perror("[BLE] read でエラーが発生しました");
            break;
        }
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;

        uint8_t *ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)ptr;
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;

            char addr[18];
            ba2str(&info->bdaddr, addr);   // 子機のMACアドレス

            char name[128] = "";
            int pos = 0;

            // アドバタイズデータから Local Name を取り出す
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
                if (field_len == 0) break;
                if (pos + field_len >= info->length) break;

                uint8_t field_type = info->data[pos + 1];

                // Local Name (Complete:0x09, Shortened:0x08)
                if (field_type == 0x09 || field_type == 0x08) {
                    int name_len = field_len - 1;
                    if (name_len > (int)sizeof(name) - 1)
                        name_len = (int)sizeof(name) - 1;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                }

                pos += field_len + 1;
            }

            if (name[0] != '\0') {
                // child 側の形式: "CubeNode|SURFACE:FRONT"
                char *p = strstr(name, "SURFACE:");
                if (p) {
                    p += strlen("SURFACE:");
                    char surface[32];
                    int si = 0;
                    // '|' か終端までを面名として抜き出す
                    while (*p != '\0' && *p != '|' && si < (int)sizeof(surface)-1) {
                        surface[si++] = *p++;
                    }
                    surface[si] = '\0';

                    // "<MAC>,<SURFACE>" 形式にして process_received_data へ
                    char combined[MAX_DATA_LEN];
                    snprintf(combined, sizeof(combined), "%s,%s", addr, surface);

                    // ★ ここでは余計なデバッグ表示はしない
                    //    直接 1行ログの関数へ渡す
                    process_received_data(combined);
                }
            }

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    // スキャン停止
    enable = 0x00;
    cmd[0] = enable;
    cmd[1] = 0x00;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("[BLE] スキャン停止に失敗しました");
    }

    close(sock);
    return NULL;
}

/**
 * 親機として実行されるメインプログラム。
 */
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <parent_full_address>\n", argv[0]);
        return 1;
    }
    const char *parent_addr = PARENT_ADDR_ARG;

    printf("\n==================================\n");
    printf("👑 親機プログラム開始 (PARENT PROGRAM STARTING)\n");
    printf("==================================\n");
    printf("確定した親機アドレス: %s\n", parent_addr);
    printf("役割: 子機から送られてくる面情報を受信し、ログに記録します。\n");

    pthread_t rx_thread;
    if (pthread_create(&rx_thread, NULL,
                       BLE_receive_data_server, NULL) != 0) {
        perror("BLE受信スレッドの生成に失敗しました");
        return 1;
    }

    printf("親機メイン処理を実行中...\n");
    
    pthread_join(rx_thread, NULL);

    return 0;
}
