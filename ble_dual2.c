// ================================================
// BLE Dual (Advertise + Scan) 自律キー版 — 修正版（最終親アドレスをフル表示する）
// 変更点:
// 1) 親アドレスは下三オクテットのみ advertise する（例: 9A:77:A1）
// 2) 検出・親決定は下三オクテットで行うが、最終確認完了時の出力はフルアドレスを表示する
// 3) perceived_parent の "last3" と "full" を別々に管理して整合性を取る
// ================================================

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define ADV_INTERVAL_MS 500
#define SCAN_INTERVAL_MS 200
#define SCAN_WINDOW_MS 100
#define MAX_DEVS 100
#define SCAN_DURATION_SEC 20        // 初期スキャン時間（秒）
#define PARENT_EXCHANGE_SEC 20      // 親アドレス交換フェーズ時間（秒）
#define RETRY_DELAY_SEC 5           // 再試行前の待機（秒）
#define MAX_ATTEMPTS 2              // 初回＋再試行 = 2回

int dev_id;
int sock;
int my_key;
int round_count = 0;
char device_name[32] = "CubeNode";
char my_addr[18] = "(unknown)"; // フルアドレス

typedef struct {
    char addr[18];
    int key;
} DeviceKey;

DeviceKey detected_devices[MAX_DEVS];
int detected_count = 0;
pthread_mutex_t detected_mutex = PTHREAD_MUTEX_INITIALIZER;

// 親交換フェーズ関連
int parent_phase = 0; // 0: 通常、1: 親交換フェーズ中（advertise に P: を付加）
char perceived_parent_addr_last3[18] = "(unknown)"; // 下三オクテット形式
char perceived_parent_addr_full[18] = "(unknown)";  // フルアドレス形式（最終出力用）
pthread_mutex_t parent_mutex = PTHREAD_MUTEX_INITIALIZER;

// 最終結果フラグ（scan_thread が設定）
int final_success = 0;
char final_parent_addr_last3[18] = "(none)"; // 下三オクテット形式
char final_parent_addr_full[32] = "(none)";  // フルアドレス形式

// ==================================================
// 🔑 アドレスと時刻から固有キーを生成
// ==================================================
int generate_unique_key(bdaddr_t *addr) {
    unsigned int seed = (unsigned int)time(NULL);
    for (int i = 0; i < 6; i++) {
        seed ^= ((unsigned int)addr->b[i] << (i * 4));
    }
    srand(seed);
    return rand() % 256;
}

// =========================
// 下三オクテットを抽出: フルアドレス("AA:BB:CC:DD:EE:FF") -> "DD:EE:FF"
// ※ ba2str の出力順に合わせる
// =========================
void get_last_three(const char *full_addr, char *out, size_t out_sz) {
    out[0] = '\0';
    if (!full_addr || strlen(full_addr) < 8) return;
    const char *r = full_addr + strlen(full_addr) - 1;
    int cnt = 0;
    int found = 0;
    while (r > full_addr) {
        if (*r == ':') {
            cnt++;
            if (cnt == 2) { // 末尾から2個目のコロンの次が3オクテットの先頭
                r++;
                found = 1;
                break;
            }
        }
        r--;
    }
    if (!found) {
        size_t len = strlen(full_addr);
        if (len >= 8) {
            const char *s = full_addr + len - 8;
            strncpy(out, s, out_sz - 1);
            out[out_sz - 1] = '\0';
        }
        return;
    }
    strncpy(out, r, out_sz - 1);
    out[out_sz - 1] = '\0';
}

// =========================
// advertise helper: P: タグ抽出用関数（下三オクテットを取り出す）
// 例: name に "CubeNode|12|3|P:9A:77:A1" が含まれていれば out_parent に "9A:77:A1" を返す
// =========================
void extract_parent_tag_from_name(const char *name, char *out_parent, size_t out_sz) {
    out_parent[0] = '\0';
    const char *p = strstr(name, "|P:");
    if (!p) return;
    p += 3;
    size_t i = 0;
    while (*p != '\0' && *p != '|' && i + 1 < out_sz) {
        out_parent[i++] = *p++;
    }
    out_parent[i] = '\0';
}

// =========================
// 広告スレッド
//  親交換フェーズ中は name に "|P:<addr_last3>" を追加して送信する
// =========================
void *advertise_thread(void *arg) {
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    adv_params_cp.min_interval = htobs((uint16_t)(ADV_INTERVAL_MS * 1.6));
    adv_params_cp.max_interval = htobs((uint16_t)(ADV_INTERVAL_MS * 1.6));
    adv_params_cp.advtype = 0x00;
    adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07;
    adv_params_cp.filter = 0x00;

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("Failed to set advertising parameters");
        pthread_exit(NULL);
    }

    uint8_t adv_data[31];
    struct {
        uint8_t length;
        uint8_t data[31];
    } __attribute__((packed)) adv_data_cp_struct;

    static int first_print = 1;
    while (1) {
        int len = 0;
        memset(adv_data, 0, sizeof(adv_data));

        // Flags (3 bytes)
        adv_data[len++] = 2;
        adv_data[len++] = 0x01;
        adv_data[len++] = 0x06;

        // デバイス名 + key + connect情報 (+ optional parent info)
        char name_field[64];
        pthread_mutex_lock(&detected_mutex);
        int connect_count = detected_count;
        pthread_mutex_unlock(&detected_mutex);

        pthread_mutex_lock(&parent_mutex);
        int pphase = parent_phase;
        char paddr_copy_last3[18];
        strncpy(paddr_copy_last3, perceived_parent_addr_last3, sizeof(paddr_copy_last3));
        paddr_copy_last3[sizeof(paddr_copy_last3)-1] = '\0';
        pthread_mutex_unlock(&parent_mutex);

        if (pphase == 0) {
            snprintf(name_field, sizeof(name_field), "%s|%d|%d", device_name, my_key, connect_count);
        } else {
            // 親交換フェーズ：自分が認識している親アドレス（下三オクテット）を含めて送る
            snprintf(name_field, sizeof(name_field), "%s|%d|%d|P:%s", device_name, my_key, connect_count, paddr_copy_last3);
        }

        int name_len = (int)strlen(name_field);
        int remaining = (int)sizeof(adv_data) - len - 2;
        if (remaining < 0) remaining = 0;
        if (name_len > remaining) name_len = remaining;

        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09;
        if (name_len > 0) {
            memcpy(&adv_data[len], name_field, name_len);
            len += name_len;
        }

        adv_data_cp_struct.length = (uint8_t)len;
        memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
        memcpy(adv_data_cp_struct.data, adv_data, len);

        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                         len + 1, &adv_data_cp_struct) < 0) {
            perror("Failed to set advertising data");
        }

        uint8_t enable = 0x01;
        if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                         1, &enable) < 0) {
            perror("Failed to enable advertising");
            pthread_exit(NULL);
        }

        if (first_print) {
            printf("Advertising start: name=%s key=%d\n", device_name, my_key);
            first_print = 0;
        }

        round_count++;
        usleep(ADV_INTERVAL_MS * 1000);
    }
    return NULL;
}

// =========================
// スキャンスレッド
//  - 初期スキャンで隣接デバイスを検出 (detected_devices に格納)
//  - 最小キーを元に親を決定（perceived_parent に格納, 下三オクテット）
//  - 親交換フェーズ：parent_phase=1 にして P:<addr_last3> を advertise させつつ、他機の P: 報告を収集（PARENT_EXCHANGE_SEC）
//  - 検出した総台数（自分含む）を基準に一致判定（自動台数検出）
//  - 一致なら final_success=1, final_parent_addr 設定。そうでなければ final_success=0
// =========================
void *scan_thread(void *arg) {
    // HCI filter: LE meta events
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        perror("Failed to set HCI filter");
    }

    // set scan parameters
    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01;
    scan_params_cp.interval = htobs((uint16_t)(SCAN_INTERVAL_MS * 1.6));
    scan_params_cp.window = htobs((uint16_t)(SCAN_WINDOW_MS * 1.6));
    scan_params_cp.own_bdaddr_type = 0x00;
    scan_params_cp.filter = 0x00;
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS,
                     sizeof(scan_params_cp), &scan_params_cp) < 0) {
        perror("Set scan parameters failed");
        pthread_exit(NULL);
    }

    uint8_t enable = 0x01;
    uint8_t filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("Enable scan failed");
        pthread_exit(NULL);
    }

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    time_t start_time = time(NULL);

    // --- 初期スキャン（周囲の把握） ---
    while (difftime(time(NULL), start_time) < SCAN_DURATION_SEC) {
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            perror("Read error");
            break;
        }

        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
        uint8_t *data = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)(data);
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
            char addr[18];
            ba2str(&info->bdaddr, addr);

            char name[128] = "";
            int key = -1;
            int connect_count = -1;

            int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
                if (field_len == 0) break;
                if (pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];

                if (field_type == 0x09) { // Complete Local Name
                    int name_len = field_len - 1;
                    if (name_len > (int)sizeof(name) - 1) name_len = (int)sizeof(name) - 1;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                    // parse name like "%s|%d|%d"  (may have extra |P:...)
                    sscanf(name, "%*[^|]|%d|%d", &key, &connect_count);
                }
                pos += field_len + 1;
            }

            // --- key==-1 の場合はスキップ ---
            if (key < 0) {
                offset = (uint8_t *)info + sizeof(*info) + info->length;
                continue;
            }

            pthread_mutex_lock(&detected_mutex);
            int exists = 0;
            for (int j = 0; j < detected_count; j++) {
                if (strcmp(detected_devices[j].addr, addr) == 0) {
                    detected_devices[j].key = key;
                    exists = 1;
                    break;
                }
            }
            if (!exists && detected_count < MAX_DEVS) {
                strncpy(detected_devices[detected_count].addr, addr, sizeof(detected_devices[detected_count].addr));
                detected_devices[detected_count].addr[sizeof(detected_devices[detected_count].addr)-1] = '\0';
                detected_devices[detected_count].key = key;
                detected_count++;
            }
            pthread_mutex_unlock(&detected_mutex);

            printf("Scan found: %s, Key: %d, connect: %d\n",
                   addr, key, connect_count >= 0 ? connect_count : 0);

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    // disable scan temporarily
    uint8_t disable_cmd[2] = { 0x00, 0x00 };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd) < 0) {
        perror("Disable scan failed");
    }

    // --- 初期スキャン終了。現在の detected_devices から親を選定 ---
    pthread_mutex_lock(&detected_mutex);
    printf("\n--- BLE Scan Summary ---\n");
    printf("My key: %d\n", my_key);
    printf("Detected %d devices.\n", detected_count);
    pthread_mutex_unlock(&detected_mutex);

    // Determine smallest key (including self)
    int smallest_key = my_key;
    char smallest_addr_full[18] = "(self)";

    pthread_mutex_lock(&detected_mutex);
    for (int i = 0; i < detected_count; i++) {
        if (detected_devices[i].key < smallest_key) {
            smallest_key = detected_devices[i].key;
            strncpy(smallest_addr_full, detected_devices[i].addr, sizeof(smallest_addr_full));
            smallest_addr_full[sizeof(smallest_addr_full)-1] = '\0';
        }
    }
    pthread_mutex_unlock(&detected_mutex);

    // smallest_addr_full が "(self)" の場合は my_addr を使う
    if (strcmp(smallest_addr_full, "(self)") == 0) {
        strncpy(smallest_addr_full, my_addr, sizeof(smallest_addr_full));
        smallest_addr_full[sizeof(smallest_addr_full)-1] = '\0';
    }

    char smallest_addr_last3[18] = "";
    get_last_three(smallest_addr_full, smallest_addr_last3, sizeof(smallest_addr_last3));

    if (smallest_key == my_key)
        printf("✅ I am the PARENT (key=%d) — addr_last3=%s\n", my_key, smallest_addr_last3);
    else
        printf("🔹 Parent is %s (key=%d) — addr_last3=%s\n", smallest_addr_full, smallest_key, smallest_addr_last3);

    // set perceived_parent based on computed smallest_addr_last3 and full
    pthread_mutex_lock(&parent_mutex);
    strncpy(perceived_parent_addr_last3, smallest_addr_last3, sizeof(perceived_parent_addr_last3));
    perceived_parent_addr_last3[sizeof(perceived_parent_addr_last3)-1] = '\0';
    strncpy(perceived_parent_addr_full, smallest_addr_full, sizeof(perceived_parent_addr_full));
    perceived_parent_addr_full[sizeof(perceived_parent_addr_full)-1] = '\0';
    parent_phase = 1; // 親交換フェーズ中 → advertise_thread は P:<addr_last3> を送る
    pthread_mutex_unlock(&parent_mutex);

    // --- 親交換フェーズ開始: 他機が報告する親アドレスを収集 ---
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("Enable scan for parent exchange failed");
    }

    // storage for reported parents during this phase; map addr -> reported_parent
    typedef struct {
        char addr[18];
        char reported_parent[18];
    } VerEntry;
    VerEntry ver_list[MAX_DEVS];
    int ver_count = 0;

    time_t ver_start = time(NULL);
    while (difftime(time(NULL), ver_start) < PARENT_EXCHANGE_SEC) {
        int len = read(sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR) continue;
            perror("Read error (parent exchange)");
            break;
        }
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
        uint8_t *data = buf + (1 + HCI_EVENT_HDR_SIZE);
        evt_le_meta_event *meta = (evt_le_meta_event *)(data);
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
            char addr[18];
            ba2str(&info->bdaddr, addr);

            char name[128] = "";
            int pos = 0;
            while (pos < info->length) {
                uint8_t field_len = info->data[pos];
                if (field_len == 0) break;
                if (pos + field_len >= info->length) break;
                uint8_t field_type = info->data[pos + 1];

                if (field_type == 0x09) { // Complete Local Name
                    int name_len = field_len - 1;
                    if (name_len > (int)sizeof(name) - 1) name_len = (int)sizeof(name) - 1;
                    memcpy(name, &info->data[pos + 2], name_len);
                    name[name_len] = '\0';
                }
                pos += field_len + 1;
            }

            // extract P: tag if present (下三オクテット形式を想定)
            char reported_parent[18] = "";
            extract_parent_tag_from_name(name, reported_parent, sizeof(reported_parent));
            if (reported_parent[0] == '\0') {
                offset = (uint8_t *)info + sizeof(*info) + info->length;
                continue;
            }

            // store (addr -> reported_parent) into ver_list (unique by addr)
            int found = 0;
            for (int k = 0; k < ver_count; k++) {
                if (strcmp(ver_list[k].addr, addr) == 0) {
                    strncpy(ver_list[k].reported_parent, reported_parent, sizeof(ver_list[k].reported_parent));
                    found = 1;
                    break;
                }
            }
            if (!found && ver_count < MAX_DEVS) {
                strncpy(ver_list[ver_count].addr, addr, sizeof(ver_list[ver_count].addr));
                ver_list[ver_count].addr[sizeof(ver_list[ver_count].addr)-1] = '\0';
                strncpy(ver_list[ver_count].reported_parent, reported_parent, sizeof(ver_list[ver_count].reported_parent));
                ver_list[ver_count].reported_parent[sizeof(ver_list[ver_count].reported_parent)-1] = '\0';
                ver_count++;
            }

            printf("Parent-report from %s => %s\n", addr, reported_parent);

            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    // disable scan after verification phase
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd) < 0) {
        perror("Disable scan (after parent exchange) failed");
    }

    // --- 親交換フェーズの集計と判定 ---
    pthread_mutex_lock(&detected_mutex);
    int auto_total = detected_count + 1; // self included
    pthread_mutex_unlock(&detected_mutex);
    printf("\nParent exchange: received reports from %d other devices. (expected %d other devices)\n", ver_count, auto_total - 1);

    int all_match = 1;
    if (ver_count < (auto_total - 1)) {
        printf("Not all devices reported parent (received %d of %d).\n", ver_count, auto_total - 1);
        all_match = 0;
    } else {
        // 全てが報告していれば、その reported_parent が perceived_parent_addr_last3 と一致するかチェック
        for (int i = 0; i < ver_count; i++) {
            if (strcmp(ver_list[i].reported_parent, perceived_parent_addr_last3) != 0) {
                printf("Mismatch: device %s reported %s (expected %s)\n",
                       ver_list[i].addr, ver_list[i].reported_parent, perceived_parent_addr_last3);
                all_match = 0;
            }
        }
    }

    if (all_match) {
        pthread_mutex_lock(&parent_mutex);
        strncpy(final_parent_addr_last3, perceived_parent_addr_last3, sizeof(final_parent_addr_last3));
        final_parent_addr_last3[sizeof(final_parent_addr_last3)-1] = '\0';
        strncpy(final_parent_addr_full, perceived_parent_addr_full, sizeof(final_parent_addr_full));
        final_parent_addr_full[sizeof(final_parent_addr_full)-1] = '\0';
        pthread_mutex_unlock(&parent_mutex);
        final_success = 1;
        printf("\n🎉 最終確認完了 — 親機は %s (全員一致)\n", final_parent_addr_full);
    } else {
        final_success = 0;
        printf("\n⚠️ 親確認失敗 — 全員一致しませんでした\n");
    }

    // 親交換フェーズ終了後は advertise に親情報送信をやめる
    pthread_mutex_lock(&parent_mutex);
    parent_phase = 0;
    pthread_mutex_unlock(&parent_mutex);

    return NULL;
}

// =========================
// メイン関数（試行回数管理付き）
// =========================
int main(int argc, char *argv[]) {
    dev_id = hci_get_route(NULL);
    sock = hci_open_dev(dev_id);
    if (dev_id < 0 || sock < 0) {
        perror("Opening socket");
        return 1;
    }

    bdaddr_t bdaddr;
    if (hci_read_bd_addr(sock, &bdaddr, 1000) < 0) {
        perror("Warning: hci_read_bd_addr failed");
        memset(&bdaddr, 0, sizeof(bdaddr));
    }
    ba2str(&bdaddr, my_addr);

    int attempt = 0;
    while (attempt < MAX_ATTEMPTS) {
        // リセット
        pthread_mutex_lock(&detected_mutex);
        detected_count = 0;
        memset(detected_devices, 0, sizeof(detected_devices));
        pthread_mutex_unlock(&detected_mutex);

        pthread_mutex_lock(&parent_mutex);
        parent_phase = 0;
        strncpy(perceived_parent_addr_last3, "(unknown)", sizeof(perceived_parent_addr_last3));
        strncpy(perceived_parent_addr_full, "(unknown)", sizeof(perceived_parent_addr_full));
        pthread_mutex_unlock(&parent_mutex);

        // 固有キーを生成（毎回生成）
        my_key = generate_unique_key(&bdaddr);

        // ランダム遅延（0〜1秒）
        srand((unsigned int)time(NULL) ^ my_key);
        int delay_ms = rand() % 1000;
        printf("Random startup delay: %d ms\n", delay_ms);
        usleep(delay_ms * 1000);

        pthread_t t1, t2;
        if (pthread_create(&t1, NULL, advertise_thread, NULL) != 0) {
            perror("Failed to create advertise thread");
            close(sock);
            return 1;
        }
        if (pthread_create(&t2, NULL, scan_thread, NULL) != 0) {
            perror("Failed to create scan thread");
            pthread_cancel(t1);
            close(sock);
            return 1;
        }

        // scan_thread の終了を待つ（その中で親交換フェーズまで行う）
        pthread_join(t2, NULL);

        // scan が終わった後の結果確認
        if (final_success) {
            printf("Parent confirmed: %s\n", final_parent_addr_full);
            // advertise を止めて終了
            pthread_cancel(t1);
            uint8_t adv_disable = 0x00;
            if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &adv_disable) < 0) {
                // ignore
            }
            break;
        } else {
            // 失敗時の処理
            attempt++;
            pthread_cancel(t1);
            uint8_t adv_disable = 0x00;
            if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &adv_disable) < 0) {
                // ignore
            }

            if (attempt >= MAX_ATTEMPTS) {
                // 二回目の試行が終わったら強制終了（仕様）
                printf("Attempts exhausted (performed %d attempts). Ending regardless of mismatch.\n", attempt);
                break;
            } else {
                // 待機して再試行
                printf("Parent mismatch detected. Waiting %d seconds and retrying (attempt %d of %d)...\n", RETRY_DELAY_SEC, attempt+1, MAX_ATTEMPTS);
                sleep(RETRY_DELAY_SEC);
                // final_success をクリアして次ループへ
                final_success = 0;
                strncpy(final_parent_addr_last3, "(none)", sizeof(final_parent_addr_last3));
                strncpy(final_parent_addr_full, "(none)", sizeof(final_parent_addr_full));
            }
        }
    } // attempts loop

    close(sock);
    return 0;
}
