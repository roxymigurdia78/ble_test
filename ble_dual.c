// ble_dual.c
// ================================================
// BLE Dual (Advertise + Scan) 自律キー版
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
#define SCAN_DURATION_SEC 20        
#define PARENT_EXCHANGE_SEC 20      
#define RETRY_DELAY_SEC 5           
#define MAX_ATTEMPTS 2              

#define KEY_LIST_FILE "parent_key_list.txt"

int dev_id;
int sock;
int my_key;
int round_count = 0;
char device_name[32] = "CubeNode";
char my_addr[18] = "(unknown)"; 

typedef struct {
    char addr[18];
    int key;
} DeviceKey;

DeviceKey detected_devices[MAX_DEVS];
int detected_count = 0;
pthread_mutex_t detected_mutex = PTHREAD_MUTEX_INITIALIZER;

int parent_phase = 0; 
char perceived_parent_addr_last3[18] = "(unknown)"; 
char perceived_parent_addr_full[18] = "(unknown)";  
pthread_mutex_t parent_mutex = PTHREAD_MUTEX_INITIALIZER;

int final_success = 0;
char final_parent_addr_last3[18] = "(none)"; 
char final_parent_addr_full[32] = "(none)";  

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
// 下三オクテットを抽出
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
            if (cnt == 2) {
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
// P: タグ抽出用関数
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

// ==================================================
// 親になったノード用: key & addr を昇順でファイル保存
// 形式: 「key addr」を1行に1組
// ==================================================
void save_key_list(const char *filename) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror("save_key_list: fopen");
        return;
    }

    typedef struct {
        int  key;
        char addr[18];
    } KeyEntry;

    KeyEntry list[MAX_DEVS + 1];
    int count = 0;

    // 自分自身
    strncpy(list[count].addr, my_addr, sizeof(list[count].addr));
    list[count].addr[sizeof(list[count].addr) - 1] = '\0';
    list[count].key = my_key;
    count++;

    // 検出した他ノード
    pthread_mutex_lock(&detected_mutex);
    for (int i = 0; i < detected_count && count < MAX_DEVS + 1; i++) {
        if (strcmp(detected_devices[i].addr, my_addr) == 0) {
            continue; // 自分が混ざっていたらスキップ
        }
        // 重複チェック
        int dup = 0;
        for (int j = 0; j < count; j++) {
            if (strcmp(list[j].addr, detected_devices[i].addr) == 0) {
                dup = 1;
                break;
            }
        }
        if (dup) continue;

        strncpy(list[count].addr, detected_devices[i].addr, sizeof(list[count].addr));
        list[count].addr[sizeof(list[count].addr) - 1] = '\0';
        list[count].key = detected_devices[i].key;
        count++;
    }
    pthread_mutex_unlock(&detected_mutex);

    // key 昇順ソート
    for (int i = 0; i < count - 1; i++) {
        for (int j = i + 1; j < count; j++) {
            if (list[j].key < list[i].key) {
                KeyEntry tmp = list[i];
                list[i] = list[j];
                list[j] = tmp;
            }
        }
    }

    // ファイル出力
    for (int i = 0; i < count; i++) {
        fprintf(fp, "%d %s\n", list[i].key, list[i].addr);
    }

    fclose(fp);
    printf("Saved %d entries to %s\n", count, filename);
}

// =========================
// 広告スレッド
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

        adv_data[len++] = 2;
        adv_data[len++] = 0x01;
        adv_data[len++] = 0x06;

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
// =========================
void *scan_thread(void *arg) {
    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    if (setsockopt(sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)) < 0) {
        perror("Failed to set HCI filter");
    }

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
                    sscanf(name, "%*[^|]|%d|%d", &key, &connect_count);
                }
                pos += field_len + 1;
            }

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

    uint8_t disable_cmd[2] = { 0x00, 0x00 };
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd) < 0) {
        perror("Disable scan failed");
    }

    // --- 親選定 ---
    pthread_mutex_lock(&detected_mutex);
    printf("\n--- BLE Scan Summary ---\n");
    printf("My key: %d\n", my_key);
    printf("Detected %d devices.\n", detected_count);
    pthread_mutex_unlock(&detected_mutex);

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

    pthread_mutex_lock(&parent_mutex);
    strncpy(perceived_parent_addr_last3, smallest_addr_last3, sizeof(perceived_parent_addr_last3));
    perceived_parent_addr_last3[sizeof(perceived_parent_addr_last3)-1] = '\0';
    strncpy(perceived_parent_addr_full, smallest_addr_full, sizeof(perceived_parent_addr_full));
    perceived_parent_addr_full[sizeof(perceived_parent_addr_full)-1] = '\0';
    parent_phase = 1; 
    pthread_mutex_unlock(&parent_mutex);

    // --- 親交換フェーズ開始 ---
    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE,
                     sizeof(cmd), cmd) < 0) {
        perror("Enable scan for parent exchange failed");
    }

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

            char reported_parent[18] = "";
            extract_parent_tag_from_name(name, reported_parent, sizeof(reported_parent));
            if (reported_parent[0] == '\0') {
                offset = (uint8_t *)info + sizeof(*info) + info->length;
                continue;
            }

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

    if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd) < 0) {
        perror("Disable scan (after parent exchange) failed");
    }

    // --- 親交換フェーズの集計と判定 ---
    pthread_mutex_lock(&detected_mutex);
    int auto_total = detected_count + 1; 
    pthread_mutex_unlock(&detected_mutex);
    printf("\nParent exchange: received reports from %d other devices. (expected %d other devices)\n", ver_count, auto_total - 1);

    int all_match = 1;
    if (ver_count < (auto_total - 1)) {
        printf("Not all devices reported parent (received %d of %d).\n", ver_count, auto_total - 1);
        all_match = 0;
    } else {
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
        pthread_mutex_lock(&detected_mutex);
        detected_count = 0;
        memset(detected_devices, 0, sizeof(detected_devices));
        pthread_mutex_unlock(&detected_mutex);

        pthread_mutex_lock(&parent_mutex);
        parent_phase = 0;
        strncpy(perceived_parent_addr_last3, "(unknown)", sizeof(perceived_parent_addr_last3));
        strncpy(perceived_parent_addr_full, "(unknown)", sizeof(perceived_parent_addr_full));
        pthread_mutex_unlock(&parent_mutex);

        my_key = generate_unique_key(&bdaddr);

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

        pthread_join(t2, NULL);

        if (final_success) {
            printf("Parent confirmed: %s\n", final_parent_addr_full);
            pthread_cancel(t1);
            uint8_t adv_disable = 0x00;
            if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &adv_disable) < 0) {
                // ignore
            }

            // ===============================
            // 親/子判定と外部プログラム実行
            // ===============================

            if (strcmp(final_parent_addr_full, my_addr) == 0) {
                printf("I am PARENT — executing parent program.\n");

                // 親になるノードだけ、キーリストを保存して parent に渡す
                save_key_list(KEY_LIST_FILE);

                char cmd[128];
                snprintf(cmd, sizeof(cmd), "./parent %s", final_parent_addr_full);
                printf("[DEBUG] Executing command: %s\n", cmd); 
                system(cmd);   

            } else {
                printf("I am CHILD — executing child program.\n");

                char cmd[128];
                int len = snprintf(cmd, sizeof(cmd), "./child %s %s", my_addr, final_parent_addr_full);
                
                if (len >= sizeof(cmd) || len < 0) {
                    fprintf(stderr, "[FATAL ERROR] Child command string failed or too long.\n");
                    exit(1);
                }
                
                // 🚨 子機起動エラーのデバッグ用に出力
                printf("[DEBUG] Executing command: %s\n", cmd); 
                system(cmd);   
            }

            break;
        } else {
            attempt++;
            pthread_cancel(t1);
            uint8_t adv_disable = 0x00;
            if (hci_send_cmd(sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                             1, &adv_disable) < 0) {
                // ignore
            }

            if (attempt >= MAX_ATTEMPTS) {
                printf("Attempts exhausted (performed %d attempts). Ending regardless of mismatch.\n", attempt);
                break;
            } else {
                printf("Parent mismatch detected. Waiting %d seconds and retrying (attempt %d of %d)...\n", RETRY_DELAY_SEC, attempt+1, MAX_ATTEMPTS);
                sleep(RETRY_DELAY_SEC);
                final_success = 0;
                strncpy(final_parent_addr_last3, "(none)", sizeof(final_parent_addr_last3));
                strncpy(final_parent_addr_full, "(none)", sizeof(final_parent_addr_full));
            }
        }
    } // attempts loop

    close(sock);
    return 0;
}
