// parent.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define PARENT_ADDR_ARG argv[1]
#define LOG_FILE_PATH   "parent_reception_log.txt"
#define MAP_FILE_PATH   "cube_map_log.txt"
#define MAX_DATA_LEN    64

#define KEY_LIST_FILE   "parent_key_list.txt"
#define MAX_NODES       32

typedef struct {
    int  key;
    char addr[18];
    // マッピング用: 座標
    int x, y, z; 
    // 親機のキーリスト上のインデックス (0から始まる)
    int index;
} KeyEntry;

static KeyEntry g_nodes[MAX_NODES];
static int g_node_count = 0;

// 電磁石タイムの制御用
static pthread_mutex_t mag_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t  mag_cond  = PTHREAD_COND_INITIALIZER;
static char current_mag_target[18] = "";
static int  mag_end_flag = 0;

// マッピング用データ構造
// [ターゲットの子機MAC] [検知した子機MAC] [検知面] を記録
typedef struct {
    char target_addr[18];
    char detected_addr[18];
    char surface[16];
} ReportEntry;

#define MAX_REPORTS 256
static ReportEntry g_reports[MAX_REPORTS];
static int g_report_count = 0;
static pthread_mutex_t report_mutex = PTHREAD_MUTEX_INITIALIZER;

// ==========================================================
// 関数プロトタイプ宣言 
// ==========================================================
void coordinate_mapping(void); 

// ==========================================================
// アドレスから Key を検索するヘルパー関数
// ==========================================================
int get_key_by_addr(const char *addr) {
    for (int i = 0; i < g_node_count; i++) {
        if (strcmp(g_nodes[i].addr, addr) == 0) {
            return g_nodes[i].key;
        }
    }
    return -1; // 見つからない場合
}

// ==========================================================
// ログファイルに書き込みを行う関数 (面情報)
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
// child からの「電磁石タイム終了(ME)」ADV を検知したときに呼ぶ
// ==========================================================
void register_mag_end_from_child(const char *child_addr) {
    pthread_mutex_lock(&mag_mutex);
    if (current_mag_target[0] != '\0' &&
        strcmp(current_mag_target, child_addr) == 0) {
        mag_end_flag = 1;
        pthread_cond_signal(&mag_cond);
    }
    pthread_mutex_unlock(&mag_mutex);
}

// ==========================================================
// 受信データ文字列を解析し、ログに記録・マッピング情報を保存する関数
// (child 側では "<子機MAC>,<面>" を想定)
// ==========================================================
void process_received_data(const char *received_data) {
    char data_copy[MAX_DATA_LEN];
    strncpy(data_copy, received_data, MAX_DATA_LEN);
    data_copy[MAX_DATA_LEN - 1] = '\0';
    
    char *child_addr = strtok(data_copy, ",");
    char *surface    = strtok(NULL, ",");

    if (child_addr && surface) {
        
        int child_key = get_key_by_addr(child_addr); 
        
        // Key情報を含める
        printf("Surface-report from %s (Key: %d) => %s (Target: %s)\n", 
               child_addr, 
               child_key,
               surface, 
               current_mag_target[0] != '\0' ? current_mag_target : "None");

        // ログファイルにも保存
        write_log(child_addr, surface);

        // マッピング情報を保存
        pthread_mutex_lock(&report_mutex);
        // current_mag_target は電磁石を駆動したノードのアドレス
        if (g_report_count < MAX_REPORTS && current_mag_target[0] != '\0') {
            strncpy(g_reports[g_report_count].target_addr, current_mag_target, 18);
            strncpy(g_reports[g_report_count].detected_addr, child_addr, 18);
            strncpy(g_reports[g_report_count].surface, surface, 16);
            g_reports[g_report_count].target_addr[17] = '\0';
            g_reports[g_report_count].detected_addr[17] = '\0';
            g_reports[g_report_count].surface[15] = '\0';
            g_report_count++;
        }
        pthread_mutex_unlock(&report_mutex);

    } else {
        fprintf(stderr,
                "[ERROR] 受信データの形式が不正です: %s\n",
                received_data ? received_data : "(null)");
    }
}

// ==========================================================
// BLE 受信スレッド
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

    printf("[COMM] BLEスキャンを開始しました。子機からの SURFACE / ME 広告を待っています...\n");

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

                    // ここでは余計なデバッグ表示はしない
                    process_received_data(combined);
                }

                // --- 追加: 電磁石タイム終了通知 (Local Name == "ME") を検知 ---
                if (strncmp(name, "ME", 2) == 0) {
                    // この広告の送信元アドレス addr が current_mag_target と一致していれば終了
                    register_mag_end_from_child(addr);
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

// ==========================================================
// parent_key_list.txt を読み込む
// ==========================================================
int load_key_list(const char *path) {
    FILE *fp = fopen(path, "r");
    if (!fp) {
        perror("キーリストファイルを開けませんでした");
        return -1;
    }

    g_node_count = 0;
    while (g_node_count < MAX_NODES) {
        int key;
        char addr[18];
        if (fscanf(fp, "%d %17s", &key, addr) != 2) {
            break;
        }
        g_nodes[g_node_count].key = key;
        strncpy(g_nodes[g_node_count].addr, addr, sizeof(g_nodes[g_node_count].addr));
        g_nodes[g_node_count].addr[sizeof(g_nodes[g_node_count].addr) - 1] = '\0';
        g_nodes[g_node_count].x = g_nodes[g_node_count].y = g_nodes[g_node_count].z = -1; // 初期値
        g_nodes[g_node_count].index = g_node_count;
        g_node_count++;
    }

    fclose(fp);
    return g_node_count;
}

// ==========================================================
// 読み込んだキーとアドレスを表示
// ==========================================================
void print_key_list(void) {
    printf("\n--- Parent Key List (ascending) ---\n");
    for (int i = 0; i < g_node_count; i++) {
        printf("  [%d] key=%d addr=%s\n", i, g_nodes[i].key, g_nodes[i].addr);
    }
    printf("-----------------------------------\n");
}


// ==========================================================
// 電磁石タイム用：キー順に子機アドレスを 10 秒ずつアドバタイズする
// ==========================================================
void *mag_sequence_thread(void *arg) {
    (void)arg;

    if (g_node_count <= 0) {
        fprintf(stderr, "[MAG] キー情報がないため、電磁石タイムシーケンスをスキップします。\n");
        return NULL;
    }

    int dev_id = hci_get_route(NULL);
    if (dev_id < 0) {
        perror("[MAG] hci_get_route に失敗しました");
        return NULL;
    }

    int sock_adv = hci_open_dev(dev_id);
    if (sock_adv < 0) {
        perror("[MAG] hci_open_dev に失敗しました");
        return NULL;
    }

    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    uint16_t interval = (uint16_t)(500 * 1.6); // 500ms
    adv_params_cp.min_interval     = htobs(interval);
    adv_params_cp.max_interval     = htobs(interval);
    adv_params_cp.advtype          = 0x00;    // Connectable undirected
    adv_params_cp.own_bdaddr_type  = 0x00;    // Public
    adv_params_cp.chan_map         = 0x07;
    adv_params_cp.filter           = 0x00;

    if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS,
                     sizeof(adv_params_cp), &adv_params_cp) < 0) {
        perror("[MAG] アドバタイズパラメータ設定に失敗しました");
        close(sock_adv);
        return NULL;
    }

    for (int idx = 0; idx < g_node_count; idx++) {
        const char *target_addr = g_nodes[idx].addr;
        int key = g_nodes[idx].key;

        // ★ 親機自身の MAC アドレスが対象の場合 (idx == 0)
        if (idx == 0) {
            printf("[MAG] key=%d addr=%s は親機自身です。電磁石を駆動し、隣接キューブのレポートを待ちます。\n",
                   key, target_addr);
            
            // 1. 親機電磁石駆動開始 (current_mag_targetを設定)
            pthread_mutex_lock(&mag_mutex);
            strncpy(current_mag_target, target_addr, sizeof(current_mag_target));
            current_mag_target[sizeof(current_mag_target) - 1] = '\0';
            mag_end_flag = 0; 
            pthread_mutex_unlock(&mag_mutex);

            printf("[MAG] 親機の電磁石を駆動中 (10秒間)... `./coil start 10` を実行します。\n");
            
            // ★ coil.c を呼び出し、10秒間駆動させる
            system("./coil start 10");
            
            // 2. 親機駆動終了。
            pthread_mutex_lock(&mag_mutex);
            current_mag_target[0] = '\0'; // ターゲットをリセット
            pthread_mutex_unlock(&mag_mutex);

            // 親機は ME 通知を待たないので、レポート受信のために余裕を持たせる
            printf("[MAG] 親機駆動終了。5秒間のクールダウン後、次の子機へ移行します。\n");
            sleep(5);
            continue; 
        }

        // --- 子機をターゲットとした MT アドバタイズ ---
        
        // 現在ターゲットとする子機アドレスを更新
        pthread_mutex_lock(&mag_mutex);
        strncpy(current_mag_target, target_addr, sizeof(current_mag_target));
        current_mag_target[sizeof(current_mag_target) - 1] = '\0';
        mag_end_flag = 0;
        pthread_mutex_unlock(&mag_mutex);

        // --- 広告データ "MT:<addr>" 作成 ---
        uint8_t adv_data[31];
        memset(adv_data, 0, sizeof(adv_data));
        int len = 0;

        // Flags
        adv_data[len++] = 2;
        adv_data[len++] = 0x01;
        adv_data[len++] = 0x06;

        char name_field[32];
        snprintf(name_field, sizeof(name_field), "MT:%s", target_addr);
        int name_len = (int)strlen(name_field);
        if (name_len > 26) name_len = 26; // 31 - 5

        adv_data[len++] = (uint8_t)(name_len + 1);
        adv_data[len++] = 0x09; // Complete Local Name
        memcpy(&adv_data[len], name_field, name_len);
        len += name_len;

        struct {
            uint8_t length;
            uint8_t data[31];
        } __attribute__((packed)) adv_data_cp_struct;

        adv_data_cp_struct.length = (uint8_t)len;
        memset(adv_data_cp_struct.data, 0, sizeof(adv_data_cp_struct.data));
        memcpy(adv_data_cp_struct.data, adv_data, len);

        if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA,
                         len + 1, &adv_data_cp_struct) < 0) {
            perror("[MAG] 広告データ設定に失敗しました");
            continue;
        }

        uint8_t enable = 0x01;
        if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                         1, &enable) < 0) {
            perror("[MAG] アドバタイズ開始に失敗しました");
            continue;
        }

        printf("[MAG] key=%d addr=%s に対して 10 秒間 MT アドバタイズ開始\n",
               key, target_addr);
        sleep(10); // 10秒間広告を送信

        enable = 0x00;
        if (hci_send_cmd(sock_adv, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE,
                         1, &enable) < 0) {
            perror("[MAG] アドバタイズ停止に失敗しました");
        }

        // child からの「電磁石タイム終了 (ME)」ADV を待つ
        printf("[MAG] addr=%s からの ME 通知を待機...\n", target_addr);
        pthread_mutex_lock(&mag_mutex);
        struct timespec ts;
        // タイムアウト設定: 30秒
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_sec += 30; 
        
        while (!mag_end_flag) {
            if (pthread_cond_timedwait(&mag_cond, &mag_mutex, &ts) == ETIMEDOUT) {
                fprintf(stderr, "[MAG] 警告: ME通知がタイムアウトしました (addr=%s)。次の子機へ移行します。\n", target_addr);
                break;
            }
        }
        pthread_mutex_unlock(&mag_mutex);
        
        if (mag_end_flag) {
            printf("[MAG] ME 通知を受信しました。\n");
        }

        // クールダウン
        printf("[MAG] 5秒間のクールダウン後、次の子機へ移行します。\n");
        sleep(5);
    }

    pthread_mutex_lock(&mag_mutex);
    current_mag_target[0] = '\0';
    mag_end_flag = 0;
    pthread_mutex_unlock(&mag_mutex);

    close(sock_adv);
    printf("[MAG] 全ての子機への電磁石タイム通知が完了しました。\n");
    
    // --- マッピング処理の開始 ---
    coordinate_mapping(); 
    
    return NULL;
}

// ==========================================================
// マッピング処理 (メイン処理から呼ばれる)
// ==========================================================
void coordinate_mapping(void) {
    printf("\n\n===== マッピング処理開始 =====\n");
    FILE *fp = fopen(MAP_FILE_PATH, "w");
    if (!fp) {
        perror("マップファイルを開けませんでした");
        return;
    }
    fprintf(fp, "# Cube Mapping Log\n");

    // 親機を (0, 0, 0) に設定
    int parent_index = -1;
    for (int i = 0; i < g_node_count; i++) {
        if (g_nodes[i].index == 0) { // g_nodes[0] は常に親機
            g_nodes[i].x = 0;
            g_nodes[i].y = 0;
            g_nodes[i].z = 0;
            parent_index = i;
            break;
        }
    }
    
    if (parent_index == -1) {
        fprintf(stderr, "[MAP] 致命的なエラー: 親機情報が見つかりません。\n");
        fclose(fp);
        return;
    }

    // マッピング処理を初期化
    int mapped_count = 1; // 親機自身

    // 座標が未決定の子機がなくなるまでループ (BFS的な処理)
    while (mapped_count < g_node_count) {
        int new_mapped = 0;
        
        // 既に座標が決定しているノードを基準にする
        for (int i = 0; i < g_node_count; i++) {
            if (g_nodes[i].x == -1) continue; // 座標未決定

            int current_x = g_nodes[i].x;
            int current_y = g_nodes[i].y;
            int current_z = g_nodes[i].z;
            const char *current_addr = g_nodes[i].addr;

            // このノードをターゲットとしたレポートをチェック
            pthread_mutex_lock(&report_mutex);
            for (int r = 0; r < g_report_count; r++) {
                // target_addr: 電磁石を駆動させたノード (i番目のノード)
                // detected_addr: それを検知した隣接ノード (座標未決定かもしれないノード)
                if (strcmp(g_reports[r].target_addr, current_addr) == 0) {
                    
                    const char *detected_addr = g_reports[r].detected_addr;
                    const char *surface = g_reports[r].surface;
                    
                    // detected_addr が既に座標決定済みかチェック
                    int detected_index = -1;
                    for (int j = 0; j < g_node_count; j++) {
                        if (strcmp(g_nodes[j].addr, detected_addr) == 0) {
                            detected_index = j;
                            break;
                        }
                    }

                    if (detected_index != -1 && g_nodes[detected_index].x == -1) {
                        // 新しい座標を決定
                        int new_x = current_x;
                        int new_y = current_y;
                        int new_z = current_z;
                        
                        // 検出面から隣接ノードの相対座標を決定 (簡易版)
                        if (strcmp(surface, "FRONT") == 0)      new_y += 1; // Y+方向
                        else if (strcmp(surface, "BACK") == 0)  new_y -= 1; // Y-方向
                        else if (strcmp(surface, "LEFT") == 0)  new_x -= 1; // X-方向
                        else if (strcmp(surface, "RIGHT") == 0) new_x += 1; // X+方向
                        else if (strcmp(surface, "TOP") == 0)   new_z += 1; // Z+方向
                        else if (strcmp(surface, "BOTTOM") == 0)new_z -= 1; // Z-方向

                        // 重複チェック (本来は必要だが、ここではシンプルに設定)
                        g_nodes[detected_index].x = new_x;
                        g_nodes[detected_index].y = new_y;
                        g_nodes[detected_index].z = new_z;
                        new_mapped++;
                        mapped_count++;
                        
                        printf("[MAP] Mapped %s at (%d, %d, %d) from %s (Surface: %s)\n",
                               detected_addr, new_x, new_y, new_z, current_addr, surface);
                    }
                }
            }
            pthread_mutex_unlock(&report_mutex);
        }

        if (new_mapped == 0 && mapped_count < g_node_count) {
            printf("[MAP] 警告: 座標未決定のノードがありますが、新しいマッピングができませんでした。\n");
            break;
        }
    }

    // 結果を出力
    fprintf(fp, "\n# Final Cube Coordinates\n");
    for (int i = 0; i < g_node_count; i++) {
        fprintf(fp, "[%s] Key: %d, Coords: (%d, %d, %d)\n", 
                g_nodes[i].addr, g_nodes[i].key, g_nodes[i].x, g_nodes[i].y, g_nodes[i].z);
    }
    
    // 全レポートを出力
    fprintf(fp, "\n# Raw Reports (Target | Detected | Surface)\n");
    pthread_mutex_lock(&report_mutex);
    for (int r = 0; r < g_report_count; r++) {
        fprintf(fp, "%s | %s | %s\n", 
                g_reports[r].target_addr, g_reports[r].detected_addr, g_reports[r].surface);
    }
    pthread_mutex_unlock(&report_mutex);

    fclose(fp);
    printf("===== マッピング処理完了。結果は %s に出力されました。 =====\n", MAP_FILE_PATH);
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
    printf("役割: 子機からの面情報を受信し、電磁石タイムを制御、ログとマッピングを記録します。\n");

    // --- キーリストの読み込みと表示 ---
    int loaded = load_key_list(KEY_LIST_FILE);
    if (loaded > 0) {
        print_key_list();
        printf("\n--- 電磁石タイム ---\n");
        printf("10秒間の準備フェーズに入ります...\n");
        sleep(10);
    } else {
        printf("[WARN] %s が読み込めなかったため、キー情報なしで起動します。\n",
               KEY_LIST_FILE);
    }

    pthread_t rx_thread;
    if (pthread_create(&rx_thread, NULL,
                       BLE_receive_data_server, NULL) != 0) {
        perror("BLE受信スレッドの生成に失敗しました");
        return 1;
    }

    // 電磁石タイムシーケンススレッド（キーが取れている場合のみ）
    pthread_t mag_thread;
    if (loaded > 0) {
        if (pthread_create(&mag_thread, NULL,
                           mag_sequence_thread, NULL) != 0) {
            perror("電磁石タイムシーケンススレッドの生成に失敗しました");
        }
    }

    printf("親機メイン処理を実行中...\n");
    
    // mag_thread を待つことで、電磁石タイムとマッピング処理を完了させる
    if (loaded > 0) {
        pthread_join(mag_thread, NULL);
    }

    printf("電磁石タイムシーケンスが完了しました。受信スレッドは継続します (Ctrl+Cで終了)。\n");
    // 無限ループの受信スレッドは、ユーザーが Ctrl+C で終了させる想定

    return 0;
}
