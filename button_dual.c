// ================================================
// BLE Dual "Stable Connection" Version
// - ソケット分離による通信安定化
// - WiringPiによるLED制御 (青:待機, 赤:選挙, 緑:成功)
// - ★修正済: ADV_INTERVAL_MS を 50ms に短縮し、開始信号の検出速度を向上
// - ★修正済: get_last_three をアドレス末尾2バイト（XX:XX形式, 5文字）切り出しに修正し、比較文字列の不整合を解消
// ================================================
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <pthread.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <wiringPi.h> // GPIO制御用

// --- 設定 ---
#define ADV_INTERVAL_MS      50   // ★修正: 広告間隔を50msに短縮
#define SCAN_INTERVAL_MS     100  // 反応速度優先
#define SCAN_WINDOW_MS       100  // 常に聞き耳を立てる
#define MAX_DEVS             100
#define PARENT_EXCHANGE_SEC  20   // 選挙時間

// --- GPIOピン設定 (BCM番号) ---
#define PIN_LED_RED    12
#define PIN_LED_GREEN  13
#define PIN_LED_BLUE   19
#define PIN_BUTTON     16

// --- グローバル変数 ---
int dev_id;
int global_adv_sock; // ★送信(Advertise)専用ソケット
int my_key;
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
char perceived_parent_addr_last3[18] = "(unknown)"; // 実際は末尾5文字（XX:XX）
char perceived_parent_addr_full[18]  = "(unknown)";
pthread_mutex_t parent_mutex = PTHREAD_MUTEX_INITIALIZER;

int start_flag = 0;   
pthread_mutex_t start_mutex = PTHREAD_MUTEX_INITIALIZER;

int final_success = 0;
char final_parent_addr_full[32]  = "(none)";


// ==================================================
// LED制御関数
// ==================================================
void led_all_off() {
    digitalWrite(PIN_LED_RED, 0);
    digitalWrite(PIN_LED_GREEN, 0);
    digitalWrite(PIN_LED_BLUE, 0);
}

void led_set_blue() {
    led_all_off();
    digitalWrite(PIN_LED_BLUE, 1);
}

void led_set_red() {
    led_all_off();
    digitalWrite(PIN_LED_RED, 1);
}

void led_set_green() {
    led_all_off();
    digitalWrite(PIN_LED_GREEN, 1);
}

void led_init() {
    pinMode(PIN_LED_RED, OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    pinMode(PIN_LED_BLUE, OUTPUT);
    led_all_off();
}

// ==================================================
// ユーティリティ
// ==================================================
int generate_unique_key(bdaddr_t *addr) {
    unsigned int h = 0;
    for (int i = 0; i < 6; i++) h = h * 257u + (unsigned int)addr->b[i];
    return (int)(h & 0xFFu);
}

// ★修正箇所: get_last_threeを末尾2バイト（XX:XX形式, 5文字）切り出しに修正
void get_last_three(const char *full_addr, char *out, size_t out_sz) {
    out[0] = '\0';
    // 末尾2バイト (XX:XX) の切り出しに必要な長さチェック
    if (!full_addr || strlen(full_addr) < 5 || out_sz < 6) return;
    
    // 末尾5文字（XX:XX）の開始位置
    const char *s = full_addr + strlen(full_addr) - 5;
    
    // 5文字をコピー
    strncpy(out, s, 5); 
    
    // 6文字目を必ずNULL終端にする
    out[5] = '\0'; 
}

// 親タグからアドレスを抽出
void extract_parent_tag_from_name(const char *name, char *out_parent, size_t out_sz) {
    out_parent[0] = '\0';
    const char *p = strstr(name, "|P:");
    if (!p) return;
    
    p += 3; // "|P:" の次からがアドレス
    
    size_t i = 0;
    // '|' か '\0' に達するまでコピー
    while (*p != '\0' && *p != '|' && i + 1 < out_sz) out_parent[i++] = *p++;
    
    // 必ずNULL終端にする
    out_parent[i] = '\0';
}

int extract_start_flag_from_name(const char *name) {
    const char *p = strstr(name, "|S");
    if (!p) return 0;
    char c = *(p + 2);
    if (c == '\0' || c == '|') return 1;
    return 0;
}

int get_start_flag(void) {
    int v;
    pthread_mutex_lock(&start_mutex);
    v = start_flag;
    pthread_mutex_unlock(&start_mutex);
    return v;
}

void set_start_flag(void) {
    pthread_mutex_lock(&start_mutex);
    if (!start_flag) {
        start_flag = 1;
        printf("[INFO] Start flag set! LED turning RED.\n");
        led_set_red(); // ここで赤点灯
    }
    pthread_mutex_unlock(&start_mutex);
}

// =========================
// 広告スレッド (★送信専用ソケット使用)
// =========================
void *advertise_thread(void *arg) {
    le_set_advertising_parameters_cp adv_params_cp;
    memset(&adv_params_cp, 0, sizeof(adv_params_cp));
    // ADV_INTERVAL_MS に基づいて設定 (50ms)
    adv_params_cp.min_interval = htobs((uint16_t)(ADV_INTERVAL_MS * 1.6));
    adv_params_cp.max_interval = htobs((uint16_t)(ADV_INTERVAL_MS * 1.6));
    adv_params_cp.advtype = 0x00;
    adv_params_cp.own_bdaddr_type = 0x00;
    adv_params_cp.chan_map = 0x07;
    adv_params_cp.filter = 0x00;

    // global_adv_sock を使用
    hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    uint8_t adv_data[31];
    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;

    while (1) {
        int len = 0;
        memset(adv_data, 0, sizeof(adv_data));
        adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

        char name_field[64];
        pthread_mutex_lock(&parent_mutex);
        int pphase = parent_phase;
        char paddr_copy[18];
        strncpy(paddr_copy, perceived_parent_addr_last3, sizeof(paddr_copy)); // 末尾2バイト（XX:XX）
        pthread_mutex_unlock(&parent_mutex);

        int sf = get_start_flag();

        if (!sf && pphase == 0)      snprintf(name_field, sizeof(name_field), "%s|%d", device_name, my_key);
        else if (sf && pphase == 0)  snprintf(name_field, sizeof(name_field), "%s|%d|S", device_name, my_key);
        else if (!sf && pphase == 1) snprintf(name_field, sizeof(name_field), "%s|%d|P:%s", device_name, my_key, paddr_copy);
        else                         snprintf(name_field, sizeof(name_field), "%s|%d|S|P:%s", device_name, my_key, paddr_copy);

        int name_len = strlen(name_field);
        int remaining = sizeof(adv_data) - len - 2;
        if (name_len > remaining) name_len = remaining;

        adv_data[len++] = name_len + 1;
        adv_data[len++] = 0x09;
        memcpy(&adv_data[len], name_field, name_len);
        len += name_len;

        adv_data_cp_struct.length = len;
        memcpy(adv_data_cp_struct.data, adv_data, len);
        
        // global_adv_sock を使用
        hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_data_cp_struct);

        uint8_t enable = 0x01;
        hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

        usleep(ADV_INTERVAL_MS * 1000); // 50ms ごとに広告を繰り返す
    }
    return NULL;
}

// =========================
// スキャンスレッド (★受信専用ソケット新規作成)
// =========================
void *scan_thread(void *arg) {
    // ★重要: ここで受信専用ソケットを作ることで通信を安定させる
    int scan_sock = hci_open_dev(dev_id);
    if (scan_sock < 0) {
        perror("Failed to open scan socket");
        pthread_exit(NULL);
    }

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(scan_sock, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    le_set_scan_parameters_cp scan_params_cp;
    memset(&scan_params_cp, 0, sizeof(scan_params_cp));
    scan_params_cp.type = 0x01; 
    scan_params_cp.interval = htobs((uint16_t)(SCAN_INTERVAL_MS * 1.6));
    scan_params_cp.window   = htobs((uint16_t)(SCAN_WINDOW_MS * 1.6));
    scan_params_cp.own_bdaddr_type = 0x00;
    // scan_sock を使用
    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);

    uint8_t enable = 0x01;
    uint8_t filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);

    unsigned char buf[HCI_MAX_EVENT_SIZE];

    printf("=== Waiting for button press or remote |S ===\n");
    led_set_blue(); // 青点灯（待機）

    // ★無限待機
    while (!get_start_flag()) {
        int len = read(scan_sock, buf, sizeof(buf));
        if (len < 0) {
            if (errno == EINTR || errno == EAGAIN) continue;
            usleep(100000); continue; 
        }

        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
        evt_le_meta_event *meta = (evt_le_meta_event *)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
            char addr[18]; ba2str(&info->bdaddr, addr);
            char name[128] = "";
            int key = -1;
            int pos = 0;
            while (pos < info->length) {
                uint8_t f_len = info->data[pos];
                if (f_len == 0 || pos + f_len >= info->length) break;
                if (info->data[pos + 1] == 0x09) {
                    int n_len = f_len - 1; if(n_len>127) n_len=127;
                    memcpy(name, &info->data[pos + 2], n_len); name[n_len] = '\0';
                    sscanf(name, "%*[^|]|%d", &key);
                }
                pos += f_len + 1;
            }

            if (key >= 0) {
                pthread_mutex_lock(&detected_mutex);
                int exists = 0;
                for (int j = 0; j < detected_count; j++) {
                    if (strcmp(detected_devices[j].addr, addr) == 0) {
                        detected_devices[j].key = key; exists = 1; break;
                    }
                }
                if (!exists && detected_count < MAX_DEVS) {
                    strncpy(detected_devices[detected_count].addr, addr, 18);
                    detected_devices[detected_count].key = key;
                    detected_count++;
                }
                pthread_mutex_unlock(&detected_mutex);
            }

            if (name[0] != '\0' && extract_start_flag_from_name(name)) {
                printf("[INFO] Remote START detected from %s\n", addr);
                set_start_flag(); // -> 赤点灯
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    // --- 選挙開始 ---
    printf("\n=== Election running... ===\n");
    // 赤点灯は set_start_flag() 内で行われているはずだが念のため
    led_set_red(); 

    uint8_t disable_cmd[2] = { 0x00, 0x00 };
    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd);

    int smallest_key = my_key;
    char smallest_addr_full[18] = "(self)";

    pthread_mutex_lock(&detected_mutex);
    for (int i = 0; i < detected_count; i++) {
        if (detected_devices[i].key < smallest_key) {
            smallest_key = detected_devices[i].key;
            strncpy(smallest_addr_full, detected_devices[i].addr, 18);
        }
    }
    pthread_mutex_unlock(&detected_mutex);

    if (strcmp(smallest_addr_full, "(self)") == 0) strncpy(smallest_addr_full, my_addr, 18);
    
    char smallest_addr_last3[18];
    // ★修正: 末尾2バイト（XX:XX, 5文字）を切り出す
    get_last_three(smallest_addr_full, smallest_addr_last3, 18); 

    if (smallest_key == my_key) printf("✅ I am candidate PARENT (key=%d)\n", my_key);
    else printf("🔹 Parent candidate is %s (key=%d)\n", smallest_addr_full, smallest_key);

    pthread_mutex_lock(&parent_mutex);
    strncpy(perceived_parent_addr_last3, smallest_addr_last3, 18);
    strncpy(perceived_parent_addr_full, smallest_addr_full, 18);
    parent_phase = 1; 
    pthread_mutex_unlock(&parent_mutex);

    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);

    typedef struct { char addr[18]; char rep_parent[18]; } VerEntry;
    VerEntry ver_list[MAX_DEVS];
    int ver_count = 0;
    time_t ver_start = time(NULL);

    while (difftime(time(NULL), ver_start) < PARENT_EXCHANGE_SEC) {
        int len = read(scan_sock, buf, sizeof(buf));
        if (len < 0) continue;
        if (len < (1 + HCI_EVENT_HDR_SIZE)) continue;
        evt_le_meta_event *meta = (evt_le_meta_event *)(buf + (1 + HCI_EVENT_HDR_SIZE));
        if (meta->subevent != EVT_LE_ADVERTISING_REPORT) continue;

        uint8_t reports = meta->data[0];
        uint8_t *offset = meta->data + 1;

        for (int i = 0; i < reports; i++) {
            le_advertising_info *info = (le_advertising_info *)offset;
            char addr[18]; ba2str(&info->bdaddr, addr);
            char name[128] = "";
            int pos = 0;
            while (pos < info->length) {
                uint8_t f_len = info->data[pos];
                if (f_len == 0 || pos + f_len >= info->length) break;
                if (info->data[pos + 1] == 0x09) {
                    int n_len = f_len - 1; if(n_len>127) n_len=127;
                    memcpy(name, &info->data[pos + 2], n_len); name[n_len] = '\0';
                }
                pos += f_len + 1;
            }

            char rep_parent[18] = "";
            extract_parent_tag_from_name(name, rep_parent, 18);
            if (rep_parent[0] != '\0') {
                int found = 0;
                for(int k=0; k<ver_count; k++) {
                    if(strcmp(ver_list[k].addr, addr) == 0) {
                         strncpy(ver_list[k].rep_parent, rep_parent, 18); found=1; break;
                    }
                }
                if(!found && ver_count < MAX_DEVS) {
                    strncpy(ver_list[ver_count].addr, addr, 18);
                    strncpy(ver_list[ver_count].rep_parent, rep_parent, 18);
                    ver_count++;
                    printf("Parent-report from %s => %s\n", addr, rep_parent);
                }
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd);
    close(scan_sock); // 受信ソケットを閉じる

    pthread_mutex_lock(&detected_mutex);
    int total = detected_count + 1;
    pthread_mutex_unlock(&detected_mutex);
    
    printf("\nReceived reports from %d/%d devices.\n", ver_count, total-1);
    
    // ★判定ロジック
    int all_match = 0;
    
    // 自分ひとりだけなら成功とみなす
    if (total == 1) {
        all_match = 1;
        printf("[DEBUG] Only 1 node detected. Success assumed.\n");
    } else if (ver_count > 0) {
        // 誰か一人でも見つけていて、意見が一致していれば成功とみなす（条件緩和）
        int match_count = 0;
        
        // デバッグ追加: 比較対象の文字列を出力して確認
        printf("[DEBUG] --- Verification check --- \n");
        printf("[DEBUG] Perceived Parent Last 2 Bytes: '%s'\n", perceived_parent_addr_last3);

        for(int i=0; i<ver_count; i++) {
             // rep_parent は末尾2バイト（XX:XX形式）が格納されている想定
             int is_match = (strcmp(ver_list[i].rep_parent, perceived_parent_addr_last3) == 0);
             printf("[DEBUG] Report from %s: '%s' (Match=%s)\n", ver_list[i].addr, ver_list[i].rep_parent, is_match ? "YES" : "NO");

             if(is_match) match_count++;
        }
        
        printf("[DEBUG] ver_count=%d (reports received), match_count=%d (reports matching perceived parent).\n", ver_count, match_count);
        
        // ★条件緩和: 1人でも一致していればOKとする
        if (match_count > 0) all_match = 1; 
    }

    if (all_match) {
        pthread_mutex_lock(&parent_mutex);
        strncpy(final_parent_addr_full, perceived_parent_addr_full, 32);
        pthread_mutex_unlock(&parent_mutex);
        final_success = 1;
        printf("🎉 CONFIRMED Parent: %s\n", final_parent_addr_full);
    } else {
        final_success = 0;
        printf("⚠️ Mismatch or missing reports.\n");
    }

    pthread_mutex_lock(&parent_mutex);
    parent_phase = 0;
    pthread_mutex_unlock(&parent_mutex);

    return NULL;
}

// =========================
// ボタンスレッド
// =========================
void *button_thread(void *arg) {
    printf("Button thread active on GPIO%d\n", PIN_BUTTON);
    int last = digitalRead(PIN_BUTTON);

    while (!get_start_flag()) {
        int val = digitalRead(PIN_BUTTON);
        if (last == 1 && val == 0) {
            delay(30);
            if (digitalRead(PIN_BUTTON) == 0) {
                printf("[BUTTON] Pressed! Starting election.\n");
                set_start_flag();
                break;
            }
        }
        last = val;
        delay(10);
    }
    return NULL;
}

// =========================
// メイン関数
// =========================
int main(int argc, char *argv[]) {
    // ★リアルタイムログのための追加: stdoutのバッファリングを無効化する
    setvbuf(stdout, NULL, _IONBF, 0);

    // 1. GPIO初期化 (WiringPi)
    if (wiringPiSetupGpio() < 0) {
        perror("wiringPiSetupGpio failed");
        return 1;
    }
    
    // LEDとボタン設定
    led_init(); 
    pinMode(PIN_BUTTON, INPUT);
    pullUpDnControl(PIN_BUTTON, PUD_UP);

    // 2. Bluetooth初期化 (送信ソケット)
    dev_id = hci_get_route(NULL);
    global_adv_sock = hci_open_dev(dev_id); // ★ここは送信専用にする
    if (dev_id < 0 || global_adv_sock < 0) {
        perror("Opening socket");
        return 1;
    }

    bdaddr_t bdaddr;
    hci_read_bd_addr(global_adv_sock, &bdaddr, 1000);
    ba2str(&bdaddr, my_addr);
    
    // ----------------------------------------------
    // キー生成をランダム化する修正
    // 1. 乱数のシードを設定
    srand(time(NULL) ^ getpid()); 
    // 2. キーをランダムな値で生成 (0から255の範囲)
    my_key = (rand() % 256); 
    // ----------------------------------------------

    printf("Starting... My addr=%s, key=%d (Random)\n", my_addr, my_key);

    pthread_t t_adv, t_scan, t_btn;
    pthread_create(&t_adv, NULL, advertise_thread, NULL);
    pthread_create(&t_scan, NULL, scan_thread, NULL); 
    pthread_create(&t_btn, NULL, button_thread, NULL);

    pthread_join(t_scan, NULL);

    // 終了処理
    pthread_cancel(t_adv);
    pthread_cancel(t_btn);
    uint8_t d=0; 
    hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &d);
    close(global_adv_sock);

    // 結果実行
    if (final_success) {
        led_set_green(); // 緑点灯
        
        if (strcmp(final_parent_addr_full, my_addr) == 0) {
            printf("I am PARENT.\n");
            char cmd[128]; 
            snprintf(cmd, sizeof(cmd), "./parent %s", final_parent_addr_full);
            system(cmd);
        } else {
            // ★修正: ./child に親ノードのフルアドレスも引数として渡す
            printf("I am CHILD.\n");
            char cmd[128]; 
            snprintf(cmd, sizeof(cmd), "./child %s %s", my_addr, final_parent_addr_full);
            system(cmd);
        }
        
        // 10秒待ってから消灯
        sleep(10);
        led_all_off();
    } else {
        printf("Election failed.\n");
        led_set_blue(); // 失敗時は青に戻る
    }

    return 0;
}
