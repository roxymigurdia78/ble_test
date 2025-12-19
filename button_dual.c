// ================================================
// BLE Dual "Stable Connection" Version
// - ソケット分離による通信安定化
// - WiringPiによるLED制御
// - 実行結果(Exit Code)による点滅機能
// - ★修正: parent_key_list.txt の自動生成機能を追加
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
#include <sys/wait.h> // WEXITSTATUS用
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <wiringPi.h>

// --- 設定 ---
#define ADV_INTERVAL_MS      50   
#define SCAN_INTERVAL_MS     100  
#define SCAN_WINDOW_MS       100  
#define MAX_DEVS             100
#define PARENT_EXCHANGE_SEC  20   

// --- GPIOピン設定 (BCM番号) ---
#define PIN_LED_RED    12
#define PIN_LED_GREEN  13
#define PIN_LED_BLUE   19
#define PIN_BUTTON     16

// --- グローバル変数 ---
int dev_id;
int global_adv_sock;
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
char perceived_parent_addr_last3[18] = "(unknown)";
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

// LED点滅関数
void led_blink(int pin, int interval_ms) {
    led_all_off();
    for (int i = 0; i < 3; i++) {
        digitalWrite(pin, 1);
        usleep(interval_ms * 1000);
        digitalWrite(pin, 0);
        usleep(interval_ms * 1000);
    }
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
void get_last_three(const char *full_addr, char *out, size_t out_sz) {
    out[0] = '\0';
    if (!full_addr || strlen(full_addr) < 5 || out_sz < 6) return;
    const char *s = full_addr + strlen(full_addr) - 5;
    strncpy(out, s, 5); 
    out[5] = '\0'; 
}

void extract_parent_tag_from_name(const char *name, char *out_parent, size_t out_sz) {
    out_parent[0] = '\0';
    const char *p = strstr(name, "|P:");
    if (!p) return;
    p += 3; 
    size_t i = 0;
    while (*p != '\0' && *p != '|' && i + 1 < out_sz) out_parent[i++] = *p++;
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
        led_set_red(); 
    }
    pthread_mutex_unlock(&start_mutex);
}

// ★追加: 発見したデバイスリストをファイルに保存する関数
void save_device_list_for_parent(const char *filename) {
    FILE *fp = fopen(filename, "w");
    if (!fp) {
        perror("[ERROR] Failed to open key list file for writing");
        return;
    }

    // 1. 自分自身を書き込む
    fprintf(fp, "%d %s\n", my_key, my_addr);

    // 2. 発見した他のデバイスを書き込む
    pthread_mutex_lock(&detected_mutex);
    for (int i = 0; i < detected_count; i++) {
        // キーが有効(0以上)な場合のみ書き込む
        if (detected_devices[i].key >= 0) {
            fprintf(fp, "%d %s\n", detected_devices[i].key, detected_devices[i].addr);
        }
    }
    pthread_mutex_unlock(&detected_mutex);

    fclose(fp);
    printf("[INFO] Device list auto-saved to %s (Total: %d nodes including self)\n", filename, detected_count + 1);
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
    adv_params_cp.chan_map = 0x07;
    hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_PARAMETERS, sizeof(adv_params_cp), &adv_params_cp);

    uint8_t adv_data[31];
    struct { uint8_t length; uint8_t data[31]; } __attribute__((packed)) adv_data_cp_struct;

    while (1) {
        int len = 0;
        memset(adv_data, 0, sizeof(adv_data));
        adv_data[len++] = 2; adv_data[len++] = 0x01; adv_data[len++] = 0x06;

        char name_field[64];
        pthread_mutex_lock(&parent_mutex);
        char paddr_copy[18];
        strncpy(paddr_copy, perceived_parent_addr_last3, sizeof(paddr_copy));
        int pphase = parent_phase;
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
        
        hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISING_DATA, len + 1, &adv_data_cp_struct);
        uint8_t enable = 0x01;
        hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &enable);

        usleep(ADV_INTERVAL_MS * 1000); 
    }
    return NULL;
}

// =========================
// スキャンスレッド
// =========================
void *scan_thread(void *arg) {
    int scan_sock = hci_open_dev(dev_id);
    if (scan_sock < 0) pthread_exit(NULL);

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
    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_PARAMETERS, sizeof(scan_params_cp), &scan_params_cp);

    uint8_t enable = 0x01, filter_dup = 0x00;
    uint8_t cmd[2] = { enable, filter_dup };
    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(cmd), cmd);

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    printf("=== Waiting for button press or remote |S ===\n");
    led_set_blue(); 

    while (!get_start_flag()) {
        int len = read(scan_sock, buf, sizeof(buf));
        if (len < 0) { if (errno==EINTR || errno==EAGAIN) continue; usleep(100000); continue; }
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
                printf("[INFO] Remote START from %s\n", addr);
                set_start_flag(); 
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    printf("\n=== Election running... ===\n");
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
    get_last_three(smallest_addr_full, smallest_addr_last3, 18); 

    if (smallest_key == my_key) printf("✅ I am candidate PARENT (key=%d)\n", my_key);
    else printf("🔹 Parent candidate: %s (key=%d)\n", smallest_addr_full, smallest_key);

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
                }
            }
            offset = (uint8_t *)info + sizeof(*info) + info->length;
        }
    }

    hci_send_cmd(scan_sock, OGF_LE_CTL, OCF_LE_SET_SCAN_ENABLE, sizeof(disable_cmd), disable_cmd);
    close(scan_sock); 

    pthread_mutex_lock(&detected_mutex);
    int total = detected_count + 1;
    pthread_mutex_unlock(&detected_mutex);
    
    printf("\nReceived reports from %d/%d devices.\n", ver_count, total-1);
    
    int all_match = 0;
    if (total == 1) {
        all_match = 1;
    } else if (ver_count > 0) {
        int match_count = 0;
        for(int i=0; i<ver_count; i++) {
             if(strcmp(ver_list[i].rep_parent, perceived_parent_addr_last3) == 0) match_count++;
        }
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
        printf("⚠️ Election failed.\n");
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
    int last = digitalRead(PIN_BUTTON);
    while (!get_start_flag()) {
        int val = digitalRead(PIN_BUTTON);
        if (last == 1 && val == 0) {
            delay(30);
            if (digitalRead(PIN_BUTTON) == 0) {
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
// main
// =========================
int main(int argc, char *argv[]) {
    setvbuf(stdout, NULL, _IONBF, 0);

    if (wiringPiSetupGpio() < 0) return 1;
    led_init(); 
    pinMode(PIN_BUTTON, INPUT);
    pullUpDnControl(PIN_BUTTON, PUD_UP);

    dev_id = hci_get_route(NULL);
    global_adv_sock = hci_open_dev(dev_id); 
    if (dev_id < 0 || global_adv_sock < 0) return 1;

    bdaddr_t bdaddr;
    hci_read_bd_addr(global_adv_sock, &bdaddr, 1000);
    ba2str(&bdaddr, my_addr);
    
    srand(time(NULL) ^ getpid()); 
    my_key = (rand() % 256); 

    printf("Starting... Addr=%s, Key=%d\n", my_addr, my_key);

    pthread_t t_adv, t_scan, t_btn;
    pthread_create(&t_adv, NULL, advertise_thread, NULL);
    pthread_create(&t_scan, NULL, scan_thread, NULL); 
    pthread_create(&t_btn, NULL, button_thread, NULL);

    pthread_join(t_scan, NULL);

    pthread_cancel(t_adv);
    pthread_cancel(t_btn);
    uint8_t d=0; 
    hci_send_cmd(global_adv_sock, OGF_LE_CTL, OCF_LE_SET_ADVERTISE_ENABLE, 1, &d);
    close(global_adv_sock);

    // ★実行と結果判定
    if (final_success) {
        printf("Election Success. Running main program...\n");
        led_set_green(); // 実行中緑点灯

        int sys_ret = 0;
        char cmd[128]; 

        if (strcmp(final_parent_addr_full, my_addr) == 0) {
            printf("I am PARENT.\n");
            
            // ★ 親機になる場合、リストをファイルへ自動保存してから起動
            save_device_list_for_parent("parent_key_list.txt");

            snprintf(cmd, sizeof(cmd), "./parent %s", final_parent_addr_full);
            sys_ret = system(cmd);
        } else {
            printf("I am CHILD.\n");
            
            // --- 追加: 検知した台数(detected_count)を環境変数としてセット ---
            // 自分(子機)から見ると「親機+もう1台の子機=2台」が見えているはずなので、
            // detected_count は 2 となり、これが期待するサイクル数と一致します。
            char count_str[16];
            snprintf(count_str, sizeof(count_str), "%d", detected_count);
            setenv("TOTAL_CHILD_NODES", count_str, 1); 
            printf("[INFO] Set TOTAL_CHILD_NODES=%s for child process.\n", count_str);
            // -------------------------------------------------------------

            snprintf(cmd, sizeof(cmd), "./child %s %s", my_addr, final_parent_addr_full);
            sys_ret = system(cmd);
        }

        int exit_code = 0;
        if (WIFEXITED(sys_ret)) {
            exit_code = WEXITSTATUS(sys_ret);
        } else {
            exit_code = -1; 
        }

        printf("Main program exit code: %d\n", exit_code);

        if (exit_code == 0) {
            printf("✅ SUCCESS: Green Blink x3\n");
            led_blink(PIN_LED_GREEN, 300); 
        } else {
            printf("❌ FAILURE: Red Blink x3\n");
            led_blink(PIN_LED_RED, 300);   
        }
        sleep(1); led_all_off(); 

    } else {
        printf("Election failed.\n");
        led_set_blue(); 
    }
    return 0;
}
