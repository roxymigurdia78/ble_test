#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <pthread.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sched.h>

// Linux拡張機能の警告抑制
#define _GNU_SOURCE

// ハードウェア制御
#include <wiringPi.h>
#include <softPwm.h>
#include <softTone.h>

// Bluetoothライブラリ
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

// 楽譜データ
#include "music_data.h"

// --- 設定 ---
#define MAP_FILE "mapping.txt"
#define HCI_DEV_ID 0

// 現在の曲ID
int current_song_id = 0; 

// ★パケットタイプ定義
#define PKT_TYPE_START     0x01 // 演奏開始
#define PKT_TYPE_STOP      0xFF // 強制停止
#define PKT_TYPE_CANDIDATE 0x10 // ★追加: 選挙の立候補パケット

#define SONG_ID_STOP 0xFF

#define PIN_SWITCH  17
#define PIN_SPK     27
#define PIN_LED_R   12
#define PIN_LED_G   13
#define PIN_LED_B   19
#define PIN_EXIT_BUTTON 16

const int SENSORS[] = {14, 15, 18, 23};
#define NUM_SENSORS 4

#define MY_COMPANY_ID 0xFFFF

#define INTERVAL_MS 0  
#define ORIGIN_START_DELAY 1000 
#define GRADATION_SPEED_MS 16666

// ★選挙の期間 (ms)
// 反応してからこの時間だけ、他の候補者と数値を比べ合う
#define ELECTION_WINDOW_MS 300 

// --- 通信パケット定義 (拡張) ---
typedef struct __attribute__((packed)) {
    uint16_t company_id;
    uint16_t seq;
    uint8_t  packet_type; // ★追加: パケットの種類
    uint8_t  ticket;      // ★追加: 選挙用ランダム値 (0-255)
    uint8_t  rank;        // ★追加: タイブレーカー用ランク
    uint8_t  song_id; 
    int8_t   origin_x; 
    int8_t   origin_y; 
    int8_t   origin_z; 
} EffectPayload;

// 構造体定義
typedef struct {
    char mac[18];
    int key; 
    int x; int y; int z;
} NodeInfo;

// --- 曲ごとのパート優先度定義 ---
static const int PART_PRIORITY_SONG0[] = {1, 2, 3, 4, 5, 6};
static const int PRIORITY_LEN_SONG0 = 6;
static const int PART_PRIORITY_SONG1[] = {1, 2, 3, 4, 5, 6, 7};
static const int PRIORITY_LEN_SONG1 = 7;

static const int *CURRENT_PRIORITY = NULL;
static int CURRENT_PRIORITY_LEN = 0;

void select_priority_for_song(int song_id) {
    current_song_id = song_id;

    if (song_id == 0) {
        CURRENT_PRIORITY = PART_PRIORITY_SONG0;
        CURRENT_PRIORITY_LEN = PRIORITY_LEN_SONG0;
    } else if (song_id == 1) {
        CURRENT_PRIORITY = PART_PRIORITY_SONG1;
        CURRENT_PRIORITY_LEN = PRIORITY_LEN_SONG1;
    } else {
        CURRENT_PRIORITY = PART_PRIORITY_SONG0;     
        CURRENT_PRIORITY_LEN = 6;
        current_song_id = 0;
    }
}

// --- グローバル変数 ---
int device_handle = -1;
char my_mac_addr[18] = {0};
int my_part_id = 1; 
int my_rank = 0;
int my_x = 0, my_y = 0, my_z = 0;

static volatile int is_playing = 0;
static unsigned long last_trigger_time = 0;
static int last_handled_seq = -1;

// ★選挙ロジック用変数
static int election_running = 0;        // 選挙中か？
static unsigned long election_start_time = 0; // 選挙開始時刻
static uint8_t my_ticket = 0;           // 自分のくじ引き番号
static int pending_song_id = 0;         // 候補中の曲
static int pending_trigger_pin = -1;    // 押されたピン

// ---------------------------------------------------------------------------
// 高精度タイマー & ユーティリティ
// ---------------------------------------------------------------------------
static inline unsigned long get_time_ms_raw() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (unsigned long)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}
#define get_time_ms get_time_ms_raw

static void sleep_ms(unsigned long ms) {
    if (ms == 0) return;
    struct timespec req;
    req.tv_sec = ms / 1000;
    req.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&req, NULL);
}

static void sleep_us(unsigned long us) {
    if (us == 0) return;
    struct timespec req;
    req.tv_sec = us / 1000000;
    req.tv_nsec = (us % 1000000) * 1000L;
    nanosleep(&req, NULL);
}

static pthread_mutex_t play_mtx = PTHREAD_MUTEX_INITIALIZER;

static int try_start_playing(void) {
    int ok = 0;
    pthread_mutex_lock(&play_mtx);
    if (!is_playing) {
        is_playing = 1;
        ok = 1;
    }
    pthread_mutex_unlock(&play_mtx);
    return ok;
}

static void stop_playing(void) {
    pthread_mutex_lock(&play_mtx);
    is_playing = 0;
    pthread_mutex_unlock(&play_mtx);
}

static void try_set_realtime_priority() {
    struct sched_param p;
    p.sched_priority = 50; 
    if (sched_setscheduler(0, SCHED_FIFO, &p) == 0) {
        printf("[INFO] Set SCHED_FIFO priority %d\n", p.sched_priority);
    }
}

int kbhit(void) {
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if(ch != EOF) { ungetc(ch, stdin); return 1; }
    return 0;
}

// ---------------------------------------------------------------------------
// LED & Music
// ---------------------------------------------------------------------------
void set_led_color(int r, int g, int b) {
    if(r < 0) r = 0; if(r > 100) r = 100;
    if(g < 0) g = 0; if(g > 100) g = 100;
    if(b < 0) b = 0; if(b > 100) b = 100;

    // 元のコードの設定を維持 (Active High / カソードコモン想定)
    softPwmWrite(PIN_LED_R, r);
    softPwmWrite(PIN_LED_G, g);
    softPwmWrite(PIN_LED_B, b);
}

void hsv_to_rgb(float h, float s, float v, int *r, int *g, int *b) {
    float c = v * s;
    float x = c * (1.0f - fabs(fmod(h / 60.0f, 2.0f) - 1.0f));
    float m = v - c;
    float rp, gp, bp;
    if (h < 60.0f)       { rp = c; gp = x; bp = 0; }
    else if (h < 120.0f) { rp = x; gp = c; bp = 0; }
    else if (h < 180.0f) { rp = 0; gp = c; bp = x; }
    else if (h < 240.0f) { rp = 0; gp = x; bp = c; }
    else if (h < 300.0f) { rp = x; gp = 0; bp = c; }
    else                 { rp = c; gp = 0; bp = x; }
    *r = (int)((rp + m) * 100.0f);
    *g = (int)((gp + m) * 100.0f);
    *b = (int)((bp + m) * 100.0f);
}

typedef struct { int hop; unsigned long start_time_ms; } LedArgs;

void *led_thread_func(void *arg) {
    LedArgs *args = (LedArgs*)arg;
    int hop = args->hop;
    unsigned long base_time = args->start_time_ms;
    free(arg);

    float bpm = get_song_bpm(current_song_id);
    if (bpm <= 0) bpm = 120.0f;
    unsigned long beat_ms = (unsigned long)(60000.0f / bpm);

    while (is_playing) {
        unsigned long now = get_time_ms();
        long elapsed = (long)(now - base_time);

        if (elapsed < 0) {
            set_led_color(0, 0, 0);
            sleep_ms(5);
            continue;
        }

        int beat = elapsed / beat_ms;
        int max_hop = beat;

        if (hop <= max_hop) {
            float hue = fmodf((beat * 30.0f) + (hop * 30.0f), 360.0f);
            int r, g, b;
            hsv_to_rgb(hue, 1.0f, 1.0f, &r, &g, &b);
            set_led_color(r, g, b);
        } else {
            set_led_color(0, 0, 0);
        }
        sleep_ms(10);
    }
    set_led_color(0, 0, 0);
    return NULL;
}

static void play_note_for_us(int freq, unsigned long usec) {
    if (freq > 0) {
        softToneWrite(PIN_SPK, freq);
        sleep_us(usec);
        softToneWrite(PIN_SPK, 0);
    } else {
        sleep_us(usec);
    }
}

// ---------------------------------------------------------------------------
// BLE 送受信 (選挙用パケット対応)
// ---------------------------------------------------------------------------
void send_packet(int seq, int type, int ticket, int song_id, int ox, int oy, int oz) {
    hci_le_set_advertise_enable(device_handle, 0, 1000);
    hci_le_set_scan_enable(device_handle, 0x00, 0x00, 1000);

    EffectPayload payload;
    payload.company_id = MY_COMPANY_ID;
    payload.seq = (uint16_t)seq;
    payload.packet_type = (uint8_t)type; // ★種類
    payload.ticket = (uint8_t)ticket;    // ★くじ引き番号
    payload.rank = (uint8_t)my_rank;     // ★ランク(同じ番号だった時の判定用)
    payload.song_id = (uint8_t)song_id; 
    payload.origin_x = (int8_t)ox;
    payload.origin_y = (int8_t)oy;
    payload.origin_z = (int8_t)oz;

    uint8_t adv_data[31]; memset(adv_data, 0, sizeof(adv_data));
    int idx = 0;
    adv_data[idx++] = 2; adv_data[idx++] = 0x01; adv_data[idx++] = 0x06;
    int payload_len = sizeof(EffectPayload);
    adv_data[idx++] = payload_len + 1;
    adv_data[idx++] = 0xFF;
    memcpy(&adv_data[idx], &payload, payload_len);
    idx += payload_len;

    struct hci_request rq;
    le_set_advertising_data_cp data_cp;
    memset(&data_cp, 0, sizeof(data_cp));
    data_cp.length = idx;
    memcpy(data_cp.data, adv_data, 31);

    memset(&rq, 0, sizeof(rq));
    rq.ogf = OGF_LE_CTL; rq.ocf = OCF_LE_SET_ADVERTISING_DATA;
    rq.cparam = &data_cp; rq.clen = LE_SET_ADVERTISING_DATA_CP_SIZE;
    rq.rparam = NULL; rq.rlen = 0; rq.event = EVT_CMD_COMPLETE;
    hci_send_req(device_handle, &rq, 1000);

    le_set_advertising_parameters_cp params_cp;
    memset(&params_cp, 0, sizeof(params_cp));
    params_cp.min_interval = htobs(0x0040); 
    params_cp.max_interval = htobs(0x0040);
    params_cp.advtype = 0x00; params_cp.chan_map = 0x07; params_cp.filter = 0x00;
    memset(&rq, 0, sizeof(rq));
    rq.ogf = OGF_LE_CTL; rq.ocf = OCF_LE_SET_ADVERTISING_PARAMETERS;
    rq.cparam = &params_cp; rq.clen = LE_SET_ADVERTISING_PARAMETERS_CP_SIZE;
    rq.rparam = NULL; rq.rlen = 0; rq.event = EVT_CMD_COMPLETE;
    hci_send_req(device_handle, &rq, 1000);

    hci_le_set_advertise_enable(device_handle, 1, 1000);
    sleep_ms(100); 
    hci_le_set_advertise_enable(device_handle, 0, 1000);
    
    hci_le_set_scan_parameters(device_handle, 0x01, 0x40, 0x40, 0x00, 0x00, 1000);
    hci_le_set_scan_enable(device_handle, 0x01, 0x00, 1000);
}

// ---------------------------------------------------------------------------
// 音楽再生制御
// ---------------------------------------------------------------------------
typedef struct { unsigned long target_time; int seq; int dist; int ox; int oy; int oz; } ScheduleArgs;

void *scheduled_play_thread(void *arg) {
    ScheduleArgs *sa = (ScheduleArgs*)arg;
    unsigned long target = sa->target_time;
    int seq = sa->seq;
    int dist = sa->dist;
    free(sa);

    printf("   [SCHED] Sync Start at %lu (Wait %ld ms) SongID:%d\n", 
            target, (long)(target - get_time_ms()), current_song_id);

    while (1) {
        unsigned long now = get_time_ms();
        long diff = (long)(target - now);
        if (diff <= 0) break; 
        if (diff > 10) sleep_ms(diff - 8);
        else { while ((long)(target - get_time_ms()) > 0); break; }
    }

    if (is_playing) return NULL;
    if (!try_start_playing()) return NULL;

    int max_parts = 1;
    // Note: music_data.hの実装に合わせて調整してください
    // ここでは簡易的に2曲決め打ち
    if (current_song_id == 0) max_parts = 6;
    else if (current_song_id == 1) max_parts = 7;

    int dynamic_part_id = 1;
    if (CURRENT_PRIORITY && CURRENT_PRIORITY_LEN > 0) {
        dynamic_part_id = CURRENT_PRIORITY[ my_rank % CURRENT_PRIORITY_LEN ];
    }

    Note *score = get_music_part(current_song_id, dynamic_part_id);
    if (!score) {
        stop_playing();
        return NULL;
    }

    printf("   [EFFECT] ♪ START (Seq:%d, Dist:%d -> Part:%d)\n", seq, dist, dynamic_part_id);

    pthread_t th_led;
    LedArgs *la = malloc(sizeof(LedArgs));
    la->hop = dist; 
    la->start_time_ms = target; 
    pthread_create(&th_led, NULL, led_thread_func, la);
    pthread_detach(th_led);

    int i = 0;
    const int GAP_US = 10000;
    while (score[i].freq != -1 && is_playing) {
        unsigned long total_duration_us = (unsigned long)(score[i].duration_ms * 1000.0f);
        if (score[i].freq > 0) {
            unsigned long play_us = (total_duration_us > 10000) ? (total_duration_us - 10000) : total_duration_us;
            play_note_for_us((int)score[i].freq, play_us);
            if (play_us < total_duration_us) sleep_us(10000);
        } else {
            sleep_us(total_duration_us);
        }
        i++;
    }
    softToneWrite(PIN_SPK, 0);
    sleep_ms(500);
    stop_playing();
    set_led_color(0,0,0);
    return NULL;
}

// ---------------------------------------------------------------------------
// 初期設定・マップ処理
// ---------------------------------------------------------------------------
void get_local_mac() {
    struct hci_dev_info di;
    if (hci_devinfo(HCI_DEV_ID, &di) < 0) strcpy(my_mac_addr, "00:00:00:00:00:00");
    else ba2str(&di.bdaddr, my_mac_addr);
    printf("[INIT] My MAC: %s\n", my_mac_addr);
}

int compare_nodes(const void *a, const void *b) {
    NodeInfo *nA = (NodeInfo *)a;
    NodeInfo *nB = (NodeInfo *)b;
    return nA->key - nB->key;
}

void determine_role_from_map() {
    select_priority_for_song(current_song_id);
    FILE *fp = fopen(MAP_FILE, "r");
    if (!fp) { my_part_id = 1; return; }
    NodeInfo nodes[100];
    int count = 0;
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        char mac[32]; int k, x, y, z;
        if (sscanf(line, "[%[^]]] Key: %d, Coords: (%d, %d, %d)", mac, &k, &x, &y, &z) == 5) {
            strcpy(nodes[count].mac, mac);
            nodes[count].key = k; 
            nodes[count].x = x; nodes[count].y = y; nodes[count].z = z;
            if (strcasecmp(mac, my_mac_addr) == 0) { my_x = x; my_y = y; my_z = z; }
            count++;
        }
    }
    fclose(fp);
    qsort(nodes, count, sizeof(NodeInfo), compare_nodes);
    
    my_rank = -1;
    for (int i = 0; i < count; i++) {
        if (strcasecmp(nodes[i].mac, my_mac_addr) == 0) { my_rank = i; break; }
    }
    if (my_rank == -1) { my_part_id = 1; my_rank = 0; return; }
    printf("[INIT] I am Key:%d at (%d, %d, %d).\n", nodes[my_rank].key, my_x, my_y, my_z);
}

void cleanup_and_exit(int sig) {
    printf("\n[SYSTEM] Shutting down...\n");
    stop_playing();
    
    // ★LED終了処理: 元のコードに合わせて 0 (消灯)
    softPwmWrite(PIN_LED_R, 0); 
    softPwmWrite(PIN_LED_G, 0); 
    softPwmWrite(PIN_LED_B, 0);
    usleep(50000); 
    
    // Low(0) = 消灯
    pinMode(PIN_LED_R, OUTPUT); digitalWrite(PIN_LED_R, 0);
    pinMode(PIN_LED_G, OUTPUT); digitalWrite(PIN_LED_G, 0);
    pinMode(PIN_LED_B, OUTPUT); digitalWrite(PIN_LED_B, 0);
    
    softToneWrite(PIN_SPK, 0);
    if (device_handle >= 0) hci_le_set_advertise_enable(device_handle, 0, 1000);
    exit(0);
}

// ---------------------------------------------------------------------------
// メインループ
// ---------------------------------------------------------------------------
int main() {
    printf("=== Effect Main (Election + Original LED Logic) ===\n");
    signal(SIGINT, cleanup_and_exit);
    srand(time(NULL)); 

    try_set_realtime_priority();

    if (wiringPiSetupGpio() == -1) return 1;
    pinMode(PIN_SWITCH, INPUT); pullUpDnControl(PIN_SWITCH, PUD_UP);
    pinMode(PIN_EXIT_BUTTON, INPUT); pullUpDnControl(PIN_EXIT_BUTTON, PUD_UP);

    for (int i=0; i<NUM_SENSORS; i++) { pinMode(SENSORS[i], INPUT); pullUpDnControl(SENSORS[i], PUD_DOWN); }
    
    // ★初期化: 0 (消灯)
    softPwmCreate(PIN_LED_R, 0, 100); 
    softPwmCreate(PIN_LED_G, 0, 100); 
    softPwmCreate(PIN_LED_B, 0, 100);
    softToneCreate(PIN_SPK);

    // 起動デモ
    printf("Startup Demo...\n");
    set_led_color(100, 0, 0); sleep_ms(300); 
    set_led_color(0, 100, 0); sleep_ms(300); 
    set_led_color(0, 0, 100); sleep_ms(300); 
    set_led_color(50, 50, 50); sleep_ms(500); 
    set_led_color(0, 0, 0);   sleep_ms(200); 

    system("sudo hciconfig hci0 down"); system("sudo hciconfig hci0 up");
    device_handle = hci_open_dev(HCI_DEV_ID);
    get_local_mac(); determine_role_from_map();

    hci_le_set_scan_parameters(device_handle, 0x01, 0x40, 0x40, 0x00, 0x00, 1000);
    hci_le_set_scan_enable(device_handle, 0x01, 0x00, 1000);

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(device_handle, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    fd_set rfds; struct timeval tv;

    printf("System Ready.\n");

    while (1) {
        // --- ボタン処理 ---
        if (digitalRead(PIN_EXIT_BUTTON) == 0) {
             usleep(20000);
             if (digitalRead(PIN_EXIT_BUTTON) == 0) {
                 printf("\n[EXIT BUTTON] Global STOP\n");
                 for(int k=0; k<5; k++) {
                     send_packet(0, PKT_TYPE_STOP, 0, 0, 0, 0, 0);
                     usleep(50000);
                 }
                 cleanup_and_exit(0);
             }
        }

        // --- センサ処理 ---
        int sensor_hit = 0;
        int trigger_pin = -1;

        if (get_time_ms() - last_trigger_time > 3000) {
            for (int i=0; i<NUM_SENSORS; i++) {
                if (digitalRead(SENSORS[i]) == HIGH) { 
                    sensor_hit = 1; trigger_pin = SENSORS[i]; break; 
                }
            }
        }
        if (kbhit()) { getchar(); sensor_hit = 1; trigger_pin = 999; }

        // ★★★ 選挙への立候補 ★★★
        if (sensor_hit) {
            if (!is_playing && !election_running) {
                election_running = 1;
                election_start_time = get_time_ms();
                my_ticket = rand() % 255; // 自分の強さ(乱数)
                pending_song_id = rand() % 2;
                pending_trigger_pin = trigger_pin;
                
                printf("Candidate! Ticket: %d\n", my_ticket);
                send_packet(0, PKT_TYPE_CANDIDATE, my_ticket, pending_song_id, 0, 0, 0);
            }
        }

        // ★★★ 選挙結果判定 ★★★
        if (election_running) {
            if (get_time_ms() - election_start_time >= ELECTION_WINDOW_MS) {
                // 勝ち残り！ -> 自分がOriginとしてスタート
                election_running = 0;
                static uint8_t local_seq = 0;
                uint16_t global_seq = ((my_rank & 0xFF) << 8) | (++local_seq);
                last_trigger_time = get_time_ms();
                
                select_priority_for_song(pending_song_id);
                printf("\n[WINNER] I am Origin! Ticket:%d\n", my_ticket);

                unsigned long target = get_time_ms() + ORIGIN_START_DELAY;
                
                send_packet(global_seq, PKT_TYPE_START, 0, pending_song_id, my_x, my_y, my_z);

                ScheduleArgs *sch = malloc(sizeof(ScheduleArgs));
                sch->target_time = target; sch->seq = global_seq; sch->dist = 0;
                sch->ox = my_x; sch->oy = my_y; sch->oz = my_z;
                pthread_t th; pthread_create(&th, NULL, scheduled_play_thread, sch); pthread_detach(th);

                if (pending_trigger_pin != 999 && pending_trigger_pin != -1) {
                    int c=0; while(digitalRead(pending_trigger_pin)==HIGH && c<50) {sleep_ms(100);c++;}
                    sleep_ms(2000);
                }
            }
        }

        // --- 受信処理 ---
        FD_ZERO(&rfds); FD_SET(device_handle, &rfds);
        tv.tv_sec = 0; tv.tv_usec = 10000;
        if (select(device_handle + 1, &rfds, NULL, NULL, &tv) > 0) {
            int len = read(device_handle, buf, sizeof(buf));
            if (len > 0) {
                evt_le_meta_event *meta = (void *)(buf + 1 + HCI_EVENT_HDR_SIZE);
                if (meta->subevent == EVT_LE_ADVERTISING_REPORT) {
                    le_advertising_info *info = (le_advertising_info *)(meta->data + 1);
                    char sender_mac[18]; ba2str(&info->bdaddr, sender_mac);
                    if (strcasecmp(sender_mac, my_mac_addr) == 0) continue;

                    int offset = 0;
                    while (offset < info->length) {
                        int dlen = info->data[offset];
                        if (dlen == 0) break;
                        int type = info->data[offset+1];

                        if (type == 0xFF && dlen >= (int)sizeof(EffectPayload)) {
                            EffectPayload *p = (EffectPayload *)&info->data[offset+2];
                            if (p->company_id != MY_COMPANY_ID) { offset += dlen + 1; continue; }

                            // STOP信号
                            if (p->packet_type == PKT_TYPE_STOP) {
                                printf("\n[RECV] STOP Signal!\n");
                                for(int k=0; k<3; k++) {
                                    send_packet(0, PKT_TYPE_STOP, 0, 0, 0, 0, 0);
                                    usleep(50000);
                                }
                                cleanup_and_exit(0);
                            }

                            // ★★★ 選挙ロジック (他者の立候補を受信) ★★★
                            if (p->packet_type == PKT_TYPE_CANDIDATE) {
                                if (election_running) {
                                    // 相手の方が強いチケットを持っている、または同じで相手のランクが高い(IDが小さい)場合
                                    if (p->ticket > my_ticket || (p->ticket == my_ticket && p->rank < my_rank)) {
                                        printf("Lost election to Ticket:%d (My:%d)\n", p->ticket, my_ticket);
                                        election_running = 0; // 降りる
                                    }
                                }
                                offset += dlen + 1;
                                continue;
                            }

                            // ★★★ スタート信号を受信 ★★★
                            if (p->packet_type == PKT_TYPE_START) {
                                // もし選挙中なら、誰かが勝利してスタートしたということなので降りる
                                if (election_running) election_running = 0;

                                int seq = p->seq;
                                if (seq == last_handled_seq) { offset += dlen + 1; continue; }
                                last_handled_seq = seq;

                                select_priority_for_song(p->song_id);
                                int dist = abs(my_x - p->origin_x) + abs(my_y - p->origin_y) + abs(my_z - p->origin_z);
                                int est_hop = dist * 10;
                                int wait = ORIGIN_START_DELAY - est_hop;
                                if (wait < 0) wait = 0;
                                unsigned long target = get_time_ms() + wait;

                                printf("[RECV] START Song:%d Dist:%d\n", p->song_id, dist);

                                if (!is_playing) {
                                    printf("[RELAY] Relaying START...\n");
                                    send_packet(seq, PKT_TYPE_START, 0, p->song_id, p->origin_x, p->origin_y, p->origin_z);
                                    
                                    ScheduleArgs *sch = malloc(sizeof(ScheduleArgs));
                                    sch->target_time = target; sch->seq = seq; sch->dist = dist;
                                    sch->ox = p->origin_x; sch->oy = p->origin_y; sch->oz = p->origin_z;
                                    pthread_t th; pthread_create(&th, NULL, scheduled_play_thread, sch); pthread_detach(th);
                                }
                            }
                        }
                        offset += dlen + 1;
                    }
                }
            }
        }
    }
    return 0;
}
