// effect_main_lightweight_sync.c
// Lightweight sync improvements:
// - ORIGIN_COMPENSATION_MS adjustable via env
// - scheduled_play_thread + music thread attempt SCHED_FIFO
// - tighter busy-wait final path for better ms-level precision
// - uses CLOCK_MONOTONIC_RAW and nanosleep-based sleeps

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

// Hardware control
#include <wiringPi.h>
#include <softPwm.h>
#include <softTone.h>

// Bluetooth
#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

// Music data
#include "music_data.h"

// --- Settings ---
#define MAP_FILE "cube_map_dummy.txt"
#define HCI_DEV_ID 0
#define CURRENT_SONG_ID 0

#define PIN_SWITCH  17
#define PIN_SPK     27
#define PIN_LED_R   12
#define PIN_LED_G   13
#define PIN_LED_B   19
const int SENSORS[] = {0, 5, 6, 26};
#define NUM_SENSORS 4

#define MY_COMPANY_ID 0xFFFF
#define START_DELAY_MS 2000

// Default compensation: user found -5ms works well; keep as default but allow env override.
static int ORIGIN_COMPENSATION_MS = -5;

// Payload (binary, manufacturer data)
typedef struct __attribute__((packed)) {
    uint16_t company_id;
    uint16_t seq;
    uint8_t  hop;
    uint32_t delay_ms;
} EffectPayload;

typedef struct {
    char mac[18];
    int x,y,z;
} NodeInfo;

const int PART_PRIORITY[] = {1,5,3,2,4,6,7,8,9,10,11,12};
#define PRIORITY_LEN 12

// Globals
int device_handle = -1;
char my_mac_addr[18] = {0};
int my_part_id = 1;
int my_rank = 0;
int my_x = 0, my_y = 0, my_z = 0;

static volatile int is_playing = 0;
static unsigned long last_trigger_time = 0;
static unsigned long last_recv_time = 0;
static int last_handled_seq = -1;

// ---------------------------------------------------------------------------
// high precision timer & sleep helpers
// ---------------------------------------------------------------------------
static inline unsigned long get_time_ms_raw() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    return (unsigned long)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}
#define get_time_ms get_time_ms_raw

static void sleep_ms(unsigned long ms) {
    if (ms == 0) return;
    struct timespec req = { (time_t)(ms/1000), (long)((ms%1000) * 1000000L) };
    nanosleep(&req, NULL);
}
static void sleep_us(unsigned long us) {
    if (us == 0) return;
    struct timespec req = { (time_t)(us/1000000), (long)((us%1000000) * 1000L) };
    nanosleep(&req, NULL);
}

// attempt to set current thread to realtime SCHED_FIFO at given priority (best-effort)
static void try_thread_realtime(pthread_t thr, int prio) {
    struct sched_param param;
    param.sched_priority = prio;
    int ret = pthread_setschedparam(thr, SCHED_FIFO, &param);
    if (ret != 0) {
        // best-effort: ignore failure silently (require root or CAP_SYS_NICE)
        // Uncomment below for debugging:
        // fprintf(stderr, "[WARN] pthread_setschedparam failed: %s\n", strerror(ret));
    }
}

// small helper to set current thread realtime with moderate priority
static void set_self_realtime_moderate() {
    try_thread_realtime(pthread_self(), 60);
}

// ---------------------------------------------------------------------------
// kbhit
// ---------------------------------------------------------------------------
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
    if (ch != EOF) { ungetc(ch, stdin); return 1; }
    return 0;
}

// ---------------------------------------------------------------------------
// Effects: LED, note playback
// ---------------------------------------------------------------------------
void set_led_color(int r, int g, int b) {
    softPwmWrite(PIN_LED_R, 100 - r);
    softPwmWrite(PIN_LED_G, 100 - g);
    softPwmWrite(PIN_LED_B, 100 - b);
}

const int RAINBOW_COLORS[12][3] = {
    {100,0,0},{100,50,0},{100,100,0},{50,100,0},
    {0,100,0},{0,100,50},{0,100,100},{0,50,100},
    {0,0,100},{50,0,100},{100,0,100},{100,0,50}
};

typedef struct { int hop; unsigned long start_time_ms; } LedArgs;

void *led_thread_func(void *arg) {
    LedArgs *args = (LedArgs*)arg;
    int hop = args->hop;
    unsigned long base_time = args->start_time_ms;
    free(arg);
    int start_color_idx = (my_rank * 3) % 12;
    int hop_shift = hop * -2;
    float bpm = get_song_bpm(CURRENT_SONG_ID);
    int beat_ms = (int)(60000.0 / (bpm>0?bpm:120.0));

    unsigned long anim_start = get_time_ms();
    while (is_playing || (get_time_ms() - anim_start < 2000)) {
        unsigned long now = get_time_ms();
        unsigned long elapsed = (now > base_time) ? (now - base_time) : 0;
        int beat_count = elapsed / beat_ms;
        int color_idx = (start_color_idx + hop_shift + beat_count) % 12;
        while (color_idx < 0) color_idx += 12;
        set_led_color(RAINBOW_COLORS[color_idx][0], RAINBOW_COLORS[color_idx][1], RAINBOW_COLORS[color_idx][2]);
        sleep_ms(20);
    }
    set_led_color(0,0,0);
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

void *music_thread_func(void *arg) {
    int seq = *(int*)arg;
    free(arg);

    // try elevate this thread to moderate RT priority (best-effort)
    set_self_realtime_moderate();

    if (is_playing) return NULL;
    is_playing = 1;

    Note *score = get_music_part(CURRENT_SONG_ID, my_part_id);
    if (!score) { is_playing = 0; return NULL; }

    printf("   [EFFECT] ♪ Music Play NOW! (Part:%d, Seq:%d)\n", my_part_id, seq);

    // start led for local
    pthread_t th_led;
    LedArgs *la = malloc(sizeof(LedArgs));
    la->hop = 0; la->start_time_ms = get_time_ms();
    pthread_create(&th_led, NULL, led_thread_func, la);
    pthread_detach(th_led);

    const int GAP_US = 10000;
    int i = 0;
    while (score[i].freq != -1) {
        unsigned long total_us = (unsigned long)(score[i].duration_ms * 1000.0f);
        if (score[i].freq > 0) {
            unsigned long play_us = (total_us > GAP_US) ? (total_us - GAP_US) : total_us;
            play_note_for_us((int)score[i].freq, play_us);
            if (play_us < total_us) sleep_us(GAP_US);
        } else {
            sleep_us(total_us);
        }
        i++;
    }

    printf("   [EFFECT] ♪ Music End\n");
    softToneWrite(PIN_SPK, 0);
    sleep_ms(500);
    is_playing = 0;
    return NULL;
}

typedef struct { unsigned long target_time; int seq; int hop; } ScheduleArgs;

// scheduled play uses adaptive sleeping and a short busy-wait at the end.
// additionally, it attempts to set realtime priority for the playback thread.
void *scheduled_play_thread(void *arg) {
    ScheduleArgs *sa = (ScheduleArgs*)arg;
    unsigned long target = sa->target_time;
    int seq = sa->seq;
    int hop = sa->hop;
    free(sa);

    // Try to increase this thread priority slightly to reduce jitter when approaching start.
    set_self_realtime_moderate();

    // wait until target: coarse sleep then tight busy wait in last few ms
    while (1) {
        unsigned long now = get_time_ms();
        long diff = (long)(target - now);
        if (diff <= 0) break;
        if (diff > 15) {
            // sleep most of remaining time but leave margin
            sleep_ms((unsigned long)(diff - 12));
        } else if (diff > 4) {
            // smaller sleeps in final window
            sleep_ms((unsigned long)(diff - 3));
        } else {
            // final tight busy-wait (diff <= 4 ms)
            while ((long)(target - get_time_ms()) > 0) {
                // minimal busy work
                ;
            }
            break;
        }
    }

    // Start playback as thread (so music_thread_func can also try to set RT priority)
    if (is_playing) return NULL;

    // create music thread (so that music_thread_func sets its own RT priority)
    pthread_t th;
    int *arg_seq = malloc(sizeof(int));
    *arg_seq = seq;
    pthread_create(&th, NULL, music_thread_func, arg_seq);
    pthread_detach(th);

    // start LED with hop
    pthread_t th_led;
    LedArgs *la = malloc(sizeof(LedArgs));
    la->hop = hop; la->start_time_ms = get_time_ms();
    pthread_create(&th_led, NULL, led_thread_func, la);
    pthread_detach(th_led);

    return NULL;
}

// ---------------------------------------------------------------------------
// BLE sending (advertising) - blocking send like before
// ---------------------------------------------------------------------------
void send_binary_packet(int seq, int hop, uint32_t delay_ms) {
    hci_le_set_advertise_enable(device_handle, 0, 1000);
    hci_le_set_scan_enable(device_handle, 0x00, 0x00, 1000);

    EffectPayload payload;
    payload.company_id = MY_COMPANY_ID;
    payload.seq = (uint16_t)seq;
    payload.hop = (uint8_t)hop;
    payload.delay_ms = delay_ms;

    uint8_t adv_data[31]; memset(adv_data,0,31);
    int idx = 0;
    adv_data[idx++] = 2; adv_data[idx++] = 0x01; adv_data[idx++] = 0x06;
    int payload_len = sizeof(payload);
    adv_data[idx++] = payload_len + 1;
    adv_data[idx++] = 0xFF;
    memcpy(&adv_data[idx], &payload, payload_len);
    idx += payload_len;

    struct hci_request rq;
    le_set_advertising_data_cp data_cp;
    memset(&data_cp,0,sizeof(data_cp));
    data_cp.length = idx;
    memcpy(data_cp.data, adv_data, 31);

    memset(&rq,0,sizeof(rq));
    rq.ogf = OGF_LE_CTL; rq.ocf = OCF_LE_SET_ADVERTISING_DATA;
    rq.cparam = &data_cp; rq.clen = LE_SET_ADVERTISING_DATA_CP_SIZE;
    rq.rparam = NULL; rq.rlen = 0; rq.event = EVT_CMD_COMPLETE;
    hci_send_req(device_handle, &rq, 1000);

    le_set_advertising_parameters_cp params_cp;
    memset(&params_cp,0,sizeof(params_cp));
    params_cp.min_interval = htobs(0x0040); params_cp.max_interval = htobs(0x0040);
    params_cp.advtype = 0x00; params_cp.chan_map = 0x07; params_cp.filter = 0x00;
    memset(&rq,0,sizeof(rq));
    rq.ogf = OGF_LE_CTL; rq.ocf = OCF_LE_SET_ADVERTISING_PARAMETERS;
    rq.cparam = &params_cp; rq.clen = LE_SET_ADVERTISING_PARAMETERS_CP_SIZE;
    rq.rparam = NULL; rq.rlen = 0; rq.event = EVT_CMD_COMPLETE;
    hci_send_req(device_handle, &rq, 1000);

    hci_le_set_advertise_enable(device_handle, 1, 1000);
    sleep_ms(100);
    hci_le_set_advertise_enable(device_handle, 0, 1000);

    hci_le_set_scan_enable(device_handle, 0x01, 0x00, 1000);
}

typedef struct { int s; int h; uint32_t d; } SendThreadArgs;
void *send_thread_func(void *arg) {
    SendThreadArgs *a = (SendThreadArgs*)arg;
    send_binary_packet(a->s, a->h, a->d);
    free(a);
    return NULL;
}

// ---------------------------------------------------------------------------
// init / map
// ---------------------------------------------------------------------------
void get_local_mac() {
    struct hci_dev_info di;
    if (hci_devinfo(HCI_DEV_ID, &di) < 0) strcpy(my_mac_addr, "00:00:00:00:00:00");
    else ba2str(&di.bdaddr, my_mac_addr);
    printf("[INIT] My MAC: %s\n", my_mac_addr);
}

int compare_nodes(const void *a, const void *b) {
    NodeInfo *A = (NodeInfo*)a; NodeInfo *B = (NodeInfo*)b;
    if (A->x != B->x) return A->x - B->x;
    if (A->y != B->y) return A->y - B->y;
    return A->z - B->z;
}

void determine_role_from_map() {
    FILE *fp = fopen(MAP_FILE, "r");
    if (!fp) { printf("[WARN] Map not found. Defaults.\n"); my_part_id = 1; return; }

    NodeInfo nodes[100];
    int count = 0;
    char line[256];

    while (fgets(line, sizeof(line), fp)) {
        char mac[32]; int k, x, y, z;
        if (sscanf(line, "[%[^]]] Key: %d, Coords: (%d, %d, %d)", mac, &k, &x, &y, &z) == 5) {
            strcpy(nodes[count].mac, mac);
            nodes[count].x = x; nodes[count].y = y; nodes[count].z = z;
            if (strcasecmp(mac, my_mac_addr) == 0) {
                my_x = x; my_y = y; my_z = z;
            }
            count++;
        }
    }
    fclose(fp);

    if (count == 0) {
        printf("[WARN] No nodes in map.\n");
        my_part_id = 1; my_rank = 0;
        return;
    }

    // sort by coordinates to produce deterministic physical order
    qsort(nodes, count, sizeof(NodeInfo), compare_nodes);

    // find my rank
    my_rank = -1;
    for (int i = 0; i < count; i++) {
        if (strcasecmp(nodes[i].mac, my_mac_addr) == 0) {
            my_rank = i;
            break;
        }
    }
    if (my_rank == -1) {
        printf("[WARN] MAC not found in map.\n");
        my_part_id = 1; my_rank = 0;
        return;
    }

    // determine available parts for this song (filter PART_PRIORITY by total_parts)
    int total_parts_in_song = get_part_count(CURRENT_SONG_ID);
    if (total_parts_in_song <= 0) total_parts_in_song = 1;

    int allowed[PRIORITY_LEN + 16];
    int allowed_count = 0;

    // first, add PART_PRIORITY entries that are valid (<= total_parts_in_song)
    for (int i = 0; i < PRIORITY_LEN; i++) {
        int p = PART_PRIORITY[i];
        if (p >= 1 && p <= total_parts_in_song) {
            // avoid duplicates
            int dup = 0;
            for (int j = 0; j < allowed_count; j++) if (allowed[j] == p) { dup = 1; break; }
            if (!dup) allowed[allowed_count++] = p;
        }
    }

    // if some parts still not included (e.g. PART_PRIORITY omitted them), append remaining parts in numeric order
    for (int p = 1; p <= total_parts_in_song; p++) {
        int found = 0;
        for (int j = 0; j < allowed_count; j++) if (allowed[j] == p) { found = 1; break; }
        if (!found) allowed[allowed_count++] = p;
    }

    // defensive: if somehow allowed_count==0
    if (allowed_count == 0) {
        my_part_id = 1;
    } else {
        my_part_id = allowed[ my_rank % allowed_count ];
    }

    printf("[INIT] Rank: %d/%d (ColorBase), Part: %d (Sound)\n", my_rank + 1, count, my_part_id);
}

int is_neighbor(const char *target_mac) {
    FILE *fp = fopen(MAP_FILE, "r");
    if (!fp) return 0;
    char line[256]; int tx=0,ty=0,tz=0, found=0;
    while (fgets(line, sizeof(line), fp)) {
        char mac[32]; int k,x,y,z;
        if (sscanf(line, "[%[^]]] Key: %d, Coords: (%d, %d, %d)", mac, &k, &x, &y, &z) == 5) {
            if (strcasecmp(mac, target_mac)==0) { tx=x; ty=y; tz=z; found=1; break; }
        }
    }
    fclose(fp);
    if (!found) return 0;
    return (abs(my_x - tx) + abs(my_y - ty) + abs(my_z - tz) == 1);
}

// ---------------------------------------------------------------------------
// main loop
// ---------------------------------------------------------------------------
void cleanup_and_exit(int sig) {
    printf("\n[SYSTEM] Shutting down...\n");
    softPwmWrite(PIN_LED_R, 100);
    softPwmWrite(PIN_LED_G, 100);
    softPwmWrite(PIN_LED_B, 100);
    softToneWrite(PIN_SPK, 0);
    if (device_handle >= 0) hci_le_set_advertise_enable(device_handle, 0, 1000);
    system("sudo hciconfig hci0 noleadv");
    exit(0);
}

int main() {
    // allow override of ORIGIN_COMPENSATION_MS via env (integer)
    char *env = getenv("ORIGIN_COMPENSATION_MS");
    if (env) {
        ORIGIN_COMPENSATION_MS = atoi(env);
        printf("[INIT] ORIGIN_COMPENSATION_MS set to %d ms from ENV\n", ORIGIN_COMPENSATION_MS);
    } else {
        printf("[INIT] ORIGIN_COMPENSATION_MS default %d ms\n", ORIGIN_COMPENSATION_MS);
    }

    printf("=== Effect Main (Lightweight Sync Improvements) ===\n");
    signal(SIGINT, cleanup_and_exit);

    // try to raise overall process scheduler moderately (best-effort)
    struct sched_param p; p.sched_priority = 10;
    if (sched_setscheduler(0, SCHED_OTHER, &p) != 0) {
        // ignore; best-effort
    }

    if (wiringPiSetupGpio() == -1) return 1;
    pinMode(PIN_SWITCH, INPUT); pullUpDnControl(PIN_SWITCH, PUD_UP);
    for (int i=0;i<NUM_SENSORS;i++) { pinMode(SENSORS[i], INPUT); pullUpDnControl(SENSORS[i], PUD_DOWN); }
    softPwmCreate(PIN_LED_R, 100, 100);
    softPwmCreate(PIN_LED_G, 100, 100);
    softPwmCreate(PIN_LED_B, 100, 100);
    softToneCreate(PIN_SPK);

    system("sudo hciconfig hci0 down"); system("sudo hciconfig hci0 up");
    device_handle = hci_open_dev(HCI_DEV_ID);
    if (device_handle < 0) { perror("HCI open failed"); return 1; }

    get_local_mac();
    determine_role_from_map();

    hci_le_set_scan_parameters(device_handle, 0x01, 0x10, 0x10, 0x00, 0x00, 1000);
    hci_le_set_scan_enable(device_handle, 0x01, 0x00, 1000);

    struct hci_filter nf; hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);
    setsockopt(device_handle, SOL_HCI, HCI_FILTER, &nf, sizeof(nf));

    unsigned char buf[HCI_MAX_EVENT_SIZE];
    fd_set rfds; struct timeval tv;

    printf("System Ready.\n");

    while (1) {
        int is_triggered = 0;
        int trigger_pin = -1;

        if (get_time_ms() - last_trigger_time > 3000) {
            for (int i=0;i<NUM_SENSORS;i++) {
                if (digitalRead(SENSORS[i]) == HIGH) { is_triggered = 1; trigger_pin = SENSORS[i]; break; }
            }
        }
        if (kbhit()) { getchar(); is_triggered = 1; trigger_pin = 999; }

        if (is_triggered) {
            if (is_playing) { sleep_ms(100); continue; }

            static int global_seq = 1;
            global_seq = (global_seq % 100) + 1;
            last_trigger_time = get_time_ms();
            last_handled_seq = global_seq;

            printf("\n[TRIGGER] Origin Triggered! (Pin %d)\n", trigger_pin);

            uint32_t delay = START_DELAY_MS;

            // send blocking adv
            send_binary_packet(global_seq, 0, delay);

            // schedule self playback based on send-completion time (+ compensation)
            unsigned long my_target = get_time_ms() + delay + (long)ORIGIN_COMPENSATION_MS;
            printf("[ORIGIN] scheduling self at %lu (now %lu, comp %d)\n", my_target, get_time_ms(), ORIGIN_COMPENSATION_MS);

            ScheduleArgs *sch = malloc(sizeof(ScheduleArgs));
            sch->target_time = my_target; sch->seq = global_seq; sch->hop = 0;
            pthread_t th; pthread_create(&th, NULL, scheduled_play_thread, sch);
            pthread_detach(th);

            if (trigger_pin != 999) {
                int cnt=0; while (digitalRead(trigger_pin) == HIGH && cnt < 50) { sleep_ms(100); cnt++; }
                sleep_ms(2000); last_trigger_time = get_time_ms();
            }
        }

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
                            EffectPayload *payload = (EffectPayload *)&info->data[offset+2];
                            if (payload->company_id != MY_COMPANY_ID) { offset += dlen + 1; continue; }

                            int seq = payload->seq;
                            int hop = payload->hop;
                            uint32_t delay_ms = payload->delay_ms;
                            unsigned long now = get_time_ms();

                            if (seq != last_handled_seq && (now - last_trigger_time > 3000)) {
                                if (!is_playing) {
                                    unsigned long my_target = now + (unsigned long)delay_ms;
                                    printf("[RECV] Delay: %u ms -> Wait until: %lu (from %s)\n",
                                        delay_ms, my_target, sender_mac);

                                    ScheduleArgs *sch = malloc(sizeof(ScheduleArgs));
                                    sch->target_time = my_target; sch->seq = seq; sch->hop = hop + 1;
                                    pthread_t th; pthread_create(&th, NULL, scheduled_play_thread, sch);
                                    pthread_detach(th);
                                }
                                last_handled_seq = seq;
                                last_recv_time = now;

                                if (is_neighbor(sender_mac)) {
                                    uint32_t next_delay = (delay_ms > 150) ? (delay_ms - 50) : 0;
                                    sleep_ms(100);
                                    SendThreadArgs *sa = malloc(sizeof(SendThreadArgs));
                                    sa->s = seq; sa->h = hop + 1; sa->d = next_delay;
                                    pthread_t ths; pthread_create(&ths, NULL, send_thread_func, sa);
                                    pthread_detach(ths);
                                    last_trigger_time = get_time_ms();
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
