import mido
import os

# =========================================================
# Song Configuration (楽曲設定)
# =========================================================

# 楽曲リスト
# key: Song ID (Cコードの song_id と対応)
# value: { "parts": { Part ID -> ファイル名 } }
# ※ BPMは自動検出するので設定不要
SONG_LIBRARY = {
    # Song ID 0: ドラクエ序曲
    0: {
        "parts": {
            1: "dq_melody_1.midi",
            2: "dq_melody_2.midi",
            3: "dq_chord_1.midi",
            4: "dq_melody_3.midi",
            5: "dq_chord_2.midi",
            6: "dq_melody_4.midi",
        }
    },

    # Song ID 1: 星に願いを
    1: {
        "parts": {
            1: "star_melody_1.midi",
            2: "star_chord_1.midi",
            3: "star_melody_2.midi",
            4: "star_melody_3.midi",
            5: "star_melody_4.midi",
            6: "star_chord_2.midi",
            7: "star_chord_3.midi",
        }
    },

    # Song ID 2: 輝く未来（BPM 100）
    2: {
        "parts": {
            1: "light_melody_1.midi",
            2: "light_chord_1.midi",
            3: "light_melody_2.midi",
            4: "light_melody_3.midi",
            5: "light_chord_2.midi",
            6: "light_melody_4.midi",
        }
    },

    # Song ID 3: All I Want for Christmas Is You（BPM 150）
    3: {
        "parts": {
            1: "c_melody_1.midi",
            2: "c_chord_1.midi",
            3: "c_melody_2.midi",
            4: "c_chord_2.midi",
            5: "c_melody_3.midi",
            6: "c_chord_3.midi",
        }
    },
}

# 再生速度倍率 (1.0 = 原曲通り)
PLAYBACK_SPEED = 1.0
OUTPUT_FILENAME = "music_data.h"

# =========================================================
# Conversion Logic
# =========================================================

def note_to_freq(note_number):
    """MIDIノート番号を周波数(Hz)に変換"""
    return 440.0 * (2.0 ** ((note_number - 69.0) / 12.0))

def get_bpm_from_file(filepath):
    """MIDIファイルから最初のBPMを取得"""
    if not os.path.exists(filepath):
        return 120.0

    try:
        mid = mido.MidiFile(filepath)
        for track in mid.tracks:
            for msg in track:
                if msg.type == 'set_tempo':
                    return mido.tempo2bpm(msg.tempo)
        return 120.0
    except:
        return 120.0

def parse_midi_file(filepath):
    """MIDIを解析して (freq, duration_ms) のリストを返す"""
    if not os.path.exists(filepath):
        print(f"[WARN] File not found: {filepath}")
        return []

    try:
        mid = mido.MidiFile(filepath)
        notes = []
        active_note = None

        for msg in mid:
            delta_time = msg.time * (1.0 / PLAYBACK_SPEED)

            if delta_time > 0:
                duration_ms = delta_time * 1000.0
                if active_note:
                    notes.append((active_note, duration_ms))
                else:
                    notes.append((0, duration_ms))  # rest

            if msg.type == 'note_on' and msg.velocity > 0:
                active_note = note_to_freq(msg.note)
            elif msg.type == 'note_off' or (msg.type == 'note_on' and msg.velocity == 0):
                active_note = None

        return notes
    except Exception as e:
        print(f"[ERROR] Failed to parse {filepath}: {e}")
        return []

def generate_c_header():
    print(f"Generating {OUTPUT_FILENAME}...")

    c_code = """#ifndef MUSIC_DATA_H
#define MUSIC_DATA_H

#include <stddef.h>

typedef struct {
    float freq;
    float duration_ms;
} Note;

"""

    analyzed_songs = {}

    # -----------------------------------------------------
    # 1. MIDI解析 & Note配列生成
    # -----------------------------------------------------
    for song_id, song_info in SONG_LIBRARY.items():
        mapping = song_info["parts"]
        print(f"Processing Song ID {song_id}...")

        part1 = mapping.get(1)
        base_bpm = get_bpm_from_file(part1) if part1 else 120.0
        final_bpm = base_bpm * PLAYBACK_SPEED

        analyzed_songs[song_id] = {
            "bpm": final_bpm,
            "part_count": len(mapping),
            "parts": {}
        }

        max_duration = 0
        temp_parts = {}

        for part_id, fname in mapping.items():
            notes = parse_midi_file(fname)
            total = sum(n[1] for n in notes)
            max_duration = max(max_duration, total)
            temp_parts[part_id] = {"file": fname, "notes": notes}

        for part_id, data in temp_parts.items():
            array_name = f"song_{song_id}_part_{part_id}"
            c_code += f"// Song {song_id}, Part {part_id} ({data['file']})\n"
            c_code += f"Note {array_name}[] = {{\n"

            for freq, dur in data["notes"]:
                if dur < 5:
                    continue
                c_code += f"    {{ {freq:.2f}, {dur:.0f} }},\n"

            padding = max_duration - sum(n[1] for n in data["notes"])
            if padding > 10:
                c_code += f"    {{ 0, {padding:.0f} }}, // Padding\n"

            c_code += "    { -1, -1 }\n};\n\n"
            analyzed_songs[song_id]["parts"][part_id] = array_name

    # -----------------------------------------------------
    # 2. 曲数 & パート数テーブル
    # -----------------------------------------------------
    song_ids = sorted(analyzed_songs.keys())
    song_count = len(song_ids)

    c_code += f"#define SONG_COUNT {song_count}\n\n"
    c_code += "static const int SONG_PART_COUNT[SONG_COUNT] = {\n"
    for sid in song_ids:
        c_code += f"    {analyzed_songs[sid]['part_count']}, // Song {sid}\n"
    c_code += "};\n\n"

    # -----------------------------------------------------
    # 3. get_music_part
    # -----------------------------------------------------
    c_code += """Note* get_music_part(int song_id, int part_id) {
    switch(song_id) {
"""
    for sid in song_ids:
        c_code += f"        case {sid}:\n            switch(part_id) {{\n"
        for pid, arr in analyzed_songs[sid]["parts"].items():
            c_code += f"                case {pid}: return {arr};\n"
        first = analyzed_songs[sid]["parts"].get(1, "NULL")
        c_code += f"                default: return {first};\n"
        c_code += "            }\n"
    c_code += """        default: return NULL;
    }
}
"""

    # -----------------------------------------------------
    # 4. BPM
    # -----------------------------------------------------
    c_code += """float get_song_bpm(int song_id) {
    switch(song_id) {
"""
    for sid in song_ids:
        c_code += f"        case {sid}: return {analyzed_songs[sid]['bpm']:.1f};\n"
    c_code += """        default: return 120.0;
    }
}
"""

    # -----------------------------------------------------
    # 5. パート数（★今回の本題）
    # -----------------------------------------------------
    c_code += """
static inline int get_part_count(int song_id) {
    if (song_id < 0 || song_id >= SONG_COUNT) return 1;
    return SONG_PART_COUNT[song_id];
}

#endif
"""

    with open(OUTPUT_FILENAME, "w") as f:
        f.write(c_code)

    print("Done.")

if __name__ == "__main__":
    generate_c_header()
