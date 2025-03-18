#!/usr/bin/env python3

import sys
import os
import time
import math
import re
import subprocess
import shutil
import threading
import signal

#
# ------------------ CONFIG ------------------
#

MEASURE_DURATION = 3.0         # seconds for each of hz/bw/delay
TABLE_HEIGHT_MARGIN = 9        # subtract some rows from terminal height
TABLE_WIDTH_MARGIN  = 0        # set to 0 so we use full terminal width

# Minimal widths for columns
MIN_TOPIC_WIDTH   = 12
MIN_TYPE_WIDTH    = 15
MIN_PUB_WIDTH     = 5
MIN_SUB_WIDTH     = 5
MIN_HZ_WIDTH      = 6
MIN_BW_WIDTH      = 8
MIN_DELAY_WIDTH   = 8
MIN_SCANNED_WIDTH = 8

#
# ------------------ GLOBAL SHARED DATA ------------------
#

data_cache_lock = threading.Lock()

# data_cache is a list of dict:
# [
#   {
#     "topic": str,
#     "type": str,
#     "pub": int,
#     "sub": int,
#     "hz": float,
#     "bw": float,
#     "delay": float,
#     "scanned": bool
#   },
#   ...
# ]
data_cache = []

#
# ------------------ HELPER: If a numeric cell is 0, show "" instead ------------------
#

def format_cell(value):
    """
    Returns an empty string if value is 0 or 0.0, 
    otherwise returns a string representation.
    We'll apply this to #pub/#sub/hz/bw/delay.
    """
    # If it's numeric zero, return empty
    if isinstance(value, (int, float)) and value == 0:
        return ""
    # else we do normal formatting if it's float?
    if isinstance(value, float):
        return f"{value:.2f}"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, str):
        return value
    return str(value)

#
# ------------------ SUBPROCESS + PARSERS ------------------
#

def run_command_for_seconds(cmd_list, duration=3.0):
    """
    Spawn 'cmd_list', read stdout for 'duration' seconds, then send SIGINT.
    We'll parse the lines afterwards.
    We'll log debug prints + exceptions.
    """
    print(f"[DEBUG] Starting command: {' '.join(cmd_list)} for ~{duration}s")
    try:
        proc = subprocess.Popen(
            cmd_list,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            universal_newlines=True
        )
    except Exception as e:
        print(f"[DEBUG] Exception spawning {cmd_list}: {e}")
        return []

    start_time = time.time()
    lines = []
    try:
        while True:
            line = proc.stdout.readline()
            if line:
                lines.append(line.rstrip('\n'))
            if (time.time() - start_time) > duration:
                break
            if proc.poll() is not None:  # process ended
                break

        # Instead of terminate(), we do SIGINT so ros2 topic hz/bw/delay
        # prints final summary lines.
        print(f"[DEBUG] Sending SIGINT to {cmd_list}")
        proc.send_signal(signal.SIGINT)

        try:
            proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            print(f"[DEBUG] {cmd_list} ignoring SIGINT; forcing kill.")
            proc.kill()

    except Exception as e:
        print(f"[DEBUG] Exception reading {cmd_list}: {e}")
        # ensure kill
        try:
            proc.kill()
        except:
            pass

    print(f"[DEBUG] Finished command: {' '.join(cmd_list)}")
    return lines

def parse_topic_info(lines):
    """
    For `ros2 topic info`, parse lines for:
      Type: ...
      Publisher count: X
      Subscriber count: Y
    Return (msg_type_str, pub_count, sub_count).
    """
    pub_count, sub_count = 0, 0
    msg_type_str = ""
    type_re = re.compile(r"Type:\s*(\S+)")
    pub_re  = re.compile(r"Publisher count:\s*(\d+)")
    sub_re  = re.compile(r"Subscriber count:\s*(\d+)")

    for ln in lines:
        tm = type_re.search(ln)
        if tm:
            msg_type_str = tm.group(1)
        pm = pub_re.search(ln)
        if pm:
            pub_count = int(pm.group(1))
        sm = sub_re.search(ln)
        if sm:
            sub_count = int(sm.group(1))

    return (msg_type_str, pub_count, sub_count)

def parse_hz_output(lines):
    """
    For `ros2 topic hz`, parse last line with 'average rate: X.XXXX'
    Return float or 0.0 if not found.
    """
    hz = 0.0
    pat = re.compile(r"average rate:\s*([\d\.]+)")
    for ln in reversed(lines):
        m = pat.search(ln)
        if m:
            try:
                hz = float(m.group(1))
            except:
                pass
            break
    return hz

def parse_bw_output(lines):
    """
    For `ros2 topic bw`, lines like '12.34 B/s', '1.23 KiB/s', '0.01 MiB/s'
    Convert to B/s float, or 0.0 if not found.
    """
    bw_bps = 0.0
    pat = re.compile(r"([\d\.]+)\s*(B|KiB|MiB)/s")
    for ln in reversed(lines):
        m = pat.search(ln)
        if m:
            try:
                val = float(m.group(1))
            except:
                val = 0.0
            unit = m.group(2)
            if unit == "B":
                bw_bps = val
            elif unit == "KiB":
                bw_bps = val * 1024
            elif unit == "MiB":
                bw_bps = val * 1024 * 1024
            break
    return bw_bps

def parse_delay_output(lines):
    """
    For `ros2 topic delay`, parse 'average delay: X.XXXX'
    Return float or 0.0 if not found.
    """
    delay_s = 0.0
    pat = re.compile(r"average delay:\s*([\d\.]+)")
    for ln in reversed(lines):
        m = pat.search(ln)
        if m:
            try:
                delay_s = float(m.group(1))
            except:
                pass
            break
    return delay_s

def get_topic_list():
    """
    Return a list of topic names from `ros2 topic list`.
    """
    try:
        out = subprocess.check_output(["ros2","topic","list"], text=True)
        lines = [ln.strip() for ln in out.splitlines() if ln.strip()]
        return lines
    except Exception as e:
        print(f"[DEBUG] Exception in get_topic_list: {e}")
        return []

#
# ------------------ PHASE 1: gather type/pub/sub quickly ------------------
#

def gather_types_and_pubsub():
    """
    For each topic, run `ros2 topic info` quickly to get:
      - msg type
      - #pub
      - #sub
    Then store in data_cache with scanned=False, and hz/bw/delay=0.0
    """
    global data_cache
    topics = get_topic_list()
    tmp_list = []
    for t in topics:
        try:
            info_out = subprocess.check_output(["ros2","topic","info",t],
                                               text=True, stderr=subprocess.STDOUT)
            info_lines = info_out.splitlines()
        except subprocess.CalledProcessError as e:
            print(f"[DEBUG] Exception in gather_types_and_pubsub for {t}: {e}")
            info_lines = e.output.splitlines() if e.output else []

        msg_type_str, pubc, subc = parse_topic_info(info_lines)
        tmp_list.append({
            "topic": t,
            "type": msg_type_str,
            "pub": pubc,
            "sub": subc,
            "hz": 0.0,
            "bw": 0.0,
            "delay": 0.0,
            "scanned": False
        })

    with data_cache_lock:
        data_cache = tmp_list[:]

#
# ------------------ PHASE 2: measure hz/bw/delay in background ------------------
#

def measure_hz_bw_delay(topic):
    """
    For a single topic, run `ros2 topic hz`, `ros2 topic bw`, `ros2 topic delay`.
    Return a dict: { "hz", "bw", "delay" }
    """
    print(f"[DEBUG] measure_hz_bw_delay({topic}) started.")
    hz_lines = run_command_for_seconds(["ros2","topic","hz",topic], MEASURE_DURATION)
    hz_val = parse_hz_output(hz_lines)

    bw_lines = run_command_for_seconds(["ros2","topic","bw",topic], MEASURE_DURATION)
    bw_val = parse_bw_output(bw_lines)

    delay_lines = run_command_for_seconds(["ros2","topic","delay",topic], MEASURE_DURATION)
    delay_val = parse_delay_output(delay_lines)

    print(f"[DEBUG] measure_hz_bw_delay({topic}) => hz={hz_val}, bw={bw_val}, delay={delay_val}")
    return {"hz": hz_val, "bw": bw_val, "delay": delay_val}

def background_scanner():
    """
    The background thread that, for each entry in data_cache,
    measures Hz/BW/Delay, sets scanned=True. 
    (We already have type/pub/sub from gather_types_and_pubsub().)
    """
    global data_cache
    while True:
        with data_cache_lock:
            unscanned = [d for d in data_cache if not d["scanned"]]
        if not unscanned:
            # no more un-scanned topics
            print("[DEBUG] background_scanner: all topics scanned.")
            break

        entry = unscanned[0]
        topic_name = entry["topic"]
        print(f"[DEBUG] background_scanner: measuring {topic_name}")
        try:
            measures = measure_hz_bw_delay(topic_name)
        except Exception as e:
            print(f"[DEBUG] Exception measuring {topic_name}: {e}")
            measures = {"hz":0.0, "bw":0.0, "delay":0.0}

        # store
        with data_cache_lock:
            for row in data_cache:
                if row["topic"] == topic_name:
                    row["hz"]     = measures["hz"]
                    row["bw"]     = measures["bw"]
                    row["delay"]  = measures["delay"]
                    row["scanned"] = True
                    print(f"[DEBUG] Marked {topic_name} as scanned.")
                    break

#
# ------------------ ASCII TABLE PRINTING ------------------
#

def draw_sep_line(widths, max_width):
    line = "+"
    for w in widths:
        line += "-"*w + "+"
    if len(line) > max_width:
        if max_width >= 1:
            return line[:max_width-1] + "+"
        else:
            return line[:max_width]
    return line

def draw_data_line(cells, widths, max_width):
    line = ""
    for i,w in enumerate(widths):
        text_area = max(1, w-1)  # space after '|'
        val = cells[i][:text_area].ljust(text_area)
        line += "| " + val
    line += "|"
    if len(line) > max_width:
        if max_width >= 1:
            return line[:max_width-1] + "|"
        else:
            return line[:max_width]
    return line

def print_table(data_list, page_idx, max_rows, term_cols):
    """
    Columns: 
     1) Topic
     2) Msg Type
     3) #Pub
     4) #Sub
     5) Hz
     6) BW
     7) Delay
     8) Scanned?
    We'll blank out numeric zero => "".
    """
    # figure out widths
    max_topic_len = max(MIN_TOPIC_WIDTH, max(len(d["topic"]) for d in data_list) if data_list else MIN_TOPIC_WIDTH)
    max_type_len  = max(MIN_TYPE_WIDTH, max(len(d["type"])  for d in data_list) if data_list else MIN_TYPE_WIDTH)

    col_topic  = max_topic_len
    col_type   = max_type_len
    col_pub    = MIN_PUB_WIDTH
    col_sub    = MIN_SUB_WIDTH
    col_hz     = MIN_HZ_WIDTH
    col_bw     = MIN_BW_WIDTH
    col_delay  = MIN_DELAY_WIDTH
    col_scan   = MIN_SCANNED_WIDTH

    display_w = max(10, term_cols)  # no margin

    col_widths = [col_topic, col_type, col_pub, col_sub, col_hz, col_bw, col_delay, col_scan]

    # top
    top_line = draw_sep_line(col_widths, display_w)
    print(top_line)

    # header
    hdr_cells = ["Topic","Msg Type","Pub","Sub","Hz","BW","Delay","Scanned?"]
    hdr_line  = draw_data_line(hdr_cells, col_widths, display_w)
    print(hdr_line)

    # mid
    mid_line = draw_sep_line(col_widths, display_w)
    print(mid_line)

    # slice data
    start_idx = page_idx * max_rows
    end_idx   = start_idx + max_rows
    visible   = data_list[start_idx:end_idx]

    for row in visible:
        # we blank out numeric zero
        cell_topic = row["topic"]
        cell_type  = row["type"] or ""
        cell_pub   = format_cell(row["pub"])
        cell_sub   = format_cell(row["sub"])
        cell_hz    = format_cell(row["hz"])
        cell_bw    = format_cell(row["bw"])
        cell_delay = format_cell(row["delay"])
        cell_scan  = "*" if row["scanned"] else ""

        row_vals = [
            cell_topic,
            cell_type,
            cell_pub,
            cell_sub,
            cell_hz,
            cell_bw,
            cell_delay,
            cell_scan,
        ]
        ln = draw_data_line(row_vals, col_widths, display_w)
        print(ln)

    # bottom
    bot_line = draw_sep_line(col_widths, display_w)
    print(bot_line)

#
# ------------------ MAIN ------------------
#

def clear_screen():
    sys.stdout.write('\033[2J\033[H')
    sys.stdout.flush()

def main():
    import rclpy
    rclpy.init()

    print("[DEBUG] Gathering topic list and message types (#pub, #sub) upfront...")
    gather_types_and_pubsub()

    print("[DEBUG] Starting background thread for Hz/BW/Delay measurement.")
    worker = threading.Thread(target=background_scanner, daemon=True)
    worker.start()

    # fix usage: size.columns => width, size.lines => height
    try:
        size = shutil.get_terminal_size()
        term_cols = size.columns  # the actual width
        term_rows = size.lines    # the actual height
    except:
        term_rows, term_cols = (24,80)

    print(f"[DEBUG] Detected terminal size lines={term_rows}, columns={term_cols} (width).")

    table_height = max(1, term_rows - TABLE_HEIGHT_MARGIN)
    max_data_rows = max(1, table_height - 2)
    page_index = 0

    print("ROS 2 Topic Monitor (Msg Type, #Pub/#Sub first, then measure Hz/BW/Delay in background).")
    print("Columns: [Topic, Msg Type, #Pub, #Sub, Hz, BW, Delay, Scanned?]")
    print("'*' => done measuring. Zero numeric => cell is blank.")
    print("Commands:")
    print("  r = redraw table (partial/final data)")
    print("  p = previous page")
    print("  n = next page")
    print("  q = quit\n")

    while True:
        cmd = input("Command [r=refresh, p=prev, n=next, q=quit] > ").strip()
        clear_screen()

        with data_cache_lock:
            total_items = len(data_cache)

        if cmd == 'q':
            print("[DEBUG] Quitting main loop.")
            break
        elif cmd in ('r',''):
            # redraw
            with data_cache_lock:
                total_pages = max(1, math.ceil(total_items / max_data_rows))
                if page_index >= total_pages:
                    page_index = total_pages - 1
                print_table(data_cache, page_index, max_data_rows, term_cols)
                print(f"\nTopics: {total_items}  Page {page_index+1}/{total_pages}.")
                print("Press 'p' or 'n' to change pages, 'r' to redraw, 'q' to quit.")
        elif cmd == 'p':
            page_index -= 1
            if page_index < 0:
                page_index = 0
            with data_cache_lock:
                total_pages = max(1, math.ceil(total_items / max_data_rows))
                print_table(data_cache, page_index, max_data_rows, term_cols)
                print(f"\nTopics: {total_items}  Page {page_index+1}/{total_pages}.")
        elif cmd == 'n':
            with data_cache_lock:
                total_pages = max(1, math.ceil(total_items / max_data_rows))
                page_index += 1
                if page_index >= total_pages:
                    page_index = total_pages - 1
                print_table(data_cache, page_index, max_data_rows, term_cols)
                print(f"\nTopics: {total_items}  Page {page_index+1}/{total_pages}.")
        else:
            print("Unknown command. Use 'r', 'p', 'n', or 'q'.")

    rclpy.shutdown()

if __name__ == "__main__":
    main()
