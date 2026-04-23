import base64
import io
import json
import os
import threading
from PIL import Image, ImageDraw, ImageFont
import xml.etree.ElementTree as ET
import numpy as np
from evdev import InputDevice, ecodes
import serial
import time
import math
import sys
import traceback


# ---------------------- Config ----------------------
if len(sys.argv) < 2:
    print("Usage: python bluetooth_tulip_extractor.py <GPX_FILE> [--resume|--start-over]")
    sys.exit(1)

GPX_FILE = sys.argv[1]
START_MODE = sys.argv[2] if len(sys.argv) >= 3 else "--resume"

OUTPUT_PATH = "/tmp/eink_note.bmp"
GPS_OUTPUT_PATH = "/tmp/eink_gps.bmp"
TMP_PATH = "/tmp/eink_note_temp.bmp"
TMP_GPS_PATH = "/tmp/eink_gps_temp.bmp"
GPS_LOG_PATH = "gps_track.log"
SYSTEM_LOG_PATH = "/home/ben/system.log"
INPUT_EVENT = "/dev/input/event5"
GPS_DEVICE = "/dev/ttyACM0"
GPS_BAUDRATE = 9600

MOVEMENT_THRESHOLD_KM = 0.005  # ~5 m

last_button_time = 0
DEBOUNCE_MS = 100

gps_port = None
gps_lock = threading.Lock()
state_lock = threading.Lock()

last_latlon = None
total_distance_km = 0.0
heading_deg = 0.0
show_speed = False
current_speed_limit = None
current_speed_kmh = 0.0
waypoint_count = 0
special_wpts = []
special_idx = 0
current_index = 0

session_start_time = time.time()
loaded_elapsed_seconds = 0.0
last_saved_elapsed_seconds = 0.0

ns = {
    'default': 'http://www.topografix.com/GPX/1/1',
    'openrally': 'http://www.openrally.org/xmlschemas/GpxExtensions/v1.0.3'
}

tree = ET.parse(GPX_FILE)
root = tree.getroot()
waypoints = root.findall('default:wpt', ns)
pressed_keys = set()

from time import time as now

font_large = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 90)
font_small = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 30)
font_marker = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 60)

special_tags = [
    'dss', 'ass', 'dz', 'fz', 'wpm', 'wpe', 'wps', 'wpc',
    'wpv', 'wpp', 'wpn', 'checkpoint', 'fn', 'dt', 'ft', 'neutralization', 'speed'
]

ROADBOOK_LOG_DIR = os.path.join(os.path.dirname(GPX_FILE), ".roadbook_logs")
ROADBOOK_BASENAME = os.path.splitext(os.path.basename(GPX_FILE))[0]
ROADBOOK_STATE_PATH = os.path.join(ROADBOOK_LOG_DIR, f"{ROADBOOK_BASENAME}.json")
ROADBOOK_EVENT_LOG_PATH = os.path.join(ROADBOOK_LOG_DIR, f"{ROADBOOK_BASENAME}.csv")


def get_elapsed_seconds():
    return loaded_elapsed_seconds + max(0.0, time.time() - session_start_time)


def format_elapsed(seconds):
    total = int(max(0, seconds))
    hours = total // 3600
    minutes = (total % 3600) // 60
    secs = total % 60
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"


def ensure_log_dir():
    os.makedirs(ROADBOOK_LOG_DIR, exist_ok=True)


def log_system_line(message):
    try:
        os.makedirs(os.path.dirname(SYSTEM_LOG_PATH), exist_ok=True)
        with open(SYSTEM_LOG_PATH, "a") as log:
            log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] {message}\n")
    except Exception:
        pass


def init_event_log():
    ensure_log_dir()
    if not os.path.exists(ROADBOOK_EVENT_LOG_PATH):
        with open(ROADBOOK_EVENT_LOG_PATH, "w") as f:
            f.write("timestamp,event,total_distance_km,current_index,special_idx,elapsed_seconds,details\n")


def append_event(event_type, details=""):
    try:
        ensure_log_dir()
        init_event_log()
        with open(ROADBOOK_EVENT_LOG_PATH, "a") as f:
            f.write(
                f"{time.strftime('%Y-%m-%d %H:%M:%S')},"
                f"{event_type},"
                f"{total_distance_km:.3f},"
                f"{current_index},"
                f"{special_idx},"
                f"{int(get_elapsed_seconds())},"
                f"{str(details).replace(',', ';')}\n"
            )
    except Exception as e:
        log_system_line(f"[ROADBOOK EVENT LOG ERROR] {e}")


def save_run_state(reason="periodic"):
    global last_saved_elapsed_seconds

    try:
        ensure_log_dir()
        state = {
            "gpx_file": GPX_FILE,
            "gpx_basename": os.path.basename(GPX_FILE),
            "saved_at": time.strftime('%Y-%m-%d %H:%M:%S'),
            "reason": reason,
            "total_distance_km": round(total_distance_km, 3),
            "current_index": int(current_index),
            "special_idx": int(special_idx),
            "waypoint_count": int(waypoint_count),
            "heading_deg": round(heading_deg, 1),
            "elapsed_seconds": int(get_elapsed_seconds()),
            "current_speed_kmh": round(current_speed_kmh, 1),
            "current_speed_limit": None if current_speed_limit is None else round(float(current_speed_limit), 1),
            "show_speed": bool(show_speed),
            "last_latlon": list(last_latlon) if last_latlon else None,
            "state_path": ROADBOOK_STATE_PATH,
            "event_log_path": ROADBOOK_EVENT_LOG_PATH,
        }
        tmp = ROADBOOK_STATE_PATH + ".tmp"
        with open(tmp, "w") as f:
            json.dump(state, f, indent=2)
        os.replace(tmp, ROADBOOK_STATE_PATH)
        last_saved_elapsed_seconds = state["elapsed_seconds"]
    except Exception as e:
        log_system_line(f"[ROADBOOK SAVE ERROR] {e}")


def load_run_state():
    global total_distance_km, current_index, special_idx, heading_deg
    global loaded_elapsed_seconds, session_start_time, last_latlon
    global current_speed_limit, show_speed

    if START_MODE == "--start-over":
        if os.path.exists(ROADBOOK_STATE_PATH):
            try:
                os.remove(ROADBOOK_STATE_PATH)
            except Exception as e:
                log_system_line(f"[ROADBOOK RESET STATE REMOVE ERROR] {e}")
        append_event("start_over", "state reset requested")
        return

    if not os.path.exists(ROADBOOK_STATE_PATH):
        append_event("new_run", "no previous state")
        return

    try:
        with open(ROADBOOK_STATE_PATH, "r") as f:
            state = json.load(f)

        total_distance_km = float(state.get("total_distance_km", 0.0))
        current_index = int(state.get("current_index", 0))
        special_idx = int(state.get("special_idx", 0))
        heading_deg = float(state.get("heading_deg", 0.0))
        loaded_elapsed_seconds = float(state.get("elapsed_seconds", 0.0))
        latlon = state.get("last_latlon")
        last_latlon = tuple(latlon) if isinstance(latlon, list) and len(latlon) == 2 else None
        current_speed_limit = state.get("current_speed_limit")
        show_speed = bool(state.get("show_speed", False))
        session_start_time = time.time()
        append_event("resume", f"loaded state from {state.get('saved_at', 'unknown')}")
    except Exception as e:
        log_system_line(f"[ROADBOOK LOAD ERROR] {e}")
        append_event("resume_failed", e)


def open_gps():
    return serial.Serial(GPS_DEVICE, baudrate=GPS_BAUDRATE, timeout=1)


def recover_gps(reason="unknown"):
    global gps_port

    with gps_lock:
        log_system_line(f"[GPS RECOVER] reason={reason}")

        try:
            if gps_port is not None:
                gps_port.close()
        except Exception as e:
            log_system_line(f"[GPS CLOSE ERROR] {e}")

        gps_port = None
        time.sleep(1)

        try:
            gps_port = open_gps()
            flushed = 0
            while gps_port.in_waiting:
                gps_port.readline()
                flushed += 1
            log_system_line(f"[GPS OPEN OK] device={GPS_DEVICE} flushed={flushed}")
            return True
        except Exception as e:
            gps_port = None
            log_system_line(f"[GPS OPEN FAILED] device={GPS_DEVICE} error={e}")
            return False


for wpt in waypoints:
    lat = float(wpt.get("lat"))
    lon = float(wpt.get("lon"))
    for tag in special_tags:
        elem = wpt.find(f'default:extensions/openrally:{tag}', ns)
        if elem is not None:
            waypoint_count += 1

            open_attr = elem.get("open")
            clear_attr = elem.get("clear")
            name = wpt.findtext('default:name', default="(no name)", namespaces=ns)

            if open_attr and clear_attr:
                speed_elem = wpt.find("default:extensions/openrally:speed", ns)
                speed_value = None
                if speed_elem is not None and speed_elem.text:
                    try:
                        speed_value = float(speed_elem.text)
                    except Exception:
                        pass

                special_wpts.append({
                    "index": len(special_wpts) + 1,
                    "lat": lat,
                    "lon": lon,
                    "open": float(open_attr) / 1000,
                    "clear": float(clear_attr) / 1000,
                    "speed": speed_value,
                    "name": name,
                    "tag": tag,
                })
            break

print(f"Total tagged waypoints: {waypoint_count}")

init_event_log()
load_run_state()
save_run_state("startup")

# Open GPS on startup, but don't die if it fails
recover_gps("startup")


# ---------------------- GPS Functions ----------------------
def haversine(lat1, lon1, lat2, lon2):
    R = 6371.0
    phi1, lam1 = math.radians(lat1), math.radians(lon1)
    phi2, lam2 = math.radians(lat2), math.radians(lon2)
    dphi, dlam = phi2 - phi1, phi2 - phi1
    dphi = phi2 - phi1
    dlam = lam2 - lam1
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing(lat1, lon1, lat2, lon2):
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return (math.degrees(math.atan2(x, y)) + 360) % 360


def update_gps():
    global last_latlon, total_distance_km, heading_deg
    global special_idx, current_speed_kmh, current_speed_limit, show_speed, gps_port

    def parse_lat(parts):
        s = parts[3]
        if not s:
            return None
        deg = float(s[:2])
        mins = float(s[2:])
        lat = deg + mins / 60.0
        return -lat if len(parts) > 4 and parts[4] == "S" else lat

    def parse_lon(parts):
        s = parts[5]
        if not s:
            return None
        deg = float(s[:3])
        mins = float(s[3:])
        lon = deg + mins / 60.0
        return -lon if len(parts) > 6 and parts[6] == "W" else lon

    line = None
    current_speed_kmh = 0.0

    if gps_port is None:
        if not recover_gps("gps_port is None in update_gps"):
            return False

    try:
        with gps_lock:
            if gps_port is None:
                return False

            while gps_port.in_waiting:
                candidate = gps_port.readline()
                if candidate.startswith(b"$GPRMC"):
                    line = candidate.decode(errors="ignore").strip()

    except (OSError, serial.SerialException) as e:
        log_system_line(f"[GPS IO ERROR] during read: {e}")
        recover_gps(f"serial read failed: {e}")
        return False
    except Exception as e:
        log_system_line(f"[GPS UNKNOWN READ ERROR] {e}")
        recover_gps(f"unknown read failure: {e}")
        return False

    if not line:
        return False

    try:
        parts = line.split(",")

        if not (len(parts) > 7 and parts[2] == "A"):
            return False

        try:
            speed_knots = float(parts[7]) if parts[7] else 0.0
        except ValueError:
            speed_knots = 0.0
        current_speed_kmh = speed_knots * 1.852

        lat = parse_lat(parts)
        lon = parse_lon(parts)
        if lat is None or lon is None:
            return False

        new_point = (lat, lon)

        if last_latlon:
            dist = haversine(*last_latlon, *new_point)
            if dist > MOVEMENT_THRESHOLD_KM:
                heading_deg = bearing(*last_latlon, *new_point)
                total_distance_km += dist
                last_latlon = new_point

                with open(GPS_LOG_PATH, "a") as f:
                    f.write(f"{time.time()},{lat},{lon},{total_distance_km:.3f},{heading_deg:.1f}\n")

                hit_details = None
                if special_idx < len(special_wpts):
                    sp = special_wpts[special_idx]
                    dist_to_sp = haversine(new_point[0], new_point[1], sp["lat"], sp["lon"])

                    if dist_to_sp < sp["clear"]:
                        current_speed_limit = None
                        show_speed = False

                        if sp.get("tag") == "fz":
                            show_speed = False
                            current_speed_limit = None
                        elif sp.get("speed") is not None:
                            current_speed_limit = sp["speed"]
                            show_speed = True

                        hit_details = f"{sp['index']}:{sp['tag']}:{sp['name']}"
                        append_event("waypoint_hit", hit_details)
                        special_idx += 1

                save_run_state("gps_move")
                if hit_details:
                    log_system_line(f"[ROADBOOK] waypoint hit {hit_details}")

                return True
            else:
                return True
        else:
            last_latlon = new_point
            save_run_state("first_fix")
            return True

    except Exception as e:
        try:
            with open(GPS_LOG_PATH, "a") as f:
                f.write(f"{time.time()},NMEA_PARSE_FAIL,{repr(line)},{e}\n")
        except Exception:
            pass
        return False


# ----------------------- GPS ----------------------
def render_gps_strip():
    gps_strip = Image.new("L", (825, 224), 255)
    draw = ImageDraw.Draw(gps_strip)

    # ---- Distance panel (left, 0–274) ----
    draw.rectangle([0, 0, 274, 199], outline=0)
    km_str = f"{total_distance_km:.2f}".replace(".", ",")
    km_bbox = draw.textbbox((0, 0), km_str, font=font_large)
    km_width = km_bbox[2] - km_bbox[0]
    km_x = 275 - km_width - 10
    draw.text((km_x, 10), km_str, font=font_large, fill=0)

    # ---- Heading panel (right, 550–824) ----
    draw.rectangle([550, 0, 824, 199], fill=200, outline=0)
    heading_str = f"{int(heading_deg):03d}°"
    heading_bbox = draw.textbbox((0, 0), heading_str, font=font_large)
    heading_width = heading_bbox[2] - heading_bbox[0]
    heading_x = 550 + (275 - heading_width) // 2
    draw.text((heading_x, 10), heading_str, font=font_large, fill=0)

    # ---- Column dividers and full outline ----
    draw.line([(275, 0), (275, 199)], fill=0)
    draw.line([(550, 0), (550, 199)], fill=0)
    draw.rectangle([0, 0, 824, 199], outline=0)

    # ---- Add black strip at bottom ----
    draw.rectangle([0, 174, 824, 223], fill=0)

    index_str = f"{special_idx}/{waypoint_count}"
    draw.text((10, 175), index_str, font=font_small, fill=255)

    elapsed_str = format_elapsed(get_elapsed_seconds())
    elapsed_bbox = draw.textbbox((0, 0), elapsed_str, font=font_small)
    elapsed_w = elapsed_bbox[2] - elapsed_bbox[0]
    draw.text((275 - elapsed_w - 10, 175), elapsed_str, font=font_small, fill=255)

    if show_speed and current_speed_limit is not None:
        speed_str = f"{current_speed_kmh:.0f} / {current_speed_limit:.0f} km/h"
        speed_bbox = draw.textbbox((0, 0), speed_str, font=font_small)
        speed_w = speed_bbox[2] - speed_bbox[0]
        draw.text((824 - speed_w - 10, 175), speed_str, font=font_small, fill=255)

    # ---- Middle Panel: Arrow toward special_idx if inside open radius ----
    if special_idx < len(special_wpts):
        sp = special_wpts[special_idx]
        if last_latlon:
            dist_to_sp = haversine(last_latlon[0], last_latlon[1], sp["lat"], sp["lon"])
            if dist_to_sp < sp["open"]:
                sp_bearing = bearing(last_latlon[0], last_latlon[1], sp["lat"], sp["lon"])
                rel_bearing = (sp_bearing - heading_deg + 360) % 360

                center_x, center_y = 412, 100
                length = 60
                angle_rad = math.radians(rel_bearing)
                end_x = center_x + length * math.sin(angle_rad)
                end_y = center_y - length * math.cos(angle_rad)

                draw.line([(center_x, center_y), (end_x, end_y)], fill=0, width=5)

                head_length = 20
                head_width = 12
                base_x = end_x - head_length * math.sin(angle_rad)
                base_y = end_y + head_length * math.cos(angle_rad)

                left_x = base_x - head_width * math.cos(angle_rad)
                left_y = base_y - head_width * math.sin(angle_rad)
                right_x = base_x + head_width * math.cos(angle_rad)
                right_y = base_y + head_width * math.sin(angle_rad)

                draw.polygon([
                    (end_x, end_y),
                    (left_x, left_y),
                    (right_x, right_y)
                ], fill=0)

                meters = int(dist_to_sp * 1000)
                dist_str = f"{meters} m"
                dist_bbox = draw.textbbox((0, 0), dist_str, font=font_small)
                dist_w = dist_bbox[2] - dist_bbox[0]
                draw.text(((825 - dist_w) // 2, 175), dist_str, font=font_small, fill=255)

    final = gps_strip.rotate(90, expand=True).quantize(colors=16)
    final.save(TMP_GPS_PATH, format="BMP")
    os.replace(TMP_GPS_PATH, GPS_OUTPUT_PATH)


# ---------------------- Helper: Stretch Contrast ----------------------
def stretch_contrast(img_l):
    arr = np.array(img_l)
    min_val = arr.min()
    max_val = arr.max()
    if max_val > min_val:
        stretched = ((arr - min_val) * (255.0 / (max_val - min_val))).astype(np.uint8)
        return Image.fromarray(stretched, mode='L')
    else:
        return img_l


# ---------------------- Render Row ----------------------
def render_row(wpt, prev_wpt=None, row_special_number=None):
    try:
        partial = ""
        partial_km = None
        if prev_wpt is not None:
            try:
                prev_dist = float(prev_wpt.findtext('default:extensions/openrally:distance', default="0", namespaces=ns).replace(",", "."))
                this_dist = float(wpt.findtext('default:extensions/openrally:distance', default="0", namespaces=ns).replace(",", "."))
                partial_km = round(this_dist - prev_dist, 2)
                partial = f"{partial_km:.2f}".replace(".", ",")
            except Exception:
                pass

        tulip_b64 = wpt.find('default:extensions/openrally:tulip', ns)
        if tulip_b64 is None or not tulip_b64.text:
            return None
        tulip_data = base64.b64decode(tulip_b64.text.strip().split(",")[-1])
        tulip_rgba = Image.open(io.BytesIO(tulip_data)).convert("RGBA")
        tulip_rgb = Image.new("RGB", tulip_rgba.size, (255, 255, 255))
        tulip_rgb.paste(tulip_rgba, mask=tulip_rgba.split()[3])
        arr = np.array(tulip_rgb).astype(float)
        luma = (0.25 * arr[..., 0] + 0.25 * arr[..., 1] + 0.5 * arr[..., 2])
        luma = np.clip(luma, 0, 255).astype(np.uint8)
        tulip_l = stretch_contrast(Image.fromarray(luma, mode='L'))
        tulip_box = Image.new("L", (275, 200), 255)
        tulip_l.thumbnail((275, 200), Image.LANCZOS)
        tulip_box.paste(tulip_l, ((275 - tulip_l.width) // 2, (200 - tulip_l.height) // 2))
        tulip_l = tulip_box

        note_img = Image.new("L", (275, 200), 255)
        note_b64 = wpt.find('default:extensions/openrally:notes', ns)
        if note_b64 is not None and note_b64.text:
            try:
                note_data = base64.b64decode(note_b64.text.strip().split(",")[-1])
                note_rgba = Image.open(io.BytesIO(note_data)).convert("RGBA")
                note_rgb = Image.new("RGB", note_rgba.size, (255, 255, 255))
                note_rgb.paste(note_rgba, mask=note_rgba.split()[3])
                arr = np.array(note_rgb).astype(float)
                luma = (0.25 * arr[..., 0] + 0.25 * arr[..., 1] + 0.5 * arr[..., 2])
                luma = np.clip(luma, 0, 255).astype(np.uint8)
                note_l = stretch_contrast(Image.fromarray(luma, mode='L'))
                note_l.thumbnail((275, 200))
                note_img = Image.new("L", (275, 200), 255)
                note_img.paste(note_l, ((275 - note_l.width) // 2, (200 - note_l.height) // 2))
            except Exception:
                pass

        final = Image.new("L", (825, 200), 255)
        draw = ImageDraw.Draw(final)

        if (
            (partial_km is not None and partial_km < 0.3)
            or wpt.find('default:extensions/openrally:wps', ns) is not None
            or wpt.find('default:extensions/openrally:wpm', ns) is not None
        ):
            draw.rectangle([0, 0, 275, 200], fill=192)

        odo = wpt.findtext('default:extensions/openrally:distance', default="", namespaces=ns).replace(".", ",")
        odo_bbox = draw.textbbox((0, 0), odo, font=font_large)
        odo_width = odo_bbox[2] - odo_bbox[0]
        odo_x = 275 - odo_width - 10
        draw.text((odo_x, 10), odo, font=font_large, fill=0)

        if partial:
            bbox = draw.textbbox((0, 0), partial, font=font_small)
            text_w = bbox[2] - bbox[0]

            px = 5
            py = 200 - font_small.size - 3
            box = [px - 5, py - 2, px + text_w + 5, py + font_small.size + 2]
            draw.rectangle(box, fill=255, outline=0)
            draw.text((px, py), partial, font=font_small, fill=0)

        final.paste(tulip_l, (275, 0))
        final.paste(note_img, (550, 0))

        cap = wpt.findtext('default:extensions/openrally:cap', default="", namespaces=ns)
        cap_bbox = draw.textbbox((0, 0), cap, font=font_small)
        cap_box_width = cap_bbox[2] - cap_bbox[0]
        cap_box_x = 555
        cap_box_y = 200 - font_small.size - 3
        draw.rectangle(
            [cap_box_x - 5, cap_box_y - 2, cap_box_x + cap_box_width + 5, cap_box_y + font_small.size + 2],
            fill=200
        )
        draw.text((cap_box_x, cap_box_y), cap, font=font_small, fill=0)

        for tag, label in [('wps', 'S'), ('wpp', 'P'), ('wpm', 'M'), ('dss', 'DSS'), ('ass', 'ASS'), ('dz', 'DZ'),
                           ('fz', 'FZ'), ('wpe', 'E'), ('wpv', 'V'), ('checkpoint', 'C'), ('neutralization', 'N')]:
            elem = wpt.find(f'default:extensions/openrally:{tag}', ns)
            if elem is not None:
                cx, cy = 275 - 35, 130
                r = 30
                fill = 128
                if tag == 'wpp':
                    fill = 255

                draw.ellipse([cx - r, cy - r, cx + r, cy + r], fill=fill, outline=0)
                label_bbox = draw.textbbox((0, 0), label, font=font_marker)
                tw = label_bbox[2] - label_bbox[0]
                th = label_bbox[3] - label_bbox[1]
                draw.text((cx - tw // 2, cy - th // 2 - 10), label, font=font_marker, fill=0)
                break

        wpt_name = wpt.findtext('default:name', default="", namespaces=ns)
        name_bbox = draw.textbbox((0, 0), wpt_name, font=font_small)
        name_box_width = name_bbox[2] - name_bbox[0]
        name_box_x = 275 - name_box_width - 5
        name_box_y = 200 - font_small.size - 3
        draw.rectangle(
            [name_box_x - 5, name_box_y - 2, name_box_x + name_box_width + 5, name_box_y + font_small.size + 2],
            fill=0
        )
        draw.text((name_box_x, name_box_y), wpt_name, font=font_small, fill=255)

        draw.rectangle([0, 0, 824, 199], outline=0)
        draw.line([(275, 0), (275, 199)], fill=0)
        draw.line([(550, 0), (550, 199)], fill=0)

        if wpt.find('default:extensions/openrally:wps', ns) is not None:
            draw.rectangle([0, 0, 824, 199], outline=0, width=5)

        if row_special_number is not None:
            circle_text = f"{row_special_number}"

            tbbox = draw.textbbox((0, 0), circle_text, font=font_small)
            tw = tbbox[2] - tbbox[0]
            th = tbbox[3] - tbbox[1]
            r = max(tw, th) // 2 + 8

            cx = 825 - r - 10
            cy = 200 - r - 5

            draw.ellipse([cx - r, cy - r, cx + r, cy + r], outline=0, fill=255)
            draw.text((cx - tw // 2, cy - th // 2 - 6), circle_text, font=font_small, fill=0)

        return final

    except Exception as e:
        print("Error rendering waypoint:", e)
        return None


# ---------------------- Render All Rows ----------------------
rendered_rows = []
row_special_number = 1
for i, wpt in enumerate(waypoints):
    prev = waypoints[i - 1] if i > 0 else None
    has_special = any(wpt.find(f'default:extensions/openrally:{tag}', ns) is not None for tag in special_tags)
    row = render_row(wpt, prev, row_special_number if has_special else None)
    if row:
        rendered_rows.append(row)
    if has_special:
        row_special_number += 1


# ---------------------- Build Strip ----------------------
def render_strip(start_idx):
    if not rendered_rows:
        return

    start_idx = max(0, min(start_idx, len(rendered_rows) - 1))
    if start_idx + 2 >= len(rendered_rows):
        start_idx = max(0, len(rendered_rows) - 3)

    strip = Image.new("L", (825, 604), 255)
    for j in range(3):
        idx = min(start_idx + 2 - j, len(rendered_rows) - 1)
        strip.paste(rendered_rows[idx], (0, j * 200))
    final = strip.rotate(90, expand=True).quantize(colors=16)
    final.save(TMP_PATH, format="BMP")
    os.replace(TMP_PATH, OUTPUT_PATH)

    print(f"Wrote {OUTPUT_PATH} for waypoints {start_idx + 1}-{min(start_idx + 3, len(rendered_rows))}")


render_strip(current_index)
render_gps_strip()


# ---------------------- GPS Loop ----------------------
def gps_loop():
    while True:
        if update_gps():
            render_gps_strip()
        time.sleep(2)


# ---------------------- State Saver Loop ----------------------
def state_saver_loop():
    while True:
        save_run_state("heartbeat")
        time.sleep(5)


# ---------------------- Button Input Loop ----------------------
def button_loop():
    global current_index, total_distance_km, last_button_time, pressed_keys
    try:
        dev = InputDevice(INPUT_EVENT)
        print(f"Listening for input from: {dev.name}")
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY:
                t = now() * 1000

                try:
                    keyname = ecodes.KEY[event.code]
                except Exception:
                    keyname = str(event.code)
                print(f"[EVENT] code={event.code} ({keyname}), value={event.value}, time={int(t)}")

                if event.value == 1 and event.code == 164:
                    print("Special combo button (164) pressed — exiting...")
                    append_event("exit_button", "key 164")
                    save_run_state("exit_button")
                    os._exit(0)

                if event.value == 1:
                    pressed_keys.add(event.code)
                elif event.value == 0:
                    pressed_keys.discard(event.code)
                else:
                    continue

                if t - last_button_time < DEBOUNCE_MS:
                    continue
                last_button_time = t

                if event.value == 1:
                    if event.code == 115:
                        total_distance_km += 0.1
                        append_event("odo_adjust", "+0.1")
                        save_run_state("odo_adjust_up")
                        render_gps_strip()
                    elif event.code == 114:
                        total_distance_km = max(0.0, total_distance_km - 0.1)
                        append_event("odo_adjust", "-0.1")
                        save_run_state("odo_adjust_down")
                        render_gps_strip()
                    elif event.code == 165:
                        if current_index < len(rendered_rows) - 3:
                            current_index += 1
                            append_event("scroll", "next")
                            save_run_state("scroll_next")
                            render_strip(current_index)
                    elif event.code == 163:
                        if current_index > 0:
                            current_index -= 1
                            append_event("scroll", "prev")
                            save_run_state("scroll_prev")
                            render_strip(current_index)
    except FileNotFoundError:
        print(f"Device {INPUT_EVENT} not found. Exiting.")
    except Exception as e:
        print(f"Error reading input device: {e}")


def system_monitor():
    os.makedirs(os.path.dirname(SYSTEM_LOG_PATH), exist_ok=True)
    with open(SYSTEM_LOG_PATH, "a") as log:
        log.write(f"\n\n=== BOOT @ {time.strftime('%Y-%m-%d %H:%M:%S')} ===\n")

    while True:
        try:
            temp = os.popen("vcgencmd measure_temp").read().strip() or "temp=?"
            throttle = os.popen("vcgencmd get_throttled").read().strip() or "throttled=?"
            volts = os.popen("vcgencmd measure_volts").read().strip() or "volt=?"
            devices = ",".join(os.listdir("/dev/input"))

            try:
                with gps_lock:
                    gps_alive = bool(gps_port is not None and gps_port.is_open)
            except Exception:
                gps_alive = False

            with open(SYSTEM_LOG_PATH, "a") as log:
                log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] ")
                log.write(f"Temp: {temp} | Throttle: {throttle} | Volts: {volts} | GPS open: {gps_alive} | Input devs: {devices}\n")
        except Exception as e:
            with open(SYSTEM_LOG_PATH, "a") as log:
                log.write(f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] [MONITOR ERROR] {e}\n")
        time.sleep(10)


def run_thread(target, name):
    def loop():
        while True:
            try:
                print(f"[INFO] Starting {name}")
                target()
            except Exception as e:
                err = f"[{time.strftime('%Y-%m-%d %H:%M:%S')}] [CRASH] {name}: {e}\n"
                print(err)
                with open(SYSTEM_LOG_PATH, "a") as log:
                    log.write(err)
                    traceback.print_exc(file=log)
            time.sleep(2)

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


run_thread(button_loop, "button_loop")
run_thread(gps_loop, "gps_loop")
run_thread(state_saver_loop, "state_saver_loop")
threading.Thread(target=system_monitor, daemon=True).start()

threading.Event().wait()
