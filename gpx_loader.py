import os
import subprocess
import sys
from PIL import Image, ImageDraw, ImageFont
from evdev import InputDevice, ecodes
from time import time as now

GPX_DIR = "/home/ben"
GPS_OUTPUT_PATH = "/tmp/eink_gps.bmp"
TMP_GPS_PATH = "/tmp/eink_gps_temp.bmp"
INPUT_EVENT = "/dev/input/event5"

font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 36)

pressed_keys = set()
last_button_time = 0
DEBOUNCE_MS = 300

def get_temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except:
        return None

def get_ip():
    try:
        result = subprocess.check_output("hostname -I", shell=True).decode().strip()
        return result.split()[0] if result else None
    except:
        return None

def list_gpx_files():
    return sorted([f for f in os.listdir(GPX_DIR) if f.endswith(".gpx")])

def render_menu(files, selected_idx):
    img = Image.new("L", (825, 224), 255)
    draw = ImageDraw.Draw(img)

    # Calculate visible window
    window_size = 5
    half = window_size // 2
    start = max(0, selected_idx - half)
    end = min(len(files), start + window_size)
    if end - start < window_size:
        start = max(0, end - window_size)

    for i, name in enumerate(files[start:end]):
        y = i * 40 + 10
        prefix = "> " if start + i == selected_idx else "  "
        draw.text((10, y), prefix + name, font=font, fill=0)

    # Temp and IP at bottom right
    temp = get_temp_c()
    ip = get_ip()

    text_lines = []
    if ip:
        text_lines.append(ip)
    if temp:
        text_lines.append(f"{temp:.1f}°C")

    y_offset = 224 - 10
    for line in reversed(text_lines):
        bbox = draw.textbbox((0, 0), line, font=font)
        tx = 825 - bbox[2] - 10
        ty = y_offset - bbox[3]
        draw.text((tx, ty), line, font=font, fill=0)
        y_offset = ty - 5

    final = img.rotate(90, expand=True).quantize(colors=16)
    final.save(TMP_GPS_PATH, format="BMP")
    os.replace(TMP_GPS_PATH, GPS_OUTPUT_PATH)

def button_loop(files):
    selected = 0
    render_menu(files, selected)

    global last_button_time, pressed_keys

    try:
        dev = InputDevice(INPUT_EVENT)
        print(f"Listening for input from: {dev.name}")
        for event in dev.read_loop():
            if event.type == ecodes.EV_KEY:
                t = now() * 1000

                try:
                    keyname = ecodes.KEY[event.code]
                except:
                    keyname = str(event.code)
                print(f"[EVENT] code={event.code} ({keyname}), value={event.value}, time={int(t)}")

                if event.value == 1 and event.code == 164:  # KEY_PLAYPAUSE
                    print("Special combo button (164) pressed — exiting...")
                    os._exit(0)

                if event.value == 1:
                    pressed_keys.add(event.code)
                elif event.value == 0:
                    pressed_keys.discard(event.code)

                if t - last_button_time < DEBOUNCE_MS:
                    continue
                last_button_time = t

                if event.value == 1:
                    if event.code == 115:  # VOLUMEUP
                        full_path = os.path.join(GPX_DIR, files[selected])
                        subprocess.run(["/usr/bin/python3", "/home/ben/bluetooth_tulip_extractor.py", full_path])
                        sys.exit(0)
                    elif event.code == 114:  # VOLUMEDOWN
                        pass  # unused here
                    elif event.code == 165:  # NEXT
                        selected = (selected + 1) % len(files)
                        render_menu(files, selected)
                    elif event.code == 163:  # PREV
                        selected = (selected - 1) % len(files)
                        render_menu(files, selected)

    except FileNotFoundError:
        print(f"Device {INPUT_EVENT} not found. Exiting.")
    except Exception as e:
        print(f"Error reading input device: {e}")

def main():
    files = list_gpx_files()
    if files:
        button_loop(files)

if __name__ == "__main__":
    main()
