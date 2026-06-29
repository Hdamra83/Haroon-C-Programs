#!/usr/bin/env python3
import serial
import time
import os
import sys
import struct
import threading
import RPi.GPIO as GPIO
from datetime import datetime

# ── CONFIG ────────────────────────────────────────────
SERIAL_PORT  = "/dev/ttyAMA10"
BAUD_RATE    = 9600
POLL_SECONDS = 1
LOG_FILE     = "bms_log.csv"
led_state    = {"mode": "off"}
LED_PIN      = 26
LED_BRAKE    = 16
# ──────────────────────────────────────────────────────

# ── GPIO SETUP ────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN,   GPIO.OUT)
GPIO.setup(LED_BRAKE, GPIO.OUT)
GPIO.output(LED_PIN,   GPIO.LOW)
GPIO.output(LED_BRAKE, GPIO.HIGH)

# ── LED CONTROLLER ────────────────────────────────────
def led_controller():
    while True:
        mode = led_state["mode"]
        if mode == "solid":
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(0.1)
        elif mode == "flash":
            GPIO.output(LED_PIN, GPIO.HIGH)
            time.sleep(0.5)
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(0.5)
        else:
            GPIO.output(LED_PIN, GPIO.LOW)
            time.sleep(0.1)

led_thread = threading.Thread(target=led_controller, daemon=True)
led_thread.start()

# ── OPEN SERIAL PORT ──────────────────────────────────
try:
    ser = serial.Serial(
        port=SERIAL_PORT,
        baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=2
    )
except Exception as e:
    print(f"ERROR: Cannot open {SERIAL_PORT}: {e}")
    sys.exit(1)

# ── JBD PROTOCOL ──────────────────────────────────────
def calc_checksum(data):
    chk = 0x10000
    for b in data:
        chk -= b
    return chk & 0xFFFF

def send_command(reg):
    """Send a read command to the BMS."""
    chk = calc_checksum([reg, 0x00])
    packet = bytes([0xDD, 0xA5, reg, 0x00,
                    (chk >> 8) & 0xFF, chk & 0xFF, 0x77])
    ser.reset_input_buffer()
    ser.write(packet)

def read_response():
    """Read and validate response from BMS."""
    # Wait for start byte 0xDD
    start = ser.read(1)
    if not start or start[0] != 0xDD:
        return None

    # Read register, status, length
    header = ser.read(3)
    if len(header) < 3:
        return None

    reg    = header[0]
    status = header[1]
    length = header[2]

    if status != 0x00:
        return None

    # Read data + checksum + end byte
    rest = ser.read(length + 3)
    if len(rest) < length + 3:
        return None

    data     = rest[:length]
    checksum = (rest[length] << 8) | rest[length+1]
    end      = rest[length+2]

    if end != 0x77:
        return None

    # Verify checksum
    chk_data = [reg, status, length] + list(data)
    expected = calc_checksum(chk_data[1:])  # status + length + data
    # Return data if end byte is correct
    return data

# ── PARSE BASIC INFO (0x03) ───────────────────────────
def parse_basic_info(data):
    if len(data) < 23:
        return None

    voltage   = struct.unpack('>H', data[0:2])[0]  / 100.0
    current   = struct.unpack('>h', data[2:4])[0]  / 100.0  # signed
    remaining = struct.unpack('>H', data[4:6])[0]  / 100.0
    design    = struct.unpack('>H', data[6:8])[0]  / 100.0
    cycles    = struct.unpack('>H', data[8:10])[0]
    percent   = data[19]
    num_temps = data[22]

    temps = []
    for i in range(num_temps):
        idx = 23 + i * 2
        if idx + 1 < len(data):
            raw = struct.unpack('>H', data[idx:idx+2])[0]
            temps.append((raw - 2731) / 10.0)

    return {
        "voltage":   voltage,
        "current":   current,
        "remaining": remaining,
        "design":    design,
        "cycles":    cycles,
        "percent":   percent,
        "temps":     temps,
    }

# ── PARSE CELL VOLTAGES (0x04) ────────────────────────
def parse_cells(data):
    cells = []
    num = len(data) // 2
    for i in range(num):
        raw = struct.unpack('>H', data[i*2:i*2+2])[0]
        cells.append(raw / 1000.0)
    return cells

# ── READ BMS ──────────────────────────────────────────
def read_bms():
    try:
        # Read basic info
        send_command(0x03)
        time.sleep(0.1)
        raw_basic = read_response()
        if not raw_basic:
            return None
        basic = parse_basic_info(raw_basic)
        if not basic:
            return None

        # Read cell voltages
        send_command(0x04)
        time.sleep(0.1)
        raw_cells = read_response()
        cells = parse_cells(raw_cells) if raw_cells else []

        basic["cells"] = cells
        return basic

    except Exception as e:
        print(f"ERROR reading BMS: {e}")
        return None

# ── TEMP ALERT ────────────────────────────────────────
def monitor_temp_alert(temps):
    if not temps:
        return
    max_temp = max(temps)
    for i, t in enumerate(temps):
        if t >= 45:
            print(f"\n🔥 CRITICAL: Probe {i+1} is {t:.1f}C — SHUTTING DOWN RISK!")
        elif t >= 35:
            print(f"\n⚠️  WARNING: Probe {i+1} is {t:.1f}C — getting hot!")

    if max_temp >= 45:
        led_state["mode"] = "solid"
    elif max_temp >= 35:
        led_state["mode"] = "flash"
    else:
        led_state["mode"] = "off"

# ── DISPLAY TO TERMINAL ───────────────────────────────
def display(data):
    now     = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    voltage = data["voltage"]
    current = data["current"]
    pct     = data["percent"]
    remain  = data["remaining"]
    temps   = data["temps"]
    cells   = data["cells"]
    state   = "Charging" if current > 0 else "Discharging" if current < 0 else "Idle"

    runtime = None
    if current < 0:
        h = remain / abs(current)
        runtime = f"{int(h)}h {int((h - int(h)) * 60)}m"

    print(f"\n{'─'*40}")
    print(f"  {now}")
    print(f"{'─'*40}")
    print(f"  Voltage      : {voltage:.2f} V")
    print(f"  State        : {state}")
    print(f"  Current      : {current:.2f} A")
    print(f"  Charge       : {pct}%  ({remain:.1f} Ah remaining)")
    if temps:
        labels = [f"Probe {i+1}: {t:.1f}C" for i, t in enumerate(temps)]
        print(f"  Temperature  : {', '.join(labels)}")
    if runtime:
        print(f"  Est. Runtime : {runtime} remaining")
    elif current > 0:
        print(f"  Est. Runtime : charging")
    else:
        print(f"  Est. Runtime : idle")

    if cells:
        print(f"\n  --- Cell Voltages ---")
        for i, v in enumerate(cells):
            warn = "⚠️ " if v < 2.5 or v > 3.65 else "✓  "
            print(f"  {warn} Cell {i+1:2d} : {v:.3f} V")
        diff = max(cells) - min(cells)
        print(f"\n  Cell Min     : {min(cells):.3f} V")
        print(f"  Cell Max     : {max(cells):.3f} V")
        print(f"  Cell Diff    : {diff:.3f} V  "
              f"{'⚠️  HIGH IMBALANCE' if diff > 0.05 else '✓ balanced'}")

    print(f"{'─'*40}")
    monitor_temp_alert(temps)
    return now, voltage, current, pct, remain, temps, runtime

# ── LOG TO CSV ────────────────────────────────────────
def log_csv(now, voltage, current, pct, remaining, temps, runtime):
    write_header = not os.path.exists(LOG_FILE)
    with open(LOG_FILE, "a") as f:
        if write_header:
            temp_headers = ",".join([f"temp{i+1}_c" for i in range(len(temps))])
            f.write(f"timestamp,voltage_v,current_a,percent,"
                    f"remaining_ah,{temp_headers},est_runtime\n")
        temp_vals = ",".join([str(t) for t in temps])
        f.write(f"{now},{voltage},{current},{pct},"
                f"{remaining},{temp_vals},{runtime or ''}\n")

# ── STARTUP TEST ──────────────────────────────────────
def startup_test():
    print("Running startup test...")
    led_state["mode"] = "flash"
    time.sleep(3)
    led_state["mode"] = "off"
    print("Startup test complete!\n")

# ── MAIN ──────────────────────────────────────────────
def main():
    print(f"JBD BMS Monitor — direct serial {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Log file: {LOG_FILE}")
    print("Press Ctrl+C to stop.\n")

    startup_test()

    try:
        while True:
            data = read_bms()
            if data:
                now, voltage, current, pct, remain, temps, runtime = display(data)
                log_csv(now, voltage, current, pct, remain, temps, runtime)
            else:
                print("WARNING: No data from BMS — retrying...")
            time.sleep(POLL_SECONDS)

    except KeyboardInterrupt:
        print("\nStopping monitor...")
        led_state["mode"] = "off"
        time.sleep(0.2)
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    main()



pip3 install pyserial --break-system-packages
