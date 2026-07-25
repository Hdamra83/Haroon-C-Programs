#!/usr/bin/env python3
import serial
import time
import os
import sys
import struct
import board
import busio
import adafruit_dht
import adafruit_bmp280
import RPi.GPIO as GPIO
from datetime import datetime

# ── CONFIG ────────────────────────────────────────────
SERIAL_PORT  = "/dev/ttyAMA10"
BAUD_RATE    = 9600
POLL_SECONDS = 1
DATA_DIR     = f"Go-Kart Data - {datetime.now().strftime('%d-%m-%Y')}"
RUN_DIR      = os.path.join(DATA_DIR, "Run Recording")
LOG_FILE     = os.path.join(RUN_DIR, "bms_log.csv")
DHT11_LOG_FILE = os.path.join(RUN_DIR, "dht11_log.csv")
BMP280_1_LOG_FILE = os.path.join(RUN_DIR, "bmp280_1_log.csv")
BMP280_2_LOG_FILE = os.path.join(RUN_DIR, "bmp280_2_log.csv")
LED_BRAKE    = 16

DHT11_PIN    = board.D26
BMP280_1_ADDR = 0x76  # SDO tied to GND
BMP280_2_ADDR = 0x77  # SDO tied to VCC
# ──────────────────────────────────────────────────────

os.makedirs(RUN_DIR, exist_ok=True)

# ── GPIO SETUP ────────────────────────────────────────
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_BRAKE, GPIO.OUT)
GPIO.output(LED_BRAKE, GPIO.HIGH)

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

# ── SETUP DHT11 ────────────────────────────────────────
try:
    dht_device = adafruit_dht.DHT11(DHT11_PIN)
except Exception as e:
    print(f"ERROR: Cannot set up DHT11 on pin 26: {e}")
    dht_device = None

# ── SETUP BMP280s (shared I2C bus: SDA=GPIO2, SCL=GPIO3) ──
try:
    i2c = busio.I2C(board.SCL, board.SDA)
except Exception as e:
    print(f"ERROR: Cannot open I2C bus: {e}")
    i2c = None

def _setup_bmp280(addr, label):
    if i2c is None:
        return None
    try:
        return adafruit_bmp280.Adafruit_BMP280_I2C(i2c, address=addr)
    except Exception as e:
        print(f"ERROR: Cannot set up BMP280 {label} at {hex(addr)}: {e}")
        return None

bmp280_1 = _setup_bmp280(BMP280_1_ADDR, "#1")
bmp280_2 = _setup_bmp280(BMP280_2_ADDR, "#2")

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

# ── DHT11 ──────────────────────────────────────────────
def read_dht11():
    """Read temperature (C) and humidity (%) from the DHT11 on GPIO26."""
    if dht_device is None:
        return None
    try:
        temp_c = dht_device.temperature
        humidity = dht_device.humidity
        if temp_c is None or humidity is None:
            return None
        return {"temp_c": temp_c, "humidity": humidity}
    except RuntimeError as e:
        # DHT11 reads drop out often — not a fatal error, just skip this cycle
        print(f"DHT11 read skipped: {e}")
        return None
    except Exception as e:
        print(f"ERROR reading DHT11: {e}")
        return None

def log_dht11_csv(temp_c, humidity):
    now = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
    write_header = not os.path.exists(DHT11_LOG_FILE)
    with open(DHT11_LOG_FILE, "a") as f:
        if write_header:
            f.write("timestamp,temp_c,humidity_pct\n")
        f.write(f"{now},{temp_c:.1f},{humidity:.1f}\n")

# ── BMP280 ─────────────────────────────────────────────
def read_bmp280(sensor):
    """Read temperature (C) and pressure (hPa) from a BMP280."""
    if sensor is None:
        return None
    try:
        return {"temp_c": sensor.temperature, "pressure_hpa": sensor.pressure}
    except Exception as e:
        print(f"ERROR reading BMP280: {e}")
        return None

def log_bmp280_csv(log_file, temp_c, pressure_hpa):
    now = datetime.now().strftime("%d-%m-%Y %H:%M:%S")
    write_header = not os.path.exists(log_file)
    with open(log_file, "a") as f:
        if write_header:
            f.write("timestamp,temp_c,pressure_hpa\n")
        f.write(f"{now},{temp_c:.2f},{pressure_hpa:.2f}\n")

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
            print(f"  Cell {i+1:2d} : {v:.3f} V")
        diff = max(cells) - min(cells)
        print(f"\n  Cell Min     : {min(cells):.3f} V")
        print(f"  Cell Max     : {max(cells):.3f} V")
        print(f"  Cell Diff    : {diff:.3f} V")

    print(f"{'─'*40}")
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

# ── MAIN ──────────────────────────────────────────────
def main():
    print(f"JBD BMS Monitor — direct serial {SERIAL_PORT} @ {BAUD_RATE} baud")
    print(f"Log file: {LOG_FILE}")
    print(f"DHT11 log file: {DHT11_LOG_FILE}")
    print(f"BMP280 #1 log file: {BMP280_1_LOG_FILE}")
    print(f"BMP280 #2 log file: {BMP280_2_LOG_FILE}")
    print("Press Ctrl+C to stop.\n")

    try:
        while True:
            data = read_bms()
            if data:
                now, voltage, current, pct, remain, temps, runtime = display(data)
                log_csv(now, voltage, current, pct, remain, temps, runtime)
            else:
                print("WARNING: No data from BMS — retrying...")

            dht11_data = read_dht11()
            if dht11_data:
                log_dht11_csv(dht11_data["temp_c"], dht11_data["humidity"])

            bmp280_1_data = read_bmp280(bmp280_1)
            if bmp280_1_data:
                log_bmp280_csv(BMP280_1_LOG_FILE, bmp280_1_data["temp_c"], bmp280_1_data["pressure_hpa"])

            bmp280_2_data = read_bmp280(bmp280_2)
            if bmp280_2_data:
                log_bmp280_csv(BMP280_2_LOG_FILE, bmp280_2_data["temp_c"], bmp280_2_data["pressure_hpa"])

            time.sleep(POLL_SECONDS)

    except KeyboardInterrupt:
        print("\nStopping monitor...")
        if dht_device is not None:
            dht_device.exit()
        GPIO.cleanup()
        ser.close()

if __name__ == "__main__":
    main()
