#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>

// ── CONFIG ────────────────────────────────────────────
const char* SERIAL_PORT = "/dev/ttyACM0";
const int   POLL_MS     = 2000;
const char* LOG_FILE    = "bms_log.csv";

// ── GPIO PINS (BCM) ───────────────────────────────────
const int LED_PIN       = 26;
const int SEG_PINS[]    = {11,4,23,8,7,10,18,25};
const int DIGIT_PINS[]  = {22,27,17,24};
// ──────────────────────────────────────────────────────

// 7-segment patterns for 0-9
const int SEG_PATTERNS[10][7] = {
    {1,1,1,1,1,1,0}, // 0
    {0,1,1,0,0,0,0}, // 1
    {1,1,0,1,1,0,1}, // 2
    {1,1,1,1,0,0,1}, // 3
    {0,1,1,0,0,1,1}, // 4
    {1,0,1,1,0,1,1}, // 5
    {1,0,1,1,1,1,1}, // 6
    {1,1,1,0,0,0,0}, // 7
    {1,1,1,1,1,1,1}, // 8
    {1,1,1,1,0,1,1}, // 9
};

// LED modes
enum LedMode { LED_OFF, LED_FLASH, LED_SOLID };
std::atomic<LedMode> ledMode(LED_OFF);

// BMS data structure
struct BMSData {
    float voltage         = 0;
    float current         = 0;
    float remaining_ah    = 0;
    float design_ah       = 0;
    int   percent         = 0;
    int   cycle_count     = 0;
    std::vector<float> temps;
    std::vector<float> cells;
    bool valid            = false;
};

int serial_fd = -1;

// ── SERIAL SETUP ──────────────────────────────────────
bool setupSerial() {
    serial_fd = open(SERIAL_PORT, O_RDWR | O_NOCTTY | O_SYNC);
    if (serial_fd < 0) {
        std::cerr << "ERROR: Cannot open " << SERIAL_PORT << std::endl;
        return false;
    }
    struct termios tty = {};
    tcgetattr(serial_fd, &tty);
    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);
    tty.c_cflag     = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag     |= (CLOCAL | CREAD);
    tty.c_cflag     &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    tty.c_lflag     = 0;
    tty.c_oflag     = 0;
    tty.c_iflag     &= ~(IXON | IXOFF | IXANY | IGNBRK);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 30;  // 3 second timeout
    tcsetattr(serial_fd, TCSANOW, &tty);
    return true;
}

// ── JBD SERIAL PROTOCOL ───────────────────────────────
std::vector<uint8_t> sendCommand(uint8_t cmd) {
    uint16_t chk = (0x10000 - cmd) & 0xFFFF;
    uint8_t packet[] = {0xDD, 0xA5, cmd, 0x00,
                        (uint8_t)(chk >> 8), (uint8_t)(chk & 0xFF), 0x77};
    tcflush(serial_fd, TCIOFLUSH);
    write(serial_fd, packet, sizeof(packet));

    std::vector<uint8_t> resp;
    uint8_t buf[256];
    int total = 0;
    auto start = std::chrono::steady_clock::now();

    while (true) {
        int n = read(serial_fd, buf + total, sizeof(buf) - total);
        if (n > 0) total += n;
        if (total >= 7 && buf[total-1] == 0x77) break;
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start).count();
        if (ms > 3000) break;
    }
    for (int i = 0; i < total; i++) resp.push_back(buf[i]);
    return resp;
}

// ── READ BMS ──────────────────────────────────────────
BMSData readBMS() {
    BMSData data;

    // Basic info (cmd 0x03)
    auto r = sendCommand(0x03);
    if (r.size() < 7 || r[0] != 0xDD || r[2] != 0x00) return data;
    uint8_t* d = r.data() + 4;

    data.voltage       = ((d[0] << 8) | d[1]) / 100.0f;
    int16_t raw_cur    = (d[2] << 8) | d[3];
    data.current       = raw_cur / 100.0f;
    data.remaining_ah  = ((d[4] << 8) | d[5]) / 100.0f;
    data.design_ah     = ((d[6] << 8) | d[7]) / 100.0f;
    data.cycle_count   = (d[8] << 8) | d[9];
    data.percent       = d[19];
    int num_temps      = d[22];
    for (int i = 0; i < num_temps; i++) {
        int raw = (d[23 + i*2] << 8) | d[24 + i*2];
        data.temps.push_back((raw - 2731) / 10.0f);
    }

    // Cell voltages (cmd 0x04)
    auto cr = sendCommand(0x04);
    if (cr.size() >= 7 && cr[0] == 0xDD && cr[2] == 0x00) {
        int num_cells = cr[3] / 2;
        for (int i = 0; i < num_cells; i++) {
            int raw = (cr[4 + i*2] << 8) | cr[5 + i*2];
            data.cells.push_back(raw / 1000.0f);
        }
    }

    data.valid = true;
    return data;
}

// ── GPIO SETUP ────────────────────────────────────────
void setupGPIO() {
    wiringPiSetupGpio();
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);
    for (int i = 0; i < 8; i++) {
        pinMode(SEG_PINS[i], OUTPUT);
        digitalWrite(SEG_PINS[i], LOW);
    }
    for (int i = 0; i < 4; i++) {
        pinMode(DIGIT_PINS[i], OUTPUT);
        digitalWrite(DIGIT_PINS[i], HIGH);
    }
}

// ── LED CONTROLLER (runs in background thread) ────────
void ledController() {
    while (true) {
        switch (ledMode.load()) {
            case LED_SOLID:
                digitalWrite(LED_PIN, HIGH);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
            case LED_FLASH:
                digitalWrite(LED_PIN, HIGH);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                digitalWrite(LED_PIN, LOW);
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                break;
            default:
                digitalWrite(LED_PIN, LOW);
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                break;
        }
    }
}

// ── TEMP ALERT ────────────────────────────────────────
void monitorTempAlert(const std::vector<float>& temps) {
    if (temps.empty()) return;
    float max_t = *std::max_element(temps.begin(), temps.end());
    for (size_t i = 0; i < temps.size(); i++) {
        if (temps[i] >= 45.0f)
            std::cout << "\nCRITICAL: Probe " << i+1
                      << " is " << temps[i] << "C - SHUTTING DOWN RISK!" << std::endl;
        else if (temps[i] >= 35.0f)
            std::cout << "\nWARNING: Probe " << i+1
                      << " is " << temps[i] << "C - getting hot!" << std::endl;
    }
    if      (max_t >= 45.0f) ledMode = LED_SOLID;
    else if (max_t >= 35.0f) ledMode = LED_FLASH;
    else                     ledMode = LED_OFF;
}

// ── 7-SEGMENT DISPLAY ─────────────────────────────────
void displayVoltage(float voltage) {
    char buf[6];
    snprintf(buf, sizeof(buf), "%4.1f", voltage);

    // Find and remove decimal point, track its position
    char digits[5] = {' ',' ',' ',' ','\0'};
    int dp_pos = -1, di = 0;
    for (int i = 0; buf[i] && di < 4; i++) {
        if (buf[i] == '.') { dp_pos = di - 1; continue; }
        digits[di++] = buf[i];
    }

    auto end_t = std::chrono::steady_clock::now()
               + std::chrono::milliseconds(1000);
    while (std::chrono::steady_clock::now() < end_t) {
        for (int d = 0; d < 4; d++) {
            char c = digits[d];
            if (c >= '0' && c <= '9') {
                int n = c - '0';
                for (int s = 0; s < 7; s++)
                    digitalWrite(SEG_PINS[s], SEG_PATTERNS[n][s]);
            } else {
                for (int s = 0; s < 7; s++)
                    digitalWrite(SEG_PINS[s], LOW);
            }
            digitalWrite(SEG_PINS[7], (d == dp_pos) ? HIGH : LOW);
            digitalWrite(DIGIT_PINS[d], LOW);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            digitalWrite(DIGIT_PINS[d], HIGH);
        }
    }
}

// ── LOG TO CSV ────────────────────────────────────────
void logCSV(const BMSData& data, const std::string& ts) {
    bool write_header = (access(LOG_FILE, F_OK) != 0);
    std::ofstream f(LOG_FILE, std::ios::app);
    if (write_header) {
        f << "timestamp,voltage_v,current_a,percent,remaining_ah";
        for (size_t i = 0; i < data.temps.size(); i++)
            f << ",temp" << i+1 << "_c";
        f << ",est_runtime\n";
    }
    f << ts << "," << data.voltage << "," << data.current << ","
      << data.percent << "," << data.remaining_ah;
    for (auto t : data.temps) f << "," << t;
    if (data.current < 0) {
        float h = data.remaining_ah / std::abs(data.current);
        f << "," << (int)h << "h " << (int)((h-(int)h)*60) << "m";
    } else {
        f << ",";
    }
    f << "\n";
}

// ── DISPLAY TO TERMINAL ───────────────────────────────
void printData(const BMSData& data, const std::string& ts) {
    std::string state = data.current > 0 ? "Charging"
                      : data.current < 0 ? "Discharging" : "Idle";
    std::cout << "\n----------------------------------------\n";
    std::cout << "  " << ts << "\n";
    std::cout << "----------------------------------------\n";
    std::cout << "  Voltage      : " << data.voltage    << " V\n";
    std::cout << "  State        : " << state           << "\n";
    std::cout << "  Current      : " << data.current    << " A\n";
    std::cout << "  Charge       : " << data.percent    << "%  ("
              << data.remaining_ah << " Ah remaining)\n";

    if (!data.temps.empty()) {
        std::cout << "  Temperature  : ";
        for (size_t i = 0; i < data.temps.size(); i++) {
            if (i) std::cout << ",  ";
            std::cout << "Probe " << i+1 << ": " << data.temps[i] << "C";
        }
        std::cout << "\n";
    }

    if (data.current < 0) {
        float h = data.remaining_ah / std::abs(data.current);
        std::cout << "  Est. Runtime : " << (int)h << "h "
                  << (int)((h-(int)h)*60) << "m remaining\n";
    } else {
        std::cout << "  Est. Runtime : " << (data.current > 0 ? "charging" : "idle") << "\n";
    }

    if (!data.cells.empty()) {
        float mn = *std::min_element(data.cells.begin(), data.cells.end());
        float mx = *std::max_element(data.cells.begin(), data.cells.end());
        std::cout << "\n  --- Cell Voltages ---\n";
        for (size_t i = 0; i < data.cells.size(); i++) {
            bool warn = data.cells[i] < 2.5f || data.cells[i] > 3.65f;
            std::cout << "  " << (warn ? "!  " : "OK ") << " Cell "
                      << i+1 << " : " << data.cells[i] << " V\n";
        }
        float diff = mx - mn;
        std::cout << "\n  Cell Min  : " << mn   << " V\n";
        std::cout << "  Cell Max  : " << mx   << " V\n";
        std::cout << "  Cell Diff : " << diff << " V "
                  << (diff > 0.05f ? "HIGH IMBALANCE" : "balanced") << "\n";
    }
    std::cout << "----------------------------------------\n";
}

// ── STARTUP TEST ──────────────────────────────────────
void startupTest() {
    std::cout << "Running startup test..." << std::endl;
    ledMode = LED_FLASH;
    displayVoltage(0.0f);
    ledMode = LED_OFF;
    std::cout << "Startup test complete!\n" << std::endl;
}

// ── GET TIMESTAMP ─────────────────────────────────────
std::string getTimestamp() {
    time_t now = time(nullptr);
    char buf[32];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    return std::string(buf);
}

// ── MAIN ──────────────────────────────────────────────
int main() {
    setupGPIO();
    if (!setupSerial()) return 1;

    std::thread led_t(ledController);
    led_t.detach();

    std::cout << "JBD BMS Monitor (C++) - " << SERIAL_PORT
              << " every " << POLL_MS << "ms\n";
    std::cout << "Log: " << LOG_FILE << "\nPress Ctrl+C to stop.\n\n";

    startupTest();

    float voltage = 0.0f;
    while (true) {
        BMSData data = readBMS();
        if (data.valid) {
            voltage = data.voltage;
            std::string ts = getTimestamp();
            printData(data, ts);
            logCSV(data, ts);
            monitorTempAlert(data.temps);
        }
        displayVoltage(voltage);
        std::this_thread::sleep_for(std::chrono::milliseconds(POLL_MS));
    }

    close(serial_fd);
    return 0;
}
