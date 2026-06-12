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
#include <unistd.h>
#include <wiringPi.h>

// ── CONFIG ────────────────────────────────────────────
const char* SERIAL_PORT = "/dev/ttyACM0";
const int   POLL_MS     = 2000;
const char* LOG_FILE    = "bms_log.csv";

// ── GPIO PINS (BCM) ───────────────────────────────────
const int LED_PIN      = 26;
const int SEG_PINS[]   = {11,4,23,8,7,10,18,25};
const int DIGIT_PINS[] = {22,27,17,24};
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
    float voltage      = 0;
    float current      = 0;
    float remaining_ah = 0;
    float design_ah    = 0;
    int   percent      = 0;
    int   cycle_count  = 0;
    std::vector<float> temps;
    std::vector<float> cells;
    bool  valid        = false;
};

// ── READ BMS VIA JBDTOOL ──────────────────────────────
BMSData readBMS() {
    BMSData data;

    char cmd[128];
    snprintf(cmd, sizeof(cmd),
             "jbdtool -t serial:%s,9600 2>/dev/null", SERIAL_PORT);

    FILE* pipe = popen(cmd, "r");
    if (!pipe) return data;

    char line[256];
    while (fgets(line, sizeof(line), pipe)) {
        std::string s(line);
        s.erase(s.find_last_not_of(" \n\r\t") + 1);

        std::istringstream ss(s);
        std::string key, val;
        if (!(ss >> key >> val)) continue;

        if      (key == "Voltage")           data.voltage      = std::stof(val);
        else if (key == "Current")           data.current      = std::stof(val);
        else if (key == "RemainingCapacity") data.remaining_ah = std::stof(val);
        else if (key == "DesignCapacity")    data.design_ah    = std::stof(val);
        else if (key == "PercentCapacity")   data.percent      = std::stoi(val);
        else if (key == "CycleCount")        data.cycle_count  = std::stoi(val);
        else if (key == "Temps") {
            std::stringstream ts(val);
            std::string t;
            while (std::getline(ts, t, ','))
                data.temps.push_back(std::stof(t));
        }
        else if (key.substr(0,4) == "Cell"
              && key != "CellTotal"
              && key != "CellMin"
              && key != "CellMax"
              && key != "CellDiff"
              && key != "CellAvg") {
            try { data.cells.push_back(std::stof(val)); } catch(...) {}
        }
    }

    pclose(pipe);
    data.valid = (data.voltage > 0);
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

// ── LED CONTROLLER (background thread) ───────────────
void ledController() {
    while (true) {
        switch (ledMode.load()) {
            case LED_SOLID:
                digitalWrite(LED_PIN, HIGH);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100));
                break;
            case LED_FLASH:
                digitalWrite(LED_PIN, HIGH);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500));
                digitalWrite(LED_PIN, LOW);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500));
                break;
            default:
                digitalWrite(LED_PIN, LOW);
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(100));
                break;
        }
    }
}

// ── TEMP ALERT + LED ──────────────────────────────────
void monitorTempAlert(const std::vector<float>& temps) {
    if (temps.empty()) return;
    float max_t = *std::max_element(temps.begin(), temps.end());

    for (size_t i = 0; i < temps.size(); i++) {
        if (temps[i] >= 45.0f)
            std::cout << "\nCRITICAL: Probe " << i+1
                      << " is " << temps[i]
                      << "C - SHUTTING DOWN RISK!" << std::endl;
        else if (temps[i] >= 35.0f)
            std::cout << "\nWARNING: Probe " << i+1
                      << " is " << temps[i]
                      << "C - getting hot!" << std::endl;
    }

    if      (max_t >= 45.0f) ledMode = LED_SOLID;
    else if (max_t >= 35.0f) ledMode = LED_FLASH;
    else                     ledMode = LED_OFF;
}

// ── 7-SEGMENT DISPLAY ─────────────────────────────────
void displayVoltage(float voltage) {
    char buf[6];
    snprintf(buf, sizeof(buf), "%4.1f", voltage);

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
            std::this_thread::sleep_for(
                std::chrono::milliseconds(5));
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
    f << ts << "," << data.voltage << "," << data.current
      << "," << data.percent << "," << data.remaining_ah;
    for (auto t : data.temps) f << "," << t;
    if (data.current < 0) {
        float h = data.remaining_ah / std::abs(data.current);
        f << "," << (int)h << "h " << (int)((h-(int)h)*60) << "m";
    } else {
        f << ",";
    }
    f << "\n";
}

// ── PRINT TO TERMINAL ─────────────────────────────────
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
            std::cout << "Probe " << i+1
                      << ": " << data.temps[i] << "C";
        }
        std::cout << "\n";
    }

    if (data.current < 0) {
        float h = data.remaining_ah / std::abs(data.current);
        std::cout << "  Est. Runtime : " << (int)h << "h "
                  << (int)((h-(int)h)*60) << "m remaining\n";
    } else {
        std::cout << "  Est. Runtime : "
                  << (data.current > 0 ? "charging" : "idle") << "\n";
    }

    if (!data.cells.empty()) {
        float mn = *std::min_element(
            data.cells.begin(), data.cells.end());
        float mx = *std::max_element(
            data.cells.begin(), data.cells.end());
        std::cout << "\n  --- Cell Voltages ---\n";
        for (size_t i = 0; i < data.cells.size(); i++) {
            bool warn = data.cells[i] < 2.5f
                     || data.cells[i] > 3.65f;
            std::cout << "  " << (warn ? "!  " : "OK ")
                      << " Cell " << i+1
                      << " : " << data.cells[i] << " V\n";
        }
        float diff = mx - mn;
        std::cout << "\n  Cell Min  : " << mn   << " V\n";
        std::cout << "  Cell Max  : " << mx   << " V\n";
        std::cout << "  Cell Diff : " << diff << " V "
                  << (diff > 0.05f ? "HIGH IMBALANCE"
                                   : "balanced") << "\n";
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
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S",
             localtime(&now));
    return std::string(buf);
}

// ── MAIN ──────────────────────────────────────────────
int main() {
    setupGPIO();

    std::thread led_t(ledController);
    led_t.detach();

    std::cout << "JBD BMS Monitor (C++) - "
              << SERIAL_PORT << " every "
              << POLL_MS << "ms\n";
    std::cout << "Log: " << LOG_FILE
              << "\nPress Ctrl+C to stop.\n\n";

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
        std::this_thread::sleep_for(
            std::chrono::milliseconds(POLL_MS));
    }

    return 0;
}



g++ -O2 -o bms_monitor bms_monitor.cpp -lwiringPi -lpthread
