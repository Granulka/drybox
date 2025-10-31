/*
  Drybox Controller - Arduino Nano PID Temperature Control

  Hardware:
  - Arduino Nano (ATmega328P)
  - DHT11 Temperature/Humidity Sensor (D8)
  - SSD1306 OLED Display 128x64 I2C (A4=SDA, A5=SCL, Address: 0x3C)
  - NTC 100k Thermistor + 6.8k resistor divider (A2)
  - 4 Control Buttons: UP(A1), OK(A3), DOWN(A6), BACK(A7)
  - MOSFET Heater Control (D3 - PWM)
  - MOSFET Fan Control (D5)
  - Active Buzzer (A0/D14)

  Features:
  - PID temperature control
  - Multiple filament drying profiles
  - Real-time temperature monitoring
  - Safety features (overheat protection, sensor fault detection)
  - Audio feedback via buzzer

  Author: Generated for drybox project
  Date: 2025-10-31
*/

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>

// Project headers
#include "thermistor.h"
#include "pid_controller.h"
#include "safety.h"

// ======== Hardware Configuration ========

// DHT Sensor
#define USE_DHT 1
#define DHT_PIN 8
#define DHT_TYPE DHT11

// Display
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// Pin Definitions
const uint8_t PIN_HEATER = 3;    // PWM MOSFET gate (Timer2)
const uint8_t PIN_FAN    = 5;    // MOSFET gate (ON if heaterPWM>0)
const uint8_t PIN_BUZZ   = A0;   // D14 (active buzzer)
const uint8_t BTN_UP     = A1;   // digital-capable (INPUT_PULLUP)
const uint8_t BTN_OK     = A3;   // digital-capable (INPUT_PULLUP)
const uint8_t BTN_DOWN   = A6;   // analog-only (needs external 10k pull-up to 5V)
const uint8_t BTN_BACK   = A7;   // analog-only (needs external 10k pull-up to 5V)
const uint8_t PIN_THERM  = A2;   // thermistor divider

// Button Debounce
const unsigned long DEBOUNCE_MS = 30;

// Data Logging
const uint16_t LOG_LEN = 120;
float logTemp[LOG_LEN];
float logHum[LOG_LEN];
uint16_t logIndex = 0;

// ======== Objects ========
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
DHT dht(DHT_PIN, DHT_TYPE);

// ======== Drying Profiles ========
struct Profile {
  const char *name;
  float setC;           // setpoint °C
  uint32_t duration_s;  // seconds
};

Profile profiles[] = {
  {"PLA",   45.0, 4UL*3600UL},   // 4 hours
  {"PETG",  65.0, 6UL*3600UL},   // 6 hours
  {"ABS",   65.0, 6UL*3600UL},   // 6 hours
  {"TPU",   50.0, 6UL*3600UL},   // 6 hours
  {"Nylon", 75.0, 12UL*3600UL}   // 12 hours
};
const uint8_t NPROF = sizeof(profiles)/sizeof(profiles[0]);

// ======== State Machine ========
enum Mode { IDLE, MENU, RUN, DONE, FAULT };
Mode mode = IDLE;
uint8_t sel = 0;  // menu selection
uint32_t tStart = 0;
Profile activePro;

// ======== PID Controller ========
SimplePID pid;

// ======== Safety Monitor ========
SafetyMonitor safety;

// ======== Buzzer Functions ========
static unsigned long buzzOffAt = 0;

inline void buzz(uint16_t ms) {
  if (ms) {
    digitalWrite(PIN_BUZZ, HIGH);
    buzzOffAt = millis() + ms;
  }
}

inline void buzzService() {
  if (buzzOffAt && millis() > buzzOffAt) {
    digitalWrite(PIN_BUZZ, LOW);
    buzzOffAt = 0;
  }
}

inline void buzzMulti(uint8_t n, uint16_t onMs, uint16_t gapMs) {
  for (uint8_t i = 0; i < n; i++) {
    buzz(onMs);
    delay(onMs + gapMs);
  }
}

// ======== Button Handling ========
struct Button {
  uint8_t pin;
  bool analogOnly;
  bool stable;
  bool lastRaw;
  unsigned long lastChange;
};

Button btnUp   = {BTN_UP,   false, false, false, 0};
Button btnOk   = {BTN_OK,   false, false, false, 0};
Button btnDown = {BTN_DOWN, true,  false, false, 0};
Button btnBack = {BTN_BACK, true,  false, false, 0};

bool rawPressed(Button &b) {
  if (b.analogOnly) {
    int v = analogRead(b.pin);
    return v < 200;  // pressed when near GND
  } else {
    return digitalRead(b.pin) == LOW;  // INPUT_PULLUP
  }
}

bool updateButton(Button &b) {
  bool reading = rawPressed(b);
  if (reading != b.lastRaw) {
    b.lastChange = millis();
    b.lastRaw = reading;
  }
  if ((millis() - b.lastChange) > DEBOUNCE_MS) {
    if (b.stable != reading) {
      b.stable = reading;
      if (b.stable) return true;  // rising edge
    }
  }
  return false;
}

// ======== Thermistor Reading ========
float readThermC() {
  int adc = analogRead(PIN_THERM);
  return adcToTemperature(adc);
}

// ======== Control ========
int heaterPWM = 0;  // 0..255

void applyOutputs() {
  analogWrite(PIN_HEATER, heaterPWM);
  digitalWrite(PIN_FAN, (heaterPWM > 0) ? HIGH : LOW);
}

// ======== Display Functions ========
void drawHeader(const char* line) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print(line);
  display.drawLine(0, 9, 127, 9, SSD1306_WHITE);
}

void drawIdle(float tH, float tA, float rh) {
  display.clearDisplay();
  drawHeader("Drybox Monitor");

  display.setCursor(0, 12);
  display.print("Heater ");
  if (isnan(tH)) display.print("--.-C");
  else { display.print(tH, 1); display.print("C"); }

  display.setCursor(0, 24);
  display.print("Ambient ");
  if (isnan(tA)) display.print("--.-C");
  else { display.print(tA, 1); display.print("C"); }

  display.setCursor(0, 36);
  display.print("Humidity ");
  if (isnan(rh)) display.print("--%");
  else { display.print(rh, 0); display.print("%"); }

  display.setCursor(0, 52);
  display.print("OK=Menu");

  display.display();
}

void drawMenu() {
  display.clearDisplay();
  drawHeader("Select Profile");

  for (uint8_t i = 0; i < NPROF; i++) {
    display.setCursor(8, 12 + i * 10);
    if (i == sel) display.print("> ");
    else display.print("  ");
    display.print(profiles[i].name);
    display.print(" ");
    display.print(profiles[i].setC, 0);
    display.print("C/");
    display.print(profiles[i].duration_s / 3600);
    display.print("h");
  }

  display.display();
}

void drawRun(float tH, uint32_t left) {
  display.clearDisplay();
  drawHeader(activePro.name);

  display.setCursor(0, 12);
  display.print("Set ");
  display.print(activePro.setC, 0);
  display.print("C  PWM ");
  display.print((heaterPWM * 100) / 255);
  display.print("%");

  display.setCursor(0, 24);
  display.print("Temp ");
  if (isnan(tH)) display.print("--.-C");
  else { display.print(tH, 1); display.print("C"); }

  display.setCursor(0, 36);
  uint32_t h = left / 3600;
  uint32_t m = (left % 3600) / 60;
  uint32_t s = left % 60;
  display.print("Time ");
  display.print(h);
  display.print("h ");
  display.print(m);
  display.print("m ");
  display.print(s);
  display.print("s");

  display.setCursor(0, 52);
  display.print("BACK=Abort");

  display.display();
}

void drawDone() {
  display.clearDisplay();
  drawHeader("Cycle Complete!");

  display.setCursor(0, 24);
  display.print(activePro.name);
  display.print(" finished");

  display.setCursor(0, 36);
  display.print("Heater & Fan OFF");

  display.setCursor(0, 52);
  display.print("OK=Menu BACK=Idle");

  display.display();
}

void drawFault(const char* msg) {
  display.clearDisplay();
  drawHeader("FAULT!");

  display.setCursor(0, 24);
  display.print(msg);

  display.setCursor(0, 36);
  display.print("Heater OFF");

  display.setCursor(0, 52);
  display.print("BACK=Idle");

  display.display();
}

// ======== Setup ========
void setup() {
  // Initialize outputs
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);

  analogWrite(PIN_HEATER, 0);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_BUZZ, LOW);

  // Initialize buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_OK, INPUT_PULLUP);
  // A6/A7 are analog-only, need external 10k pull-ups

  // Initialize display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // Display init failed - emergency stop
    while (1) {
      analogWrite(PIN_HEATER, 0);
      digitalWrite(PIN_FAN, LOW);
      buzz(100);
      delay(500);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 24);
  display.print("Drybox Controller");
  display.setCursor(0, 36);
  display.print("Initializing...");
  display.display();
  delay(1000);

  // Initialize DHT sensor
  dht.begin();

  // Initialize data logs
  for (uint16_t i = 0; i < LOG_LEN; i++) {
    logTemp[i] = NAN;
    logHum[i] = NAN;
  }

  // Initialize PID
  pid.setOutputLimits(0, 255);

  // Startup beep
  buzzMulti(2, 100, 100);
}

// ======== Main Loop ========
void loop() {
  // Read sensors
  float tH = readThermC();  // heater/chamber temp

  float tA = NAN, rh = NAN;  // ambient temp and humidity
#if USE_DHT
  tA = dht.readTemperature();
  rh = dht.readHumidity();
#endif

  // Update data logs
  logTemp[logIndex] = tH;
  logHum[logIndex] = rh;
  logIndex = (logIndex + 1) % LOG_LEN;

  // Read buttons
  bool up = updateButton(btnUp);
  bool ok = updateButton(btnOk);
  bool dn = updateButton(btnDown);
  bool back = updateButton(btnBack);

  // Safety checks
  if (safety.checkTemperatureFault(tH)) {
    heaterPWM = 0;
    applyOutputs();
    mode = FAULT;
  }

  // State machine
  switch (mode) {
    case IDLE:
      heaterPWM = 0;
      applyOutputs();
      if (ok) {
        buzz(40);
        mode = MENU;
      }
      drawIdle(tH, tA, rh);
      break;

    case MENU:
      if (up && sel > 0) {
        sel--;
        buzz(40);
      }
      if (dn && sel < NPROF - 1) {
        sel++;
        buzz(40);
      }
      if (ok) {
        activePro = profiles[sel];
        pid.sp = activePro.setC;
        pid.reset(isnan(tH) ? pid.sp : tH);
        tStart = millis();
        buzzMulti(2, 60, 60);
        mode = RUN;
      }
      if (back) {
        buzz(40);
        mode = IDLE;
      }
      drawMenu();
      break;

    case RUN: {
      // Run PID controller
      if (!isnan(tH)) {
        double out;
        if (pid.compute(tH, out, millis())) {
          heaterPWM = (int)(out + 0.5);
          if (heaterPWM < 0) heaterPWM = 0;
          if (heaterPWM > 255) heaterPWM = 255;
          applyOutputs();
        }
      } else {
        heaterPWM = 0;
        applyOutputs();
      }

      uint32_t elapsed = (millis() - tStart) / 1000UL;
      uint32_t left = (activePro.duration_s > elapsed) ? (activePro.duration_s - elapsed) : 0;

      if (left == 0) {
        heaterPWM = 0;
        applyOutputs();
        buzzMulti(3, 80, 80);
        mode = DONE;
      }
      if (back) {
        heaterPWM = 0;
        applyOutputs();
        buzz(120);
        mode = IDLE;
      }
      drawRun(tH, left);
      break;
    }

    case DONE:
      heaterPWM = 0;
      applyOutputs();
      if (ok) {
        buzz(40);
        mode = MENU;
      }
      if (back) {
        buzz(40);
        mode = IDLE;
      }
      drawDone();
      break;

    case FAULT: {
      static bool faultBeeped = false;
      heaterPWM = 0;
      applyOutputs();
      if (!faultBeeped) {
        buzz(300);
        faultBeeped = true;
      }
      if (back) {
        faultBeeped = false;
        buzz(40);
        mode = IDLE;
      }
      drawFault(safety.isThermistorFaulted() ? "Thermistor error" : "Overtemperature");
      break;
    }
  }

  // Service buzzer
  buzzService();

  delay(40);
}
