/*
  Drybox Controller - Arduino Nano PID Temperature Control

  Hardware:
  - Arduino Nano (ATmega328P)
  - DHT11 Temperature/Humidity Sensor (D8)
  - SSD1306 OLED Display 128x64 I2C (A4=SDA, A5=SCL, Address: 0x3C)
  - NTC 100k Thermistor + series resistor divider (A2), see thermistor.h
  - 4 Control Buttons (2x2): UP(A1) BACK(A7) / DOWN(A3) OK(A6)
  - MOSFET Heater Control (D3 - PWM)
  - MOSFET Fan Control (D5)
  - Active Buzzer (A0/D14)

  Features:
  - PID temperature control
  - Multiple filament drying profiles
  - Real-time temperature monitoring
  - Safety features (overheat protection, sensor fault detection)
  - Audio feedback via buzzer
*/

#include <Arduino.h>
#include <avr/wdt.h>
#include <util/delay.h>
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
#ifdef DRYBOX_DEBUG
// Serial costs ~200 B of RAM; halve the frame buffer so the debug build still fits
#undef SCREEN_HEIGHT
#define SCREEN_HEIGHT 32
#endif
#define OLED_ADDR 0x3C

// Pin Definitions
const uint8_t PIN_HEATER = 3;    // PWM MOSFET gate (Timer2)
const uint8_t PIN_FAN    = 5;    // MOSFET gate (ON if heaterPWM>0)
const uint8_t PIN_BUZZ   = A0;   // D14 (active buzzer)
// Buttons sit in a 2x2 grid:   [1 UP  ] [2 BACK]
//                              [3 DOWN] [4 OK  ]
const uint8_t BTN_UP     = A1;   // top-left,     digital-capable (INPUT_PULLUP)
const uint8_t BTN_DOWN   = A3;   // bottom-left,  digital-capable (INPUT_PULLUP)
const uint8_t BTN_BACK   = A7;   // top-right,    analog-only (needs external 10k pull-up to 5V)
const uint8_t BTN_OK     = A6;   // bottom-right, analog-only (needs external 10k pull-up to 5V)
const uint8_t PIN_THERM  = A2;   // thermistor divider

// Button Debounce
const unsigned long DEBOUNCE_MS = 30;

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
HeaterWatch heaterWatch;
bool heaterFault = false;  // latched "heater not responding" until BACK in FAULT

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
    digitalWrite(PIN_BUZZ, HIGH);
    delay(onMs);
    digitalWrite(PIN_BUZZ, LOW);
    delay(gapMs);
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
Button btnOk   = {BTN_OK,   true,  false, false, 0};
Button btnDown = {BTN_DOWN, false, false, false, 0};
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

// Fan: on for the whole cycle, then run-on after the heater was last active
// and for as long as the heater is still hot.
const unsigned long FAN_RUNON_MS = 180000UL;  // 3 min
const float FAN_COOLDOWN_C = 40.0;      // fan on above this...
const float FAN_COOLDOWN_OFF_C = 35.0;  // ...and off again below this
float lastTempC = NAN;  // last thermistor reading (NAN if invalid)
bool fanEverHeated = false;
bool fanHot = false;
unsigned long fanLastHeatMs = 0;

void applyOutputs() {
  analogWrite(PIN_HEATER, heaterPWM);

  unsigned long now = millis();
  bool heating = (heaterPWM > 0) || (mode == RUN);
  if (heating) {
    fanEverHeated = true;
    fanLastHeatMs = now;
  }
  if (isnan(lastTempC) || lastTempC < FAN_COOLDOWN_OFF_C) fanHot = false;
  else if (lastTempC > FAN_COOLDOWN_C) fanHot = true;
  bool fan = heating
          || (fanEverHeated && (now - fanLastHeatMs) < FAN_RUNON_MS)
          || fanHot;
  digitalWrite(PIN_FAN, fan ? HIGH : LOW);
}

// ======== Watchdog ========
// Interrupt-only mode (no reset): the old Nano bootloader does not clear the
// watchdog and would reset-loop forever. If loop() stalls for 4 s the ISR forces
// heater OFF and fan ON and beeps until power is cycled.
// Direct port access: D3 = PD3 (PIN_HEATER), D5 = PD5 (PIN_FAN), A0 = PC0 (PIN_BUZZ).
ISR(WDT_vect) {
  TCCR2A &= ~_BV(COM2B1);  // detach Timer2 PWM from D3
  PORTD &= ~_BV(PD3);
  PORTD |= _BV(PD5);
#ifdef DRYBOX_DEBUG
  const char *m = "\nWDT d3=";
  while (*m) { while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = *m++; }
  while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = '0' + ((PIND >> 3) & 1);
  while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = '0' + ((PIND >> 5) & 1);
  while (!(UCSR0A & _BV(UDRE0))) {} UDR0 = '\n';
#endif
  for (;;) {
    PORTC |= _BV(PC0);
    _delay_ms(100);
    PORTC &= ~_BV(PC0);
    _delay_ms(900);
  }
}

void watchdogBegin() {
  cli();
  wdt_reset();
  MCUSR &= ~_BV(WDRF);
  WDTCSR |= _BV(WDCE) | _BV(WDE);
  WDTCSR = _BV(WDIE) | _BV(WDP3);  // interrupt only, 4 s
  sei();
}

// ======== Display Functions ========
// Template: accepts both F("...") literals and RAM strings (profile names)
// Two-colour OLED: rows 0-15 are the yellow band, rows 16-63 blue.
// The header stays inside the yellow band; all other text starts at row 16 or below.
template <typename T> void drawHeader(T line) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 3);
  display.print(line);
  display.drawLine(0, 13, 127, 13, SSD1306_WHITE);
}

void drawIdle(float tH, float tA, float rh) {
  display.clearDisplay();
  drawHeader(F("Drybox Monitor"));

  display.setCursor(0, 18);
  display.print(F("Heater "));
  if (isnan(tH)) display.print(F("--.-C"));
  else { display.print(tH, 1); display.print(F("C")); }

  display.setCursor(0, 30);
  display.print(F("Ambient "));
  if (isnan(tA)) display.print(F("--.-C"));
  else { display.print(tA, 1); display.print(F("C")); }

  display.setCursor(0, 42);
  display.print(F("Humidity "));
  if (isnan(rh)) display.print(F("--%"));
  else { display.print(rh, 0); display.print(F("%")); }

  display.setCursor(0, 56);
  display.print(F("OK=Menu"));

  display.display();
}

void drawMenu() {
  display.clearDisplay();
  drawHeader(F("Select Profile"));

  for (uint8_t i = 0; i < NPROF; i++) {
    display.setCursor(8, 17 + i * 9);
    if (i == sel) display.print(F("> "));
    else display.print(F("  "));
    display.print(profiles[i].name);
    display.print(F(" "));
    display.print(profiles[i].setC, 0);
    display.print(F("C/"));
    display.print(profiles[i].duration_s / 3600);
    display.print(F("h"));
  }

  display.display();
}

void drawRun(float tH, uint32_t left) {
  display.clearDisplay();
  drawHeader(activePro.name);

  display.setCursor(0, 18);
  display.print(F("Set "));
  display.print(activePro.setC, 0);
  display.print(F("C  PWM "));
  display.print((heaterPWM * 100) / 255);
  display.print(F("%"));

  display.setCursor(0, 30);
  display.print(F("Temp "));
  if (isnan(tH)) display.print(F("--.-C"));
  else { display.print(tH, 1); display.print(F("C")); }

  display.setCursor(0, 42);
  uint32_t h = left / 3600;
  uint32_t m = (left % 3600) / 60;
  uint32_t s = left % 60;
  display.print(F("Time "));
  display.print(h);
  display.print(F("h "));
  display.print(m);
  display.print(F("m "));
  display.print(s);
  display.print(F("s"));

  display.setCursor(0, 56);
  display.print(F("BACK=Abort"));

  display.display();
}

void drawDone() {
  display.clearDisplay();
  drawHeader(F("Cycle Complete!"));

  display.setCursor(0, 30);
  display.print(activePro.name);
  display.print(F(" finished"));

  display.setCursor(0, 42);
  display.print(F("Heater OFF"));

  display.setCursor(0, 56);
  display.print(F("OK=Menu BACK=Idle"));

  display.display();
}

void drawFault(const __FlashStringHelper* msg) {
  display.clearDisplay();
  drawHeader(F("FAULT!"));

  display.setCursor(0, 30);
  display.print(msg);

  display.setCursor(0, 42);
  display.print(F("Heater OFF"));

  display.setCursor(0, 56);
  display.print(F("BACK=Idle"));

  display.display();
}

// ======== Debug / bench-test build (-DDRYBOX_DEBUG) ========
// Serial telemetry + injected buttons/temperature so the state machine can be
// exercised on a bare Nano. Not compiled into the production firmware.
#ifdef DRYBOX_DEBUG
extern char *__brkval;
extern char __heap_start;
static bool dbgSim = false;        // true: ignore physical buttons
static bool dbgTempOn = false;     // true: override thermistor reading
static float dbgTemp = NAN;
static uint8_t dbgBtn = 0;         // injected presses: 1=UP 2=OK 4=DOWN 8=BACK
static unsigned long dbgLastPrint = 0, dbgLoopMax = 0, dbgLoopPrev = 0;

static void dbgPaintStack() {
  char *p = __brkval ? __brkval : &__heap_start;
  char *sp = (char *)SP - 32;
  while (p < sp) *p++ = (char)0xA5;
}
static uint16_t dbgStackFree() {
  char *p = __brkval ? __brkval : &__heap_start;
  uint16_t n = 0;
  while (*p == (char)0xA5) { p++; n++; }
  return n;
}
// Supply voltage in mV, measured against the internal 1.1 V reference
static uint16_t dbgVccMv() {
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);
  ADCSRA |= _BV(ADSC);
  while (ADCSRA & _BV(ADSC)) {}
  return (uint16_t)(1125300L / ADC);
}
static void dbgCommands() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'u': dbgBtn |= 1; break;
      case 'o': dbgBtn |= 2; break;
      case 'd': dbgBtn |= 4; break;
      case 'b': dbgBtn |= 8; break;
      case 's': dbgSim = true; break;
      case 'r': dbgSim = false; dbgTempOn = false; break;
      case 't': dbgTemp = Serial.parseFloat(); dbgTempOn = true; break;
      case 'n': dbgTemp = NAN; dbgTempOn = true; break;
      case 'f': tStart -= 3600000UL; break;  // fast-forward cycle by 1 h
      case 'x': for (;;) {}                  // stall loop() to test the watchdog
      case 'p':                              // probe D2..D7 for pull-downs (MOSFET gates); run with 12V OFF
        for (uint8_t pin = 2; pin <= 7; pin++) {
          uint8_t lo = 0, hi = 0;
          pinMode(pin, INPUT);
          for (uint8_t i = 0; i < 16; i++) { delay(2); lo += digitalRead(pin); }
          pinMode(pin, INPUT_PULLUP);
          for (uint8_t i = 0; i < 16; i++) { delay(2); hi += digitalRead(pin); }
          pinMode(pin, INPUT);
          Serial.print(F("PROBE D")); Serial.print(pin);
          Serial.print(F(" float=")); Serial.print(lo);
          Serial.print(F("/16 pullup=")); Serial.print(hi); Serial.println(F("/16"));
        }
        pinMode(PIN_HEATER, OUTPUT);
        pinMode(PIN_FAN, OUTPUT);
        break;
    }
  }
}
static void dbgReport(float tH, float tA, float rh) {
  unsigned long now = millis();
  unsigned long dt = now - dbgLoopPrev;
  dbgLoopPrev = now;
  if (dt > dbgLoopMax) dbgLoopMax = dt;
  if (now - dbgLastPrint < 500) return;
  dbgLastPrint = now;
  Serial.print(F("mode=")); Serial.print((int)mode);
  Serial.print(F(" sel=")); Serial.print(sel);
  Serial.print(F(" tH=")); Serial.print(tH, 1);
  Serial.print(F(" tA=")); Serial.print(tA, 1);
  Serial.print(F(" rh=")); Serial.print(rh, 0);
  Serial.print(F(" adcT=")); Serial.print(analogRead(PIN_THERM));
  Serial.print(F(" a1=")); Serial.print(digitalRead(BTN_UP));
  Serial.print(F(" a3=")); Serial.print(digitalRead(BTN_DOWN));
  Serial.print(F(" a6=")); Serial.print(analogRead(BTN_OK));
  Serial.print(F(" a7=")); Serial.print(analogRead(BTN_BACK));
  Serial.print(F(" pwm=")); Serial.print(heaterPWM);
  Serial.print(F(" ocr2b=")); Serial.print(OCR2B);
  Serial.print(F(" com2b=")); Serial.print((TCCR2A >> COM2B1) & 1);
  Serial.print(F(" d3=")); Serial.print((PIND >> 3) & 1);
  Serial.print(F(" d5=")); Serial.print((PIND >> 5) & 1);
  Serial.print(F(" bz=")); Serial.print(PINC & 1);
  Serial.print(F(" hf=")); Serial.print(heaterFault);
  Serial.print(F(" vcc=")); Serial.print(dbgVccMv());
  Serial.print(F(" left="));
  Serial.print(mode == RUN ? (long)activePro.duration_s - (long)((now - tStart) / 1000UL) : -1L);
  Serial.print(F(" stkFree=")); Serial.print(dbgStackFree());
  Serial.print(F(" stkUsed="));
  Serial.print((uint16_t)(RAMEND + 1 - (uint16_t)(__brkval ? __brkval : &__heap_start) - dbgStackFree()));
  Serial.print(F(" loopMax=")); Serial.println(dbgLoopMax);
  dbgLoopMax = 0;
}
#endif

// ======== Setup ========
void setup() {
#ifdef DRYBOX_DEBUG
  Serial.begin(115200);
  Serial.setTimeout(50);
  Serial.println(F("BOOT"));
#endif
  // Initialize outputs
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  pinMode(PIN_BUZZ, OUTPUT);

  analogWrite(PIN_HEATER, 0);
  digitalWrite(PIN_FAN, LOW);
  digitalWrite(PIN_BUZZ, LOW);

  // Initialize buttons
  pinMode(BTN_UP, INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  // A6/A7 are analog-only, need external 10k pull-ups

  // Initialize display
  Wire.begin();
  // Without a timeout a stuck I2C bus blocks loop() forever with the heater PWM latched
  Wire.setWireTimeout(25000, true);
  // display.begin() only fails on malloc, so check that the OLED actually answers
  Wire.beginTransmission(OLED_ADDR);
  bool oledOk = (Wire.endTransmission() == 0);
#ifdef DRYBOX_DEBUG
  if (!oledOk) Serial.println(F("NO_OLED"));
  oledOk = true;  // bench build runs without a display
#endif
  if (!oledOk || !display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // Display init failed - emergency stop
    while (1) {
      analogWrite(PIN_HEATER, 0);
      digitalWrite(PIN_FAN, LOW);
      digitalWrite(PIN_BUZZ, HIGH);
      delay(100);
      digitalWrite(PIN_BUZZ, LOW);
      delay(400);
    }
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 24);
  display.print(F("Drybox Controller"));
  display.setCursor(0, 36);
  display.print(F("Initializing..."));
  display.display();
  delay(1000);

  // Initialize DHT sensor
  dht.begin();

  // Initialize PID
  pid.setOutputLimits(0, 255);

  // Startup beep
  buzzMulti(2, 100, 100);

  watchdogBegin();
#ifdef DRYBOX_DEBUG
  dbgPaintStack();
  Serial.println(F("SETUP_DONE"));
#endif
}

// ======== Main Loop ========
void loop() {
  // Read sensors
  wdt_reset();

  float tH = readThermC();  // heater/chamber temp
#ifdef DRYBOX_DEBUG
  if (dbgTempOn) tH = dbgTemp;
#endif
  lastTempC = tH;

  float tA = NAN, rh = NAN;  // ambient temp and humidity
#if USE_DHT
  tA = dht.readTemperature();
  rh = dht.readHumidity();
#endif

  // Read buttons
  bool up = updateButton(btnUp);
  bool ok = updateButton(btnOk);
  bool dn = updateButton(btnDown);
  bool back = updateButton(btnBack);
#ifdef DRYBOX_DEBUG
  dbgCommands();
  if (dbgSim) up = ok = dn = back = false;
  if (dbgBtn & 1) up = true;
  if (dbgBtn & 2) ok = true;
  if (dbgBtn & 4) dn = true;
  if (dbgBtn & 8) back = true;
  dbgBtn = 0;
#endif

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
        heaterWatch.reset();
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
      if (mode == RUN && heaterWatch.update(heaterPWM >= 250, tH, activePro.setC, millis())) {
        heaterPWM = 0;
        applyOutputs();
        heaterFault = true;
        mode = FAULT;
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
        heaterFault = false;
        buzz(40);
        mode = IDLE;
      }
      drawFault(heaterFault ? F("Heater no response")
                : safety.isThermistorFaulted() ? F("Thermistor error") : F("Overtemperature"));
      break;
    }
  }

  // Keep heater/fan outputs current in every state (fan run-on)
  applyOutputs();

  // Service buzzer
  buzzService();
#ifdef DRYBOX_DEBUG
  dbgReport(tH, tA, rh);
#endif

  delay(40);
}
