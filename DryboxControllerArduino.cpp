/*
  Drybox Controller – Arduino Nano
  Heater MOSFET: D3
  Fan MOSFET:    D5 (auto on with heater)
  Buttons:       A1 (UP), A3 (OK), A6 (DOWN, analog-only), A7 (BACK, analog-only)
  OLED I2C:      SDA=A4, SCL=A5 (SSD1306 128x64 at 0x3C)
  Thermistor:    A2 (100k NTC via voltage divider, see wiring notes)

  NOTES:
  - A6/A7 on Arduino Nano are ANALOG-ONLY. Use external 10k pull-ups to 5V and read via analog.
  - DHT11 is optional for now. Enable later by setting USE_DHT = 1 and picking a real pin.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======== USER CONFIG ========

// --- DHT11 (optional for now) ---
#define USE_DHT 0        // set to 1 after you connect the data pin
#define DHT_PIN 2        // placeholder; choose any free DIGITAL pin later
#define DHT_TYPE 11      // DHT11

#if USE_DHT
#include <DHT.h>
DHT dht(DHT_PIN, DHT_TYPE);
#endif

// --- Display ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
const uint8_t OLED_ADDR = 0x3C;

// --- Pins ---
const uint8_t PIN_HEATER = 3;   // MOSFET gate (logic-level N-MOSFET)
const uint8_t PIN_FAN    = 5;   // MOSFET gate (fan)
const uint8_t BTN_UP_PIN   = A1;  // digital-capable
const uint8_t BTN_OK_PIN   = A3;  // digital-capable
const uint8_t BTN_DOWN_PIN = A6;  // analog-only -> external pull-up
const uint8_t BTN_BACK_PIN = A7;  // analog-only -> external pull-up
const uint8_t THERM_PIN    = A2;

// --- Thermistor (100k NTC typical for 3D printers) ---
const float SERIES_RESISTOR     = 100000.0; // 100k fixed resistor
const float NOMINAL_RESISTANCE  = 100000.0; // 100k at 25°C
const float BETA_COEFFICIENT    = 3950.0;   // beta value
const float NOMINAL_TEMPERATURE = 25.0;     // °C
const float ADC_COUNTS          = 1023.0;   // 10-bit
const float HYSTERESIS_C        = 2.0;      // ±2°C around setpoint

// Safety limits
const float MAX_SAFE_TEMP_C = 90.0;  // Hard cutoff to protect filament/box
const unsigned SENSOR_FAULT_SAMPLES = 4;

// --- Button debounce ---
const unsigned long DEBOUNCE_MS = 30;

// --- Graph/logging ---
const uint16_t LOG_LEN = 120; // matches screen width nicely
float logTemp[LOG_LEN];
float logHum [LOG_LEN];
uint16_t logIndex = 0;

// --- Profiles ---
struct Profile {
  const char *name;
  float setC;           // heater setpoint (°C)
  uint32_t duration_s;  // total time (sec)
};

Profile profiles[] = {
  {"PLA",   45.0, 4UL*3600UL},
  {"PETG",  65.0, 6UL*3600UL},
  {"ABS",   65.0, 6UL*3600UL},
  {"TPU",   50.0, 6UL*3600UL},
  {"Nylon", 75.0,12UL*3600UL}
};
const uint8_t PROFILE_COUNT = sizeof(profiles)/sizeof(profiles[0]);

// --- State machine ---
enum Mode { IDLE, MENU, RUN, DONE, FAULT };
Mode mode = IDLE;
uint8_t menuIndex = 0;
uint32_t runStart = 0;
Profile current;

// --- Button handling (supports digital pins and A6/A7 analog-only) ---
struct Button {
  uint8_t pin;
  bool analogOnly;
  bool stable;      // debounced state (true = pressed)
  bool lastRaw;     // raw state
  unsigned long lastChange;
};

Button btnUp   = {BTN_UP_PIN,   false, false, false, 0};
Button btnOk   = {BTN_OK_PIN,   false, false, false, 0};
Button btnDown = {BTN_DOWN_PIN, true,  false, false, 0};
Button btnBack = {BTN_BACK_PIN, true,  false, false, 0};

// Helper: read raw (un-debounced) pressed/not-pressed
bool rawPressed(Button &b) {
  if (b.analogOnly) {
    // Expect external 10k pull-up to 5V, button to GND. Press => low voltage.
    int v = analogRead(b.pin); // 0..1023
    return v < 200;            // threshold ~ <1V
  } else {
    return digitalRead(b.pin) == LOW; // using INPUT_PULLUP
  }
}

// Debounce; returns true once when a new press is detected
bool updateButton(Button &b) {
  bool reading = rawPressed(b);
  if (reading != b.lastRaw) {
    b.lastChange = millis();
    b.lastRaw = reading;
  }
  if ((millis() - b.lastChange) > DEBOUNCE_MS) {
    if (b.stable != reading) {
      b.stable = reading;
      if (b.stable) return true; // new press event
    }
  }
  return false;
}

// ======== Thermistor helpers ========
bool thermFault = false;
uint8_t faultStreak = 0;

float readThermistorC() {
  int adc = analogRead(THERM_PIN);
  // Detect obvious faults
  if (adc <= 0 || adc >= 1023) {
    if (++faultStreak >= SENSOR_FAULT_SAMPLES) thermFault = true;
    return NAN;
  }
  faultStreak = 0; thermFault = false;

  // Voltage divider: 5V -- [Series R] --(A2)-- [Thermistor] -- GND
  // Rtherm = SERIES_R * (ADCmax/adc - 1)
  float rTherm = SERIES_RESISTOR * (ADC_COUNTS / (float)adc - 1.0);

  // Beta equation:
  float steinhart = rTherm / NOMINAL_RESISTANCE;   // (R/R0)
  steinhart = log(steinhart);                      // ln(R/R0)
  steinhart /= BETA_COEFFICIENT;                   // 1/B * ln(...)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15); // + (1/T0)
  steinhart = 1.0 / steinhart;                     // invert
  float tempC = steinhart - 273.15;                // K -> °C
  return tempC;
}

// ======== Control ========
bool heaterOn = false;
float setC = 0;

void setHeater(bool on) {
  heaterOn = on && !thermFault;
  digitalWrite(PIN_HEATER, heaterOn ? HIGH : LOW);
  // Fan follows heater automatically
  digitalWrite(PIN_FAN, heaterOn ? HIGH : LOW);
}

// ======== Display ========
void drawHeader(const char* line) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);
  display.print(line);
  display.drawLine(0,9,127,9,SSD1306_WHITE);
}

void drawDashboard(float heaterC, float ambC, float ambRH) {
  display.clearDisplay();
  drawHeader("Drybox Monitor");

  display.setCursor(0,12);
  display.print("Heater: ");
  if (isnan(heaterC)) display.print("--.-");
  else { display.print(heaterC,1); display.print("C"); }
  display.print(heaterOn ? "  ON" : "  OFF");

  display.setCursor(0,24);
  display.print("Ambient: ");
  if (isnan(ambC)) display.print("--.-C ");
  else { display.print(ambC,1); display.print("C "); }
  display.print("RH ");
  if (isnan(ambRH)) display.print("--");
  else display.print(ambRH,0);
  display.print("%");

  // Plot: bottom 28px area
  int y0 = 63;
  // temp graph (left axis ~ 20..90C mapped to 63..36)
  for (int x=0; x<SCREEN_WIDTH-1; x++) {
    int i1 = (logIndex + x) % LOG_LEN;
    int i2 = (logIndex + x + 1) % LOG_LEN;
    float t1 = logTemp[i1];
    float t2 = logTemp[i2];
    if (!isnan(t1) && !isnan(t2)) {
      int ty1 = map(constrain((int)(t1*10), 200, 900), 200, 900, y0, 36);
      int ty2 = map(constrain((int)(t2*10), 200, 900), 200, 900, y0, 36);
      display.drawLine(x,ty1,x+1,ty2,SSD1306_WHITE);
    }
    // humidity dots (right axis ~ 0..100% mapped to 63..36)
    float h2 = logHum[i2];
    if (!isnan(h2)) {
      int hy = map(constrain((int)(h2), 0, 100), 0, 100, y0, 36);
      display.drawPixel(x+1, hy, SSD1306_WHITE);
    }
  }

  display.display();
}

void drawMenu() {
  display.clearDisplay();
  drawHeader("Select Profile");
  for (uint8_t i=0;i<PROFILE_COUNT;i++){
    display.setCursor(8, 12 + i*10);
    if (i==menuIndex) display.print("> ");
    else display.print("  ");
    display.print(profiles[i].name);
    display.print(" ");
    display.print(profiles[i].setC,0);
    display.print("C/");
    display.print(profiles[i].duration_s/3600);
    display.print("h");
  }
  display.display();
}

void drawRun(float heaterC, uint32_t secsLeft) {
  display.clearDisplay();
  drawHeader(current.name);
  display.setCursor(0,12);
  display.print("Set ");
  display.print(current.setC,0); display.print("C  ");
  display.print(heaterOn ? "ON" : "OFF");

  display.setCursor(0,24);
  display.print("Temp ");
  if (isnan(heaterC)) display.print("--.-C");
  else { display.print(heaterC,1); display.print("C "); }
  display.setCursor(0,36);
  display.print("Time ");
  uint32_t m = secsLeft/60; uint32_t s = secsLeft%60;
  display.print(m); display.print("m ");
  display.print(s); display.print("s");

  display.setCursor(0,48);
  display.print("BACK to abort");
  display.display();
}

void drawDone() {
  display.clearDisplay();
  drawHeader("Cycle complete");
  display.setCursor(0,20); display.print(current.name);
  display.setCursor(0,32); display.print("Heater OFF, Fan OFF");
  display.setCursor(0,44); display.print("OK=menu  BACK=idle");
  display.display();
}

void drawFault(const char* msg) {
  display.clearDisplay();
  drawHeader("FAULT");
  display.setCursor(0,20); display.print(msg);
  display.setCursor(0,32); display.print("Heater OFF");
  display.setCursor(0,44); display.print("BACK=idle");
  display.display();
}

// ======== Setup/Loop ========
void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  setHeater(false);

  // Buttons
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);
  // A6/A7 analog-only: require external 10k pull-ups; no pinMode available
  // (analogRead works without pinMode)
  
  // Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    // If the display isn't found, fail safe
    while (1) { digitalWrite(PIN_HEATER, LOW); digitalWrite(PIN_FAN, LOW); }
  }
  display.clearDisplay(); display.display();

#if USE_DHT
  dht.begin();
#endif

  // Pre-fill logs with NaNs
  for (uint16_t i=0;i<LOG_LEN;i++){ logTemp[i]=NAN; logHum[i]=NAN; }
}

void loop() {
  // --- Read sensors ---
  float heaterC = readThermistorC();

  float ambC = NAN, ambRH = NAN;
#if USE_DHT
  ambC  = dht.readTemperature();
  ambRH = dht.readHumidity();
#endif

  // Update logs (1 sample per loop, ~few Hz)
  logTemp[logIndex] = heaterC;
  logHum [logIndex] = ambRH;
  logIndex = (logIndex + 1) % LOG_LEN;

  // --- Buttons ---
  bool up   = updateButton(btnUp);
  bool ok   = updateButton(btnOk);
  bool down = updateButton(btnDown);
  bool back = updateButton(btnBack);

  // --- Safety ---
  if (!isnan(heaterC) && heaterC > MAX_SAFE_TEMP_C) {
    setHeater(false);
    mode = FAULT;
  }
  if (thermFault) { setHeater(false); mode = FAULT; }

  // --- State machine ---
  switch (mode) {
    case IDLE:
      setHeater(false);
      if (ok) { mode = MENU; }
      drawDashboard(heaterC, ambC, ambRH);
      break;

    case MENU:
      if (up   && menuIndex>0) menuIndex--;
      if (down && menuIndex<PROFILE_COUNT-1) menuIndex++;
      if (ok) {
        current = profiles[menuIndex];
        setC = current.setC;
        runStart = millis();
        mode = RUN;
      }
      if (back) mode = IDLE;
      drawMenu();
      break;

    case RUN: {
      // Simple hysteresis control
      if (!isnan(heaterC)) {
        if (!heaterOn && heaterC < setC - HYSTERESIS_C) setHeater(true);
        if ( heaterOn && heaterC > setC + HYSTERESIS_C) setHeater(false);
      } else {
        setHeater(false);
      }

      uint32_t elapsed = (millis() - runStart)/1000UL;
      uint32_t left = (current.duration_s > elapsed) ? (current.duration_s - elapsed) : 0;
      if (left == 0) { setHeater(false); mode = DONE; }
      if (back) { setHeater(false); mode = IDLE; }
      drawRun(heaterC, left);
    } break;

    case DONE:
      setHeater(false);
      if (ok)   mode = MENU;
      if (back) mode = IDLE;
      drawDone();
      break;

    case FAULT:
      setHeater(false);
      if (back) mode = IDLE;
      drawFault(thermFault ? "Thermistor error" : "Overtemperature");
      break;
  }

  delay(60); // ~16 Hz loop; OLED refresh stays smooth
}

