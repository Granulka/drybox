/*
  Drybox Controller – Arduino Nano, PID heater control (PWM on D3)

  Heater MOSFET (PWM): D3
  Fan MOSFET:          D5 (auto on when heater_output > 0)
  Buttons:             A1(UP), A3(OK) -> INPUT_PULLUP
                       A6(DOWN), A7(BACK) -> analog-only, need external 10k pull-ups to 5V (press=GND)
  OLED I2C:            SDA=A4, SCL=A5 (SSD1306 128x64 at 0x3C)
  Thermistor:          A2 (100k NTC via voltage divider: 5V—100k—A2—NTC—GND)
  DHT11 (optional):    any digital pin later; disabled by default (USE_DHT=0)

  Safety:
  - Thermistor fault detection
  - Max safe temp cutoff

  Notes:
  - PID is implemented here (no external library), with derivative on measurement to reduce kick.
  - Tune Kp, Ki, Kd below. Start with modest Kp, small Ki, small Kd.
*/

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ======== USER CONFIG ========

// --- DHT11 (optional for now) ---
#define USE_DHT 0        // set to 1 after you connect the data pin & install the Adafruit DHT library
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
const uint8_t PIN_HEATER = 3;   // PWM MOSFET gate (Timer2)
const uint8_t PIN_FAN    = 5;   // MOSFET gate (digital on/off)
const uint8_t BTN_UP_PIN   = A1;  // digital-capable
const uint8_t BTN_OK_PIN   = A3;  // digital-capable
const uint8_t BTN_DOWN_PIN = A6;  // analog-only -> external pull-up
const uint8_t BTN_BACK_PIN = A7;  // analog-only -> external pull-up
const uint8_t THERM_PIN    = A2;

// --- Thermistor (100k NTC typical) ---
const float SERIES_RESISTOR     = 100000.0; // 100k fixed resistor
const float NOMINAL_RESISTANCE  = 100000.0; // 100k at 25°C
const float BETA_COEFFICIENT    = 3950.0;   // beta value
const float NOMINAL_TEMPERATURE = 25.0;     // °C
const float ADC_COUNTS          = 1023.0;   // 10-bit

// Safety limits
const float MAX_SAFE_TEMP_C = 90.0;  // Hard cutoff
const unsigned SENSOR_FAULT_SAMPLES = 4;

// --- Button debounce ---
const unsigned long DEBOUNCE_MS = 30;

// --- Graph/logging ---
const uint16_t LOG_LEN = 120;
float logTemp[LOG_LEN];
float logHum [LOG_LEN];
uint16_t logIndex = 0;

// --- Profiles ---
struct Profile {
  const char *name;
  float setC;           // setpoint °C
  uint32_t duration_s;  // seconds
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

// ======== PID (no external library) ========
struct SimplePID {
  double Kp=8.0, Ki=0.04, Kd=30.0;    // <-- TUNE ME
  double setpoint=0.0;
  double outMin=0.0, outMax=255.0;
  double integral=0.0;
  double lastInput=0.0;
  double lastOutput=0.0;
  bool   initialized=false;
  unsigned long lastTime=0;
  unsigned long sampleTimeMs=1000;    // 1 s

  void setTunings(double kp,double ki,double kd){Kp=kp;Ki=ki;Kd=kd;}
  void setOutputLimits(double mn,double mx){outMin=mn;outMax=mx;}
  void setSampleTimeMs(unsigned long ms){sampleTimeMs=ms;}

  // Returns true if a new output was computed (time step elapsed)
  bool compute(double input, double &out) {
    unsigned long now = millis();
    if (!initialized) {
      lastTime = now;
      lastInput = input;
      lastOutput = 0.0;
      initialized = true;
      out = lastOutput;
      return true;
    }
    unsigned long dtMs = now - lastTime;
    if (dtMs < sampleTimeMs) { out = lastOutput; return false; }
    double dt = dtMs/1000.0;

    double error = setpoint - input;

    // Integrator with simple anti-windup
    integral += (Ki * error * dt);
    if (integral > outMax) integral = outMax;
    if (integral < outMin) integral = outMin;

    // Derivative on measurement to reduce kick
    double dInput = (input - lastInput) / dt;

    double output = Kp*error + integral - Kd*dInput;

    // Clamp
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    lastOutput = output;
    lastInput = input;
    lastTime = now;
    out = output;
    return true;
  }

  void reset(double input, double currentOutput=0.0){
    initialized=false; integral=0.0; lastInput=input; lastOutput=currentOutput;
  }
};
SimplePID heaterPID;

// ======== Button handling ========
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

bool rawPressed(Button &b) {
  if (b.analogOnly) {
    int v = analogRead(b.pin);  // 0..1023, pull-up to 5V means idle ~1023
    return v < 200;             // pressed ~ near 0 -> threshold
  } else {
    return digitalRead(b.pin) == LOW; // INPUT_PULLUP
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
      if (b.stable) return true; // on rising edge (pressed)
    }
  }
  return false;
}

// ======== Thermistor helpers ========
bool thermFault = false;
uint8_t faultStreak = 0;

float readThermistorC() {
  int adc = analogRead(THERM_PIN);
  if (adc <= 0 || adc >= 1023) {
    if (++faultStreak >= SENSOR_FAULT_SAMPLES) thermFault = true;
    return NAN;
  }
  faultStreak = 0; thermFault = false;

  // Voltage divider: 5V -- [Series R] --(A2)-- [Thermistor] -- GND
  float rTherm = SERIES_RESISTOR * (ADC_COUNTS / (float)adc - 1.0);

  // Beta equation
  float steinhart = rTherm / NOMINAL_RESISTANCE;         // (R/R0)
  steinhart = log(steinhart);                            // ln(R/R0)
  steinhart /= BETA_COEFFICIENT;                         // 1/B * ln(...)
  steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);     // + (1/T0)
  steinhart = 1.0 / steinhart;                           // invert
  return steinhart - 273.15;                             // K → °C
}

// ======== Control wrappers ========
int heaterPWM = 0;   // 0..255

void applyOutputs() {
  // Heater PWM (Timer2 on D3)
  analogWrite(PIN_HEATER, heaterPWM);
  // Fan follows heater
  digitalWrite(PIN_FAN, (heaterPWM > 0) ? HIGH : LOW);
}

// ======== Display helpers ========
Adafruit_SSD1306 *d = &display;

void drawHeader(const char* line) {
  d->setTextSize(1);
  d->setTextColor(SSD1306_WHITE);
  d->setCursor(0,0);
  d->print(line);
  d->drawLine(0,9,127,9,SSD1306_WHITE);
}

void drawDashboard(float heaterC, float ambC, float ambRH) {
  d->clearDisplay();
  drawHeader("Drybox Monitor");

  d->setCursor(0,12);
  d->print("Heater ");
  if (isnan(heaterC)) d->print("--.-C");
  else { d->print(heaterC,1); d->print("C"); }
  d->print("  PWM ");
  d->print((heaterPWM*100)/255); d->print("%");

  d->setCursor(0,24);
  d->print("Ambient ");
  if (isnan(ambC)) d->print("--.-C ");
  else { d->print(ambC,1); d->print("C "); }
  d->print("RH ");
  if (isnan(ambRH)) d->print("--");
  else d->print(ambRH,0);
  d->print("%");

  // Plot: bottom 28px
  int y0 = 63;
  for (int x=0; x<SCREEN_WIDTH-1; x++) {
    int i1 = (logIndex + x) % LOG_LEN;
    int i2 = (logIndex + x + 1) % LOG_LEN;
    float t1 = logTemp[i1];
    float t2 = logTemp[i2];
    if (!isnan(t1) && !isnan(t2)) {
      int ty1 = map(constrain((int)(t1*10), 200, 900), 200, 900, y0, 36);
      int ty2 = map(constrain((int)(t2*10), 200, 900), 200, 900, y0, 36);
      d->drawLine(x,ty1,x+1,ty2,SSD1306_WHITE);
    }
    float h2 = logHum[i2];
    if (!isnan(h2)) {
      int hy = map(constrain((int)(h2), 0, 100), 0, 100, y0, 36);
      d->drawPixel(x+1, hy, SSD1306_WHITE);
    }
  }
  d->display();
}

void drawMenu() {
  d->clearDisplay();
  drawHeader("Select Profile");
  for (uint8_t i=0;i<PROFILE_COUNT;i++){
    d->setCursor(8, 12 + i*10);
    if (i==menuIndex) d->print("> ");
    else d->print("  ");
    d->print(profiles[i].name);
    d->print(" ");
    d->print(profiles[i].setC,0);
    d->print("C/");
    d->print(profiles[i].duration_s/3600);
    d->print("h");
  }
  d->display();
}

void drawRun(float heaterC, uint32_t secsLeft) {
  d->clearDisplay();
  drawHeader(current.name);
  d->setCursor(0,12);
  d->print("Set "); d->print(current.setC,0); d->print("C  ");
  d->print("PWM "); d->print((heaterPWM*100)/255); d->print("%");

  d->setCursor(0,24);
  d->print("Temp "); if (isnan(heaterC)) d->print("--.-C"); else { d->print(heaterC,1); d->print("C "); }
  d->setCursor(0,36);
  d->print("Time "); uint32_t m = secsLeft/60; uint32_t s = secsLeft%60; d->print(m); d->print("m "); d->print(s); d->print("s");
  d->setCursor(0,48); d->print("BACK to abort");
  d->display();
}

void drawDone() {
  d->clearDisplay();
  drawHeader("Cycle complete");
  d->setCursor(0,20); d->print(current.name);
  d->setCursor(0,32); d->print("Heater & Fan OFF");
  d->setCursor(0,44); d->print("OK=menu  BACK=idle");
  d->display();
}

void drawFault(const char* msg) {
  d->clearDisplay();
  drawHeader("FAULT");
  d->setCursor(0,20); d->print(msg);
  d->setCursor(0,32); d->print("Heater OFF");
  d->setCursor(0,44); d->print("BACK=idle");
  d->display();
}

// ======== Setup/Loop ========
enum Mode { IDLE2, MENU2, RUN2, DONE2, FAULT2 }; // avoid duplicate enum if re-paste
// (keeping the original 'Mode' enum above)

void setup() {
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  analogWrite(PIN_HEATER, 0);
  digitalWrite(PIN_FAN, LOW);

  // Buttons
  pinMode(BTN_UP_PIN, INPUT_PULLUP);
  pinMode(BTN_OK_PIN, INPUT_PULLUP);
  // A6/A7 analog-only; add external 10k pull-ups to 5V

  // Display
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (1) { analogWrite(PIN_HEATER, 0); digitalWrite(PIN_FAN, LOW); }
  }
  display.clearDisplay(); display.display();

#if USE_DHT
  dht.begin();
#endif

  // Pre-fill logs
  for (uint16_t i=0;i<LOG_LEN;i++){ logTemp[i]=NAN; logHum[i]=NAN; }

  // PID setup
  heaterPID.setOutputLimits(0,255);
  heaterPID.setSampleTimeMs(1000); // 1 s loop
  // heaterPID.setTunings(8.0, 0.04, 30.0); // already default; adjust as needed
}

void loop() {
  // --- Read sensors ---
  float heaterC = readThermistorC();

  float ambC = NAN, ambRH = NAN;
#if USE_DHT
  ambC  = dht.readTemperature();
  ambRH = dht.readHumidity();
#endif

  // Update logs
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
    heaterPWM = 0; applyOutputs(); mode = FAULT;
  }
  if (thermFault) { heaterPWM = 0; applyOutputs(); mode = FAULT; }

  // --- State machine ---
  switch (mode) {
    case IDLE:
      heaterPWM = 0; applyOutputs();
      if (ok) { mode = MENU; }
      drawDashboard(heaterC, ambC, ambRH);
      break;

    case MENU:
      if (up   && menuIndex>0) menuIndex--;
      if (down && menuIndex<PROFILE_COUNT-1) menuIndex++;
      if (ok) {
        current = profiles[menuIndex];
        heaterPID.setpoint = current.setC;
        heaterPID.reset(isnan(heaterC)? current.setC : heaterC, 0.0);
        runStart = millis();
        mode = RUN;
      }
      if (back) mode = IDLE;
      drawMenu();
      break;

    case RUN: {
      // Compute PID if we have a valid measurement
      if (!isnan(heaterC)) {
        double out;
        if (heaterPID.compute(heaterC, out)) {
          heaterPWM = (int)(out + 0.5);
          if (heaterPWM < 0) heaterPWM = 0;
          if (heaterPWM > 255) heaterPWM = 255;
          applyOutputs();
        }
      } else {
        heaterPWM = 0; applyOutputs();
      }

      uint32_t elapsed = (millis() - runStart)/1000UL;
      uint32_t left = (current.duration_s > elapsed) ? (current.duration_s - elapsed) : 0;
      if (left == 0) { heaterPWM = 0; applyOutputs(); mode = DONE; }
      if (back) { heaterPWM = 0; applyOutputs(); mode = IDLE; }
      drawRun(heaterC, left);
    } break;

    case DONE:
      heaterPWM = 0; applyOutputs();
      if (ok)   mode = MENU;
      if (back) mode = IDLE;
      drawDone();
      break;

    case FAULT:
      heaterPWM = 0; applyOutputs();
      if (back) mode = IDLE;
      drawFault(thermFault ? "Thermistor error" : "Overtemperature");
      break;
  }

  delay(60); // OLED update cadence, PID sample time is handled inside PID
}

