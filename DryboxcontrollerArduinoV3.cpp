/*
  Dry-box controller (Arduino Nano) with PID heater control and U8g2 OLED UI

  Heater MOSFET (PWM): D3
  Fan MOSFET:          D5 (auto ON when PWM>0)
  Buttons:             A1 (UP), A3 (DOWN)  -> INPUT_PULLUP
                       A6 (OK), A7 (BACK)  -> analog-only, need external 10k pull-ups to 5V (press=GND)
  OLED:                I2C SSD1306 128x64 (SDA=A4, SCL=A5)
  Thermistor:          A2 (100k NTC via divider: 5V—100k—A2—NTC—GND)
  DHT11 (optional):    any digital pin later; disabled by default (USE_DHT=0)

  PID: self-contained; tune Kp/Ki/Kd below. Sample time 1 s.
*/

#include <Arduino.h>
#include <Wire.h>
#include <U8g2lib.h>

// ======== DHT (OPTIONAL) ========
#define USE_DHT 0          // set to 1 after wiring the DHT and installing its library
#define DHT_PIN 4          // choose any free DIGITAL pin when you enable it
#define DHT_TYPE 11        // DHT11
#if USE_DHT
  #include <DHT.h>
  DHT dht(DHT_PIN, DHT_TYPE);
#endif

// ======== Display ========
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

// ======== Pins ========
const uint8_t PIN_HEATER = 3;    // PWM MOSFET gate (Timer2)
const uint8_t PIN_FAN    = 5;    // MOSFET gate (ON if heaterPWM>0)
const uint8_t BTN_UP     = A1;   // digital-capable
const uint8_t BTN_DOWN   = A3;   // digital-capable
const uint8_t BTN_OK     = A6;   // analog-only (needs external 10k pull-up)
const uint8_t BTN_BACK   = A7;   // analog-only (needs external 10k pull-up)
const uint8_t PIN_THERM  = A2;   // thermistor divider

// ======== Thermistor (100k NTC typical) ========
const float R_FIXED      = 100000.0;  // series resistor (ohms)
const float R_NOMINAL    = 100000.0;  // thermistor at 25°C (ohms)
const float T_NOMINAL_C  = 25.0;      // nominal temp (°C)
const float BETA         = 3950.0;    // beta coefficient
const float ADC_MAX      = 1023.0;

// Safety
const float MAX_SAFE_C   = 90.0;      // hard cutoff
const uint8_t FAULT_STREAK_N = 4;

// ======== Profiles ========
struct Profile { const char* name; float setC; uint32_t sec; };
Profile profiles[] = {
  {"PLA",   45, 2UL*3600UL},
  {"PETG",  65, 4UL*3600UL},
  {"ABS",   65, 4UL*3600UL},
  {"TPU",   50, 6UL*3600UL},
  {"Nylon", 75,12UL*3600UL}
};
const uint8_t NPROF = sizeof(profiles)/sizeof(profiles[0]);

// ======== History for graph ========
const uint8_t HIST = 64;
float histT[HIST];
float histH[HIST];

// ======== Buttons (debounce) ========
const unsigned long DEBOUNCE_MS = 30;
struct Btn {
  uint8_t pin; bool analogOnly;
  bool stable=false, lastRaw=false;
  unsigned long t=0;
};
Btn bUp   {BTN_UP,false}, bDown{BTN_DOWN,false}, bOk{BTN_OK,true}, bBack{BTN_BACK,true};

bool rawPressed(Btn& b) {
  if (!b.analogOnly) return digitalRead(b.pin)==LOW;          // INPUT_PULLUP
  int v = analogRead(b.pin);                                   // 0..1023
  return v < 200;                                              // requires external 10k pull-up
}
bool pressed(Btn& b) {
  bool r = rawPressed(b);
  if (r!=b.lastRaw){ b.lastRaw=r; b.t=millis(); }
  if (millis()-b.t > DEBOUNCE_MS) {
    if (b.stable!=r){ b.stable=r; if (b.stable) return true; }
  }
  return false;
}

// ======== PID ========
struct PID {
  double Kp=8.0, Ki=0.04, Kd=30.0;
  double sp=0, integ=0, lastIn=0, out=0;
  double outMin=0, outMax=255;
  unsigned long last=0, Ts=1000; // ms
  bool init=false;

  void set(double kp,double ki,double kd){Kp=kp;Ki=ki;Kd=kd;}
  void limits(double mn,double mx){outMin=mn;outMax=mx;}
  void sample(unsigned long ms){Ts=ms;}
  void reset(double in){init=false; integ=0; lastIn=in; out=0;}

  bool compute(double in){
    unsigned long now=millis(); if(!init){last=now; lastIn=in; init=true;}
    if (now-last < Ts) return false;
    double dt=(now-last)/1000.0; last=now;
    double e = sp - in;
    integ += Ki*e*dt; if(integ>outMax) integ=outMax; if(integ<outMin) integ=outMin;
    double dMeas = (in - lastIn)/dt; lastIn=in;
    out = Kp*e + integ - Kd*dMeas;
    if(out>outMax) out=outMax; if(out<outMin) out=outMin;
    return true;
  }
} pid;

// ======== State ========
enum Mode { IDLE, MENU, RUN, DONE, FAULT };
Mode mode = IDLE;
uint8_t sel = 0;
uint32_t tStart=0;
int heaterPWM=0;
bool thermFault=false; uint8_t faultStreak=0;

// ======== Helpers ========
float readThermC() {
  int adc = analogRead(PIN_THERM);
  if (adc<=0 || adc>=1023) { if(++faultStreak>=FAULT_STREAK_N) thermFault=true; return NAN; }
  faultStreak=0; thermFault=false;

  // Divider assumed: 5V — R_FIXED — A2 — NTC — GND
  // If you wired it reversed (5V—NTC—A2—R_FIXED—GND), swap this line for:
  //   float R = R_FIXED / (ADC_MAX/adc - 1.0);
  float R = R_FIXED * (ADC_MAX/adc - 1.0);

  double st = R / R_NOMINAL;
  st = log(st);
  st /= BETA;
  st += 1.0 / (T_NOMINAL_C + 273.15);
  st = 1.0 / st;
  return (float)(st - 273.15);
}

void pushHist(float t, float h){
  for (uint8_t i=0;i<HIST-1;i++){ histT[i]=histT[i+1]; histH[i]=histH[i+1]; }
  histT[HIST-1]=t; histH[HIST-1]=h;
}

void drawIdle(float tH, float RH, float tA){
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(0,8);  u8g2.print("Idle  PWM "); u8g2.print((heaterPWM*100)/255); u8g2.print("%");
  u8g2.setCursor(0,18); u8g2.print("Heater: "); if(!isnan(tH)){u8g2.print(tH,1);u8g2.print("C");} else u8g2.print("--");
  u8g2.setCursor(0,28); u8g2.print("RH: "); if(!isnan(RH)){u8g2.print(RH,0);u8g2.print("%");} else u8g2.print("--");
  u8g2.setCursor(0,38); u8g2.print("Ambient: "); if(!isnan(tA)){u8g2.print(tA,1);u8g2.print("C");} else u8g2.print("--");
  u8g2.setCursor(0,58); u8g2.print("OK=Menu");

  // mini-graph at right
  uint8_t gx=75, gy=0, gw=53, gh=32; u8g2.drawFrame(gx,gy,gw,gh);
  float tMin=1e6,tMax=-1e6,hMin=1e6,hMax=-1e6;
  for(uint8_t i=0;i<HIST;i++){ if(!isnan(histT[i])){tMin=min(tMin,histT[i]); tMax=max(tMax,histT[i]);}
                               if(!isnan(histH[i])){hMin=min(hMin,histH[i]); hMax=max(hMax,histH[i]);}}
  if (tMin==1e6){ tMin=20; tMax=90; hMin=0; hMax=100; }
  if (tMax-tMin<1) { tMax+=0.5; tMin-=0.5; }
  if (hMax-hMin<1) { hMax+=0.5; hMin-=0.5; }
  for(uint8_t x=1;x<gw-1;x++){
    uint8_t idx = map(x,1,gw-2,0,HIST-1);
    if(!isnan(histT[idx])){ float n=(histT[idx]-tMin)/(tMax-tMin); uint8_t y=gy+gh-1 - (uint8_t)(n*(gh-2)); u8g2.drawPixel(gx+x,y); }
    if(!isnan(histH[idx]) && (x&1)==0){ float n=(histH[idx]-hMin)/(hMax-hMin); uint8_t y=gy+gh-1-(uint8_t)(n*(gh-2)); u8g2.drawPixel(gx+x,y);}
  }
  u8g2.sendBuffer();
}

void drawMenu(){
  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(0,8); u8g2.print("Select Profile");
  for(uint8_t i=0;i<NPROF;i++){
    uint8_t y=20+i*10;
    if(i==sel){ u8g2.drawBox(0,y-7,128,9); u8g2.setDrawColor(0); }
    u8g2.setCursor(2,y); u8g2.print(profiles[i].name); u8g2.print(" ");
    u8g2.print((int)profiles[i].setC); u8g2.print("C ");
    u8g2.print(profiles[i].sec/3600); u8g2.print("h");
    if(i==sel) u8g2.setDrawColor(1);
  }
  u8g2.setCursor(0,62); u8g2.print("UP/DOWN, OK=start, BACK");
  u8g2.sendBuffer();
}

void drawRun(float tH, uint32_t left, float RH, float tA){
  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_5x8_tf);
  u8g2.setCursor(0,8);  u8g2.print("Drying "); u8g2.print(profiles[sel].name);
  u8g2.setCursor(0,18); u8g2.print("Set "); u8g2.print((int)profiles[sel].setC); u8g2.print("C PWM ");
  u8g2.print((heaterPWM*100)/255); u8g2.print("%");
  u8g2.setCursor(0,28); u8g2.print("Heater "); if(!isnan(tH)){u8g2.print(tH,1);u8g2.print("C");} else u8g2.print("--");
  u8g2.setCursor(0,38); u8g2.print("RH "); if(!isnan(RH)){u8g2.print(RH,0);u8g2.print("%");} else u8g2.print("--");
  u8g2.setCursor(0,48); u8g2.print("Amb "); if(!isnan(tA)){u8g2.print(tA,1);u8g2.print("C");} else u8g2.print("--");
  uint16_t m=left/60, s=left%60; u8g2.setCursor(0,58); u8g2.print("Time "); u8g2.print(m); u8g2.print("m "); u8g2.print(s); u8g2.print("s");
  u8g2.sendBuffer();
}

void drawDone(){
  u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setCursor(0,24); u8g2.print("Cycle complete");
  u8g2.setCursor(0,40); u8g2.print("OK=menu  BACK=idle");
  u8g2.sendBuffer();
}

void applyOutputs(){
  analogWrite(PIN_HEATER, heaterPWM);
  digitalWrite(PIN_FAN, heaterPWM>0 ? HIGH : LOW);
}

// ======== Setup / Loop ========
enum Mode mode2{IDLE, MENU, RUN, DONE, FAULT}; // satisfy some IDE quirk if re-pasted

void setup(){
  pinMode(PIN_HEATER, OUTPUT);
  pinMode(PIN_FAN, OUTPUT);
  analogWrite(PIN_HEATER,0);
  digitalWrite(PIN_FAN,LOW);

  pinMode(BTN_UP,   INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);
  // A6/A7 analog-only: no pinMode, require external 10k pull-ups to 5V

#if USE_DHT
  dht.begin();
#endif

  u8g2.begin();
  for(uint8_t i=0;i<HIST;i++){ histT[i]=NAN; histH[i]=NAN; }

  pid.limits(0,255);
  pid.sample(1000); // 1s
}

void loop(){
  // Sensors
  float tH = readThermC();
  float tA=NAN, RH=NAN;
#if USE_DHT
  tA  = dht.readTemperature();
  RH  = dht.readHumidity();
#endif

  // History once per second
  static unsigned long tHist=0;
  if(millis()-tHist>=1000){ pushHist(tH, RH); tHist=millis(); }

  // Buttons
  bool up=pressed(bUp), dn=pressed(bDown), ok=pressed(bOk), back=pressed(bBack);

  // Safety
  if(!isnan(tH) && tH>MAX_SAFE_C){ heaterPWM=0; applyOutputs(); mode=FAULT; }
  if(thermFault){ heaterPWM=0; applyOutputs(); mode=FAULT; }

  // State machine
  switch(mode){
    case IDLE:
      heaterPWM=0; applyOutputs();
      if(ok) mode=MENU;
      drawIdle(tH,RH,tA);
      break;

    case MENU:
      if(up && sel>0) sel--;
      if(dn && sel<NPROF-1) sel++;
      if(ok){
        pid.sp = profiles[sel].setC;
        pid.reset(isnan(tH)? pid.sp : tH);
        tStart = millis();
        mode = RUN;
      }
      if(back) mode=IDLE;
      drawMenu();
      break;

    case RUN: {
      if(!isnan(tH)){
        if(pid.compute(tH)){
          heaterPWM = (int)(pid.out + 0.5);
          if(heaterPWM<0) heaterPWM=0; if(heaterPWM>255) heaterPWM=255;
          applyOutputs();
        }
      } else {
        heaterPWM=0; applyOutputs();
      }
      uint32_t elapsed=(millis()-tStart)/1000UL;
      uint32_t left = profiles[sel].sec>elapsed ? profiles[sel].sec-elapsed : 0;
      if(left==0){ heaterPWM=0; applyOutputs(); mode=DONE; }
      if(back){ heaterPWM=0; applyOutputs(); mode=IDLE; }
      drawRun(tH,left,RH,tA);
    } break;

    case DONE:
      heaterPWM=0; applyOutputs();
      if(ok)   mode=MENU;
      if(back) mode=IDLE;
      drawDone();
      break;

    case FAULT:
      heaterPWM=0; applyOutputs();
      if(back) mode=IDLE;
      // Simple fault screen:
      u8g2.clearBuffer(); u8g2.setFont(u8g2_font_6x10_tf);
      u8g2.setCursor(0,22); u8g2.print("FAULT");
      u8g2.setCursor(0,38); u8g2.print(thermFault? "Thermistor error" : "Overtemp");
      u8g2.sendBuffer();
      break;
  }

  delay(40); // UI refresh cadence
}

