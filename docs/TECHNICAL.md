# Drybox Controller - Arduino Nano Project

## Project Overview
This is a filament drybox controller based on Arduino Nano with PID temperature control, humidity monitoring, and user-friendly OLED interface. The system maintains precise temperature control for drying 3D printer filaments using multiple preset profiles.

## Hardware Configuration

### Microcontroller
- **Board**: Arduino Nano (ATmega328P)
- **Platform**: PlatformIO with Arduino framework

### Sensors
- **DHT11**: Temperature and humidity sensor
  - Connected to: D8
  - Type: 4-pin digital sensor
  - Purpose: Ambient temperature and humidity monitoring

- **NTC Thermistor**: 100k Thermistor for heater temperature
  - Connected to: A2
  - Configuration: Voltage divider (5V --- R --- A2 --- NTC 100k --- GND), R calibrated in firmware as 12k (original notes: 6.8k)
  - Beta coefficient: 3950
  - Purpose: Primary temperature sensing for PID control

### Display
- **OLED**: SSD1306 128x64 I2C Display
  - SDA: A4
  - SCL: A5
  - Address: 0x3C
  - Power: 3.3V (with onboard pull-ups)

### Control Inputs
- **UP Button**: A1 (digital-capable, INPUT_PULLUP)
- **DOWN Button**: A3 (bottom-left, digital-capable, INPUT_PULLUP)
- **OK Button**: A6 (bottom-right, analog-only, requires external 10k pull-up to 5V)
- **BACK Button**: A7 (analog-only, requires external 10k pull-up to 5V)

### Outputs
- **Heater Control**: D3 (PWM via Timer2, 0-255)
  - Drives MOSFET gate (~100Ω series + 100k pulldown to GND)
  - Controls 12V heating element

- **Fan Control**: D5 (Digital ON/OFF)
  - Drives MOSFET gate
  - ON for the whole cycle, 3 min run-on afterwards, and while heater > 40°C
  - Requires flyback diode across fan (cathode to +12V)

- **Buzzer**: A0/D14 (Digital)
  - Active buzzer recommended
  - Provides audio feedback for user interactions

### Power
- **12V External Supply**: Powers heater and fan via MOSFETs
- **Arduino 5V**: From Nano's onboard regulator
- **Common Ground**: Shared between 12V supply and Arduino

## Software Architecture

### State Machine
The firmware implements a finite state machine with the following states:

1. **IDLE**: Monitoring mode
   - Displays current temperatures and humidity
   - Heater OFF
   - Transition: OK button → MENU

2. **MENU**: Profile selection
   - Shows available drying profiles
   - Navigate with UP/DOWN buttons
   - Transition: OK → RUN, BACK → IDLE

3. **RUN**: Active drying cycle
   - PID controller active
   - Heater and fan controlled automatically
   - Real-time status display
   - Transition: Timer expires → DONE, BACK → IDLE (abort)

4. **DONE**: Cycle complete
   - Heater OFF
   - Celebratory beeps
   - Transition: OK → MENU, BACK → IDLE

5. **FAULT**: Error state
   - Triggered by thermistor failure or overtemperature
   - Heater immediately OFF
   - Warning beep
   - Transition: BACK → IDLE (after clearing issue)

### PID Controller
Custom PID implementation without external libraries:
- **Kp**: 8.0 (Proportional gain)
- **Ki**: 0.04 (Integral gain)
- **Kd**: 30.0 (Derivative gain)
- **Sample Time**: 1000ms (1 second)
- **Output Range**: 0-255 (PWM duty cycle)
- **Features**:
  - Integral anti-windup
  - Derivative on measurement (reduces setpoint kick)
  - Bumpless initialization

### Drying Profiles
Pre-configured profiles for common filaments:

| Filament | Temperature | Duration |
|----------|-------------|----------|
| PLA      | 45°C        | 4 hours  |
| PETG     | 65°C        | 6 hours  |
| ABS      | 65°C        | 6 hours  |
| TPU      | 50°C        | 6 hours  |
| Nylon    | 75°C        | 12 hours |

### Safety Features
1. **Overheat Protection**: Hard cutoff at 90°C
2. **Thermistor Fault Detection**: Monitors for sensor disconnection
3. **Fault Streak Counter**: Prevents false triggers (4 samples)
4. **Emergency Shutoff**: Immediate heater disable on fault
5. **Display Init Check**: OLED is probed on I2C at power-on; without it the controller only beeps
6. **Heater Response Check**: full power for 10 min with < 2°C rise (and > 5°C below setpoint) → FAULT "Heater no response"
7. **Watchdog**: interrupt-only mode, 4 s. On a stalled loop: heater OFF, fan ON, beep every second until power cycle. (No reset: the old Nano bootloader would reset-loop.)
8. **I2C Timeout**: 25 ms with bus reset, so a stuck OLED bus cannot freeze the loop
9. **Fan Run-on**: fan is ON for the whole cycle, 3 min after the heater was last active, and while the heater reads > 40°C (off again below 35°C)

### Audio Feedback
- **Navigation**: 40ms beep on menu navigation
- **Start Cycle**: Double beep (60ms + 60ms gap)
- **Cycle Complete**: Triple beep (80ms + 80ms gap)
- **Abort**: Single 120ms beep
- **Fault**: Long 300ms warning beep

## Pin Summary

| Pin  | Function       | Type   | Notes                              |
|------|----------------|--------|------------------------------------|
| D3   | Heater PWM     | Output | Timer2 PWM, MOSFET gate            |
| D5   | Fan Control    | Output | Digital, MOSFET gate               |
| D8   | DHT11 Data     | I/O    | Temperature/Humidity sensor        |
| A0   | Buzzer         | Output | Active buzzer                      |
| A1   | Button UP      | Input  | INPUT_PULLUP                       |
| A2   | Thermistor     | Analog | Series resistor to 5V, NTC to GND  |
| A3   | Button DOWN    | Input  | INPUT_PULLUP                       |
| A4   | OLED SDA       | I2C    | Display data                       |
| A5   | OLED SCL       | I2C    | Display clock                      |
| A6   | Button OK      | Analog | External 10k pull-up required      |
| A7   | Button BACK    | Analog | External 10k pull-up required      |

## PlatformIO Configuration

### Dependencies
- `adafruit/Adafruit GFX Library@^1.11.9`
- `adafruit/Adafruit SSD1306@^2.5.9`
- `adafruit/DHT sensor library@^1.4.6`
- `adafruit/Adafruit Unified Sensor@^1.1.14`

### Build Commands
```bash
# Build firmware
pio run

# Upload to Arduino Nano
pio run --target upload

# Open serial monitor
pio device monitor

# Clean build
pio run --target clean
```

## Thermistor Divider Math

The thermistor circuit uses a voltage divider with a fixed series resistor. `R_FIXED_RESISTOR` is 12kΩ, calibrated on the built board against the DHT11 at room temperature (the original 6.8kΩ read ~11°C too high):

```
5V --- R_fixed --- A2 --- NTC 100kΩ --- GND
```

The ADC reads voltage at A2. To calculate thermistor resistance:
```
R_therm = R_fixed × ADC_reading / (ADC_MAX - ADC_reading)
```

Temperature calculation uses the Beta equation (simplified Steinhart-Hart):
```
1/T = 1/T0 + (1/B) × ln(R/R0)
```

Where:
- T0 = 298.15K (25°C)
- B = 3950 (Beta coefficient)
- R0 = 100kΩ (resistance at T0)

## Important Notes

### Button Wiring
- **A1 and A3**: Can use internal pull-ups (INPUT_PULLUP mode)
- **A6 and A7**: Analog-only pins, MUST have external 10kΩ pull-ups to 5V
- Button press connects pin to GND
- Reading threshold for A6/A7: < 200 (out of 1023) = pressed

### MOSFET Safety
- Series gate resistor: ~100Ω (limits current, prevents oscillation)
- Pulldown resistor: 100kΩ to GND (ensures OFF state at startup)
- Flyback diode on fan: Protects MOSFET from inductive kickback

### Power Considerations
- Nano's regulator should not overheat (check with 3.3V OLED load)
- OLED typically draws < 50mA at 3.3V
- Keep common ground between 12V supply and Arduino
- Never use `tone()` with active buzzer - conflicts with Timer2 (PWM on D3)

### Timer Conflicts
- **Timer2** is used for PWM on D3 (heater control)
- Do NOT use `tone()` function (uses Timer2) - will break heater PWM
- If passive buzzer needed, move heater to D9/D10 (Timer1)

## Automated Testing

The project includes comprehensive automated tests for critical hardware control functions. Tests run on the host computer (no Arduino required) using PlatformIO's native testing framework.

### Test Coverage

**1. Thermistor Tests** (`test/test_thermistor/`)
- ADC to resistance conversion
- Resistance to temperature calculation (Beta equation)
- Invalid ADC value handling
- Temperature range validation
- Monotonic behavior verification

**2. PID Controller Tests** (`test/test_pid/`)
- Proportional, Integral, and Derivative responses
- Output clamping and limits
- Integral anti-windup
- Derivative on measurement (no setpoint kick)
- Sample time enforcement
- Reset functionality
- Combined PID response validation

**3. Safety Monitor Tests** (`test/test_safety/`)
- Overtemperature detection (90°C limit)
- Thermistor fault detection (4 sample threshold)
- Fault recovery after valid readings
- Fault streak counting
- Boundary condition testing
- Alternating valid/invalid reading handling

### Running Tests

```bash
# Run all tests
pio test -e native

# Run tests with verbose output
pio test -e native -v

# Run specific test
pio test -e native -f test_pid
pio test -e native -f test_safety
pio test -e native -f test_thermistor
```

### Test Results

All 37 tests pass successfully:
- **13 PID tests**: PASSED
- **17 Safety tests**: PASSED
- **7 Thermistor tests**: PASSED

### Code Architecture for Testing

The codebase has been refactored for testability:

```
include/
├── thermistor.h       # Thermistor ADC/temperature calculations
├── pid_controller.h   # PID control algorithm
└── safety.h          # Safety monitoring and fault detection
```

Key functions are extracted into header-only libraries that can be:
- Tested independently without hardware
- Verified for correctness before deployment
- Reused in other projects

### Continuous Integration

Before uploading firmware to Arduino:
1. Run tests: `pio test -e native`
2. Build firmware: `pio run -e nanoatmega328`
3. Verify memory usage (RAM: 27.8%, Flash: 71.2%). The OLED buffer takes another 1024 B of heap and the DHT library needs ~390 B of stack, so static RAM must stay below ~600 B
4. Upload only after all tests pass

### Test-Driven Safety

Critical safety features are validated by tests:
- **Temperature validation**: Ensures 90°C limit is enforced
- **Fault detection**: Verifies thermistor errors trigger shutdown
- **PID stability**: Confirms output stays within bounds (0-255)
- **Sensor error handling**: Tests NAN and invalid readings

## Development History
- Hardware sketch: see `hardware-sketch.png`
- Previous iterations: see `legacy/`
- PlatformIO port: October 31, 2025
- Repository: https://github.com/Granulka/drybox.git

## Future Improvements
1. Add data logging to SD card
2. Implement custom profile editor via button interface
3. Add WiFi module for remote monitoring (ESP32 port?)
4. Graph temperature history on OLED
5. Add RH% target with dehumidifier control
6. Implement auto-tuning PID parameters
7. Add low filament humidity detection (cycle complete early)

## Troubleshooting

### Display not working
- Check I2C address (use I2C scanner sketch)
- Verify 3.3V supply
- Check SDA/SCL connections

### Temperature reads as NAN
- Check thermistor connections
- Verify the series resistor value against `R_FIXED_RESISTOR`
- Test ADC reading directly (about 913 at 25°C, about 448 at 90°C with R_fixed = 12k)

### Heater not responding
- Check MOSFET gate connections
- Verify 12V supply
- Test PWM signal with multimeter
- Check for Timer2 conflicts

### Buttons not responding
- A6/A7 need external 10kΩ pull-ups
- Check for proper debounce (30ms)
- Verify INPUT_PULLUP mode for A1/A3

### PID oscillating
- Reduce Kp (try 4.0)
- Increase Kd (try 50.0)
- Check for thermal mass issues
- Verify heater thermal contact

## Code Structure

```
Drybox/
├── platformio.ini          # PlatformIO configuration (includes native test env)
├── src/
│   └── main.cpp           # Main firmware file
├── include/               # Testable header-only libraries
│   ├── thermistor.h       # Thermistor calculations
│   ├── pid_controller.h   # PID control logic
│   └── safety.h          # Safety monitoring
├── test/                  # Automated tests (Unity framework)
│   ├── test_thermistor/   # Thermistor calculation tests
│   ├── test_pid/          # PID controller tests
│   └── test_safety/       # Safety monitor tests
├── tools/
│   └── bench_test.py      # Scripted bench test for the nano_debug build
├── docs/
│   ├── TECHNICAL.md       # This file - technical documentation
│   ├── hardware-sketch.png  # Hand-drawn wiring sketch
│   └── photos/            # Photos of the build
├── legacy/                # Earlier single-file iterations (not built)
└── README.md              # User-facing documentation
```

## License
Open source - feel free to modify and distribute

## Contact
Repository: https://github.com/Granulka/drybox.git
