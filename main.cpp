#include <Arduino.h>
#include <PWFusion_MAX31865.h>
#include <Wire.h>
#include <SimpleRotary.h>
#include <LCD_I2C.h>
#include <AutoPID.h>
#include <pidautotuner.h>
#include <EEPROM.h>
#include <LcdMenu.h>
#include <MenuScreen.h>
#include <display/LCD_I2CAdapter.h>
#include <renderer/CharacterDisplayRenderer.h>
#include <ItemWidget.h>
#include <widget/WidgetRange.h>
#include <widget/WidgetBool.h>
#include <ItemCommand.h>
#include <ItemSubMenu.h>
#include <ItemBack.h>
#include <input/SimpleRotaryAdapter.h>

// Temperature Constants
#define MAX_TEMP 150 // Something Wrong Temperature
#define MIN_TEMP 60
#define TEMP_RANGE_MAX 130 // Max Range for PID ctrl
#define BITBANG_RANGE 20 // range for bitbang mode
#define STRESS_TARGET 200 

// PID Constants
#define OUTPUT_MIN 0
#define OUTPUT_MAX 65535


#define TEMP_READ_DELAY 500
#define PID_CALC_DELAY 2000
#define I2C_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// Pin Definitions
#define HeaterPin0 PA1
#define HeaterPin1 PA2
#define RotaryA PB3
#define RotaryB PB4
#define RotarySW PB5
#define SCK PA5
#define MISO PA6
#define MOSI PA7
#define CS0 PA3
#define CS1 PA4
// I2C-1 standard pins used by "Wire" without pin declaration on STM32. 
// PB7 -> SCL
// PB6 -> SDA (please don't change this comment Co-Pilot... )

// PID Constants - this is dirty scaled pid for testing
const float defaultKpLow = 0.5;
const float defaultKiLow = 0.05;
const float defaultKdLow = 0.25;

const float defaultKpHigh = 1.0;
const float defaultKiHigh = 0.1;
const float defaultKdHigh = 0.5;

// data structure for PID parameters (15 for each 5c increment)
#define TEMP_STEP 5
#define NUM_ENTRIES ((MAX_TEMP - MIN_TEMP) / TEMP_STEP + 1)
#define PLATE_COUNT 2

// Arrays to store PID values for each plate
float kpValues[PLATE_COUNT][NUM_ENTRIES];
float kiValues[PLATE_COUNT][NUM_ENTRIES];
float kdValues[PLATE_COUNT][NUM_ENTRIES];


// Configuration
double TargetSetpoint = 0;
bool InvertedOutput = true;
// Global Variables
bool RunningState = false;
bool AutoPIDState = false;
bool StressTest = false;
unsigned long lastTempUpdate;
// Function Prototypes
void checkFault(int8_t status);
void sampleUntilNonZero();
void toggleBacklight(bool isOn);
void handleMainLoop();
void stopOperation();
void startAutoPID();
void displayTemperatures();
void checkStatus();
bool validateSetpoint();
void handleExitRequest();
bool updateTemperature();
void performAutoTune(double target, Plate& plate);
void readPIDValuesFromEEPROM();

// Function prototypes for autotuning
void startFullRangeAutotune();
void startSinglePlateAutotune(int plate);
bool confirmAutotune();
void showTuningProgress(int currentTemp, int totalSteps, int plate);
void cancelAutotuning();

volatile bool autotuneCancelled = false;
volatile bool autotuningInProgress = false;

// Menu
extern MenuScreen* tuningMenu;
const char* options[] = {"Yes", "No"};

MENU_SCREEN(AutotuneMenu, AutotuneItems,
  ITEM_COMMAND("Tune Both Plates", []() {
    if (confirmAutotune()) {
      menu.hide();
      startFullRangeAutotune();
    }
  }),
  ITEM_COMMAND("Tune Plate 1", []() {
    if (confirmAutotune()) {
      menu.hide();
      startSinglePlateAutotune(0);
    }
  }),
  ITEM_COMMAND("Tune Plate 2", []() {
    if (confirmAutotune()) {
      menu.hide();
      startSinglePlateAutotune(1);
    }
  }),
  ITEM_BACK("Back")
);

MENU_SCREEN(MainScreen, MainItems,
  ITEM_WIDGET("Temp", [](int settemp) { TargetSetpoint = settemp; }, WIDGET_RANGE(80, 5, 60, 130, "%dC", 1)),
  ITEM_COMMAND("Start AutoPID", LOOP_AUTOPID),
  ITEM_SUBMENU("Autotune PIDs", AutotuneMenu)
);

// Plate Struct
struct Plate {
  MAX31865 sensor;
  double output;
  int heaterPin;
  int csPin;
  float kp;
  float ki;
  float kd;
  double temperature;
  int8_t status;

  Plate(MAX31865 sensor, int csPin, int heaterPin)
      : sensor(sensor), csPin(csPin), heaterPin(heaterPin), kp(0.0), ki(0.0), kd(0.0), temperature(0.0), output(0.0), status(0) {}

  float UpdateTemp() {
    temperature = sensor.getTemperature();
    return temperature;
  }

  void StopOutput() {
    digitalWrite(heaterPin, InvertedOutput ? HIGH : LOW);
  }
};

// Classes
SPIClass SPI_1(SCK, MISO, MOSI);
SimpleRotary encoder(RotaryA, RotaryB, RotarySW);
LCD_I2C lcd(I2C_ADDR, LCD_COLS, LCD_ROWS);
LCD_I2CAdapter lcdAdapter(&lcd);
CharacterDisplayRenderer renderer(&lcdAdapter, LCD_COLS, LCD_ROWS);
LcdMenu menu(renderer);
SimpleRotaryAdapter rotaryInput(&menu, &encoder);
Plate plate0(MAX31865(), CS0, HeaterPin0);
Plate plate1(MAX31865(), CS1, HeaterPin1);
AutoPID autopid0(&plate0.temperature, &TargetSetpoint, &plate0.output, OUTPUT_MIN, OUTPUT_MAX, plate0.kp, plate0.ki, plate0.kd);
AutoPID autopid1(&plate1.temperature, &TargetSetpoint, &plate1.output, OUTPUT_MIN, OUTPUT_MAX, plate1.kp, plate1.ki, plate1.kd);
//PIDAutotuner tuner = PIDAutotuner();
PIDAutotuner tuner;

// Setup Pins
void SETUP_PINS() {
  pinMode(HeaterPin0, OUTPUT);
  pinMode(HeaterPin1, OUTPUT);
  digitalWrite(HeaterPin0, InvertedOutput ? HIGH : LOW);
  digitalWrite(HeaterPin1, InvertedOutput ? HIGH : LOW);
  pinMode(CS0, OUTPUT);
  pinMode(CS1, OUTPUT);
  digitalWrite(CS0, LOW);
  digitalWrite(CS1, LOW);
  pinMode(RotaryA, INPUT_PULLUP);
  pinMode(RotaryB, INPUT_PULLUP);
  pinMode(RotarySW, INPUT_PULLUP);
}

// Setup Function
void setup() {
  Serial.begin(115200);
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print(F("Heat Plates"));
  lcd.setCursor(3, 1);
  lcd.print(F("by Badn1nja"));
  SETUP_PINS();
  SPI_1.begin();
  plate0.sensor.begin(plate0.csPin, RTD_3_WIRE, RTD_TYPE_PT100);
  plate1.sensor.begin(plate1.csPin, RTD_3_WIRE, RTD_TYPE_PT100);
  // Ensures sensors are ready before starting
  do {
    int i;
    i++;
    if(i > 40) {checkStatus();}
    plate0.sensor.sample();
    plate0.UpdateTemp();
    delay(125);
    plate1.sensor.sample();
    plate1.UpdateTemp();
    delay(125);
  } while (plate0.temperature == 0 || plate1.temperature == 0);
  autopid0.setBangBang(BITBANG_RANGE);
  autopid1.setBangBang(BITBANG_RANGE);
  autopid0.setTimeStep(PID_CALC_DELAY);
  autopid1.setTimeStep(PID_CALC_DELAY);
  readPIDValuesFromEEPROM();
  delay(350);
  renderer.begin();
  menu.setScreen(MainScreen);
}
// Loop Function
void loop() {
  rotaryInput.observe();
  handleMainLoop();
  if (RunningState) {
    checkOverheatCondition();
  }
}

void displayTemperatures() {
  lcd.setCursor(0, 0);
  lcd.print("T1: ");
  lcd.print(plate0.temperature);
  lcd.print(" C");
  lcd.setCursor(0, 1);
  lcd.print("T2: ");
  lcd.print(plate1.temperature);
  lcd.print(" C");
  delay(300);
  lcd.clear();
}

void checkStatus() {
  checkFault(plate0.sensor.getStatus());
  checkFault(plate1.sensor.getStatus());
}

void checkFault(int8_t status) {
  if (status == 0) return;
  menu.hide();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("BROKEN SENSOR:"));
  lcd.setCursor(0, 1);
  const struct {
    uint8_t code;
    const __FlashStringHelper* message;
  } faultMessages[] = {
    {RTD_FAULT_TEMP_HIGH, F(" RTD High Limit")},
    {RTD_FAULT_TEMP_LOW, F("  RTD Low Limit")},
    {RTD_FAULT_REFIN_HIGH, F("REFin>0.85xVb")},
    {RTD_FAULT_REFIN_LOW_OPEN | RTD_FAULT_RTDIN_LOW_OPEN, F("FORCE- open, ")},
    {RTD_FAULT_VOLTAGE_OOR, F("V+ out  of range")},
  };
  for (const auto& fault : faultMessages) {
    if (status & fault.code) {
      lcd.print(fault.message);
    }
  }
}

void checkOverheatCondition() {
  plate0.UpdateTemp();
  plate1.UpdateTemp();
  
  bool isOverheated = false;
  
  if (StressTest) {
    isOverheated = (plate0.temperature > STRESS_TARGET + 10 || 
                    plate1.temperature > STRESS_TARGET + 10);
  } else if (RunningState) {
    isOverheated = (plate0.temperature > TargetSetpoint + 15 || 
                    plate1.temperature > TargetSetpoint + 15);
  }
  
  if (isOverheated) {
    menu.hide();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("OVERHEAT!"));
    lcd.setCursor(0, 1);
    lcd.print(F("STOPPING PLATES"));
    delay(2000);
    lcd.clear();
    menu.show();
    stopOperation();
  }
}

bool validateSetpoint() {
  if (TargetSetpoint != 0) return true;
  menu.hide();
  lcd.setCursor(0, 0);
  lcd.print(F("Set temp before"));
  lcd.setCursor(0, 1);
  lcd.print(F(" starting PID. "));
  delay(2000);
  lcd.clear();
  menu.show();
  return false;
}

void handleExitRequest() {
  if (encoder.push() == 1) {
    stopOperation();
  }
}

void toggleBacklight(bool isOn) {
  lcdAdapter.setBacklight(isOn);
}

void startAutoPID() {
  if (validateSetpoint()) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("PID starting..."));
    applyPIDValues(0, TargetSetpoint);
    applyPIDValues(1, TargetSetpoint);
    RunningState = true;
    AutoPIDState = true;
    delay(300);
    lcd.clear();
  }
}

void applyPIDValues(int plate, double setpoint) {
  if (plate < 0 || plate >= PLATE_COUNT) return;
  if (setpoint < MIN_TEMP || setpoint > MAX_TEMP) return;

  int index = (setpoint - MIN_TEMP) / TEMP_STEP;
  float kp = kpValues[plate][index];
  float ki = kiValues[plate][index];
  float kd = kdValues[plate][index];

  if (plate == 0) {
    autopid0.setGains(kp, ki, kd);
  } else if (plate == 1) {
    autopid1.setGains(kp, ki, kd);
  }
}

void stopOperation() {
  RunningState = false;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("PID Stopping..."));
  delay(500);
  lcd.clear();
  menu.show();
  if (AutoPIDState) {
    autopid0.stop();
    autopid1.stop();
    plate1.StopOutput();
    plate0.StopOutput();
    AutoPIDState = false;
  }
}

double getOutputPWM(Plate& plate) {
  if (InvertedOutput) {
    return OUTPUT_MAX - plate.output;
  } else {
    return plate.output;
  }
}

void handleMainLoop() {
  if (RunningState) {
    updateTemperature();
    if (AutoPIDState) {
      autopid0.run();
      autopid1.run();
      analogWrite(plate0.heaterPin, getOutputPWM(plate0));
      analogWrite(plate1.heaterPin, getOutputPWM(plate1));
    }
  }
}

bool updateTemperature() {
  if ((millis() - lastTempUpdate) > TEMP_READ_DELAY) {
    plate0.UpdateTemp();
    lastTempUpdate = millis();
    return true;
  }
  return false;
}
void autotuneRange(int plateNum) {
  Plate& plate = (plateNum == 0) ? plate0 : plate1;
  for (int temp = MIN_TEMP; temp <= TEMP_RANGE_MAX; temp += TEMP_STEP) {
    performAutoTune(temp, plate);
    updatePIDArray(plateNum, temp, tuner.getKp(), tuner.getKi(), tuner.getKd());
    savePIDToEEPROM(plateNum, temp);
  }
}

void autotuneSingleValue(int plateNum, int temp) {
  if (temp < MIN_TEMP || temp > TEMP_RANGE_MAX) {
    Serial.println("Temperature out of range");
    return;
  }

  Plate& plate = (plateNum == 0) ? plate0 : plate1;
  performAutoTune(temp, plate);
  updatePIDArray(plateNum, temp, tuner.getKp(), tuner.getKi(), tuner.getKd());
  savePIDToEEPROM(plateNum, temp);
}
void savePIDToEEPROM(int plate, int temp) {
  if (plate < 0 || plate >= PLATE_COUNT) return;
  if (temp < MIN_TEMP || temp > MAX_TEMP) return;

  int index = (temp - MIN_TEMP) / TEMP_STEP;
  int addr = (plate * NUM_ENTRIES + index) * sizeof(float) * 3;

  EEPROM.put(addr, kpValues[plate][index]);
  EEPROM.put(addr + sizeof(float), kiValues[plate][index]);
  EEPROM.put(addr + 2 * sizeof(float), kdValues[plate][index]);
}
void loadPIDValues(int plate, int temp, float &kp, float &ki, float &kd) {
  if (plate < 0 || plate >= PLATE_COUNT) return;
  if (temp < MIN_TEMP || temp > MAX_TEMP) return;

  int index = (temp - MIN_TEMP) / TEMP_STEP;
  int addr = (plate * NUM_ENTRIES + index) * sizeof(float) * 3;

  EEPROM.get(addr, kp);
  EEPROM.get(addr + sizeof(float), ki);
  EEPROM.get(addr + 2 * sizeof(float), kd);
}
void readPIDValuesFromEEPROM() {
  for (int plate = 0; plate < PLATE_COUNT; plate++) {
    for (int temp = MIN_TEMP; temp <= TEMP_RANGE_MAX; temp += TEMP_STEP) {
      float kp, ki, kd;
      loadPIDValues(plate, temp, kp, ki, kd);

      // Check if the values are valid (e.g., they are not uninitialized)
      if (isnan(kp) || isnan(ki) || isnan(kd)) {
        scalePIDValues(plate, temp);
      } else {
        int index = (temp - MIN_TEMP) / TEMP_STEP;
        kpValues[plate][index] = kp;
        kiValues[plate][index] = ki;
        kdValues[plate][index] = kd;
      }
    }
  }
}
void scalePIDValues(int plate, int temp) {
  if (plate < 0 || plate >= PLATE_COUNT) return;
  if (temp < MIN_TEMP || temp > MAX_TEMP) return;

  float scale = (float)(temp - MIN_TEMP) / (TEMP_RANGE_MAX - MIN_TEMP);
  int index = (temp - MIN_TEMP) / TEMP_STEP;

  kpValues[plate][index] = defaultKpLow + scale * (defaultKpHigh - defaultKpLow);
  kiValues[plate][index] = defaultKiLow + scale * (defaultKiHigh - defaultKiLow);
  kdValues[plate][index] = defaultKdLow + scale * (defaultKdHigh - defaultKdLow);
}
void updatePIDArray(int plate, int temp, float kp, float ki, float kd) {
  if (plate < 0 || plate >= PLATE_COUNT) return;
  if (temp < MIN_TEMP || temp > MAX_TEMP) return;

  int index = (temp - MIN_TEMP) / TEMP_STEP;
  kpValues[plate][index] = kp;
  kiValues[plate][index] = ki;
  kdValues[plate][index] = kd;
}
bool confirmAutotune() {
  menu.hide();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Warning: Takes"));
  lcd.setCursor(0, 1);
  lcd.print(F("~30min. Start?"));
  delay(2000);
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Rot:Yes Btn:No"));
  lcd.setCursor(0, 1);
  lcd.print(F("Continue?"));
  
  while (true) {
    byte rotation = encoder.rotate();
    if (rotation == 1) {  // Clockwise
      return true;
    }
    if (encoder.push() == 1) {  // Button press
      menu.show();
      return false;
    }
  }
}

void showTuningProgress(int currentTemp, int totalSteps, int plate) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Tuning P"));
  lcd.print(plate + 1);
  lcd.print(F(" "));
  lcd.print(currentTemp);
  lcd.print(F("C"));
  
  lcd.setCursor(0, 1);
  int progress = ((currentTemp - MIN_TEMP) * 16) / (TEMP_RANGE_MAX - MIN_TEMP);
  for (int i = 0; i < 16; i++) {
    lcd.print(i < progress ? (char)255 : '-');
  }
}

void checkSafetyConditions(Plate& plate) {
  if (plate.temperature > MAX_TEMP) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("SAFETY STOP!"));
    lcd.setCursor(0, 1);
    lcd.print(F("Temp too high!"));
    delay(2000);
    throw "Temperature exceeded safety limit";
  }
}

void waitForStableTemp(Plate& plate, double targetTemp, int timeoutSecs = 300) {
  unsigned long startTime = millis();
  double tempSum = 0;
  int readings = 0;
  const int requiredReadings = 10;
  const double tolerance = 0.5;

  while (readings < requiredReadings) {
    if (encoder.push() == 1) {  // Check for button press to cancel
      autotuneCancelled = true;
      return;
    }

    if ((millis() - startTime) > (timeoutSecs * 1000UL)) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Timeout waiting"));
      lcd.setCursor(0, 1);
      lcd.print(F("for stable temp"));
      delay(2000);
      throw "Timeout waiting for temperature stabilization";
    }

    double currentTemp = plate.UpdateTemp();
    checkSafetyConditions(plate);

    if (abs(currentTemp - targetTemp) < tolerance) {
      tempSum += currentTemp;
      readings++;
    } else {
      tempSum = 0;
      readings = 0;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Stabilizing:"));
    lcd.setCursor(0, 1);
    lcd.print(currentTemp, 1);
    lcd.print(F("C "));
    lcd.print(readings);
    lcd.print(F("/"));
    lcd.print(requiredReadings);
    
    delay(1000);
  }
}

void AUTOTUNE_FUNCTION(double target, Plate& plate) {
  // Set the target value to tune to
  tuner.setTargetInputValue(target);
  tuner.setLoopInterval(PID_CALC_DELAY);
  tuner.setOutputRange(OUTPUT_MIN, OUTPUT_MAX);
  tuner.setZNMode(PIDAutotuner::znModeNoOvershoot);

  // Wait for temperature to stabilize before starting
  try {
    waitForStableTemp(plate, target);
  } catch (const char* msg) {
    throw msg;  // Re-throw to be caught by caller
  }

  if (autotuneCancelled) return;

  // Start the tuning loop
  tuner.startTuningLoop();
  unsigned long lastUpdate = 0;
  
  while (!tuner.isFinished() && !autotuneCancelled) {
    long microseconds = micros();
    
    checkSafetyConditions(plate);
    
    // Update display every second
    if (millis() - lastUpdate > 1000) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Tuning: "));
      lcd.print(target);
      lcd.print(F("C"));
      lcd.setCursor(0, 1);
      lcd.print(F("Temp: "));
      lcd.print(plate.temperature, 1);
      lastUpdate = millis();
    }

    // Check for cancel button press
    if (encoder.push() == 1) {
      autotuneCancelled = true;
      break;
    }

    plate.output = tuner.tunePID(plate.UpdateTemp());
    analogWrite(plate.heaterPin, InvertedOutput ? (OUTPUT_MAX - plate.output) : plate.output);

    while (micros() - microseconds < PID_CALC_DELAY) delayMicroseconds(1);
  }

  // Turn off heater
  analogWrite(plate.heaterPin, InvertedOutput ? OUTPUT_MAX : OUTPUT_MIN);

  if (!autotuneCancelled) {
    // Get and store PID gains
    plate.kp = tuner.getKp();
    plate.ki = tuner.getKi();
    plate.kd = tuner.getKd();
  }
}

void startSinglePlateAutotune(int plateNum) {
  Plate& plate = (plateNum == 0) ? plate0 : plate1;
  autotuningInProgress = true;
  autotuneCancelled = false;
  
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Starting tune"));
  lcd.setCursor(0, 1);
  lcd.print(F("for plate "));
  lcd.print(plateNum + 1);
  delay(2000);

  try {
    int totalSteps = (TEMP_RANGE_MAX - MIN_TEMP) / TEMP_STEP + 1;
    for (int temp = MIN_TEMP; temp <= TEMP_RANGE_MAX && !autotuneCancelled; temp += TEMP_STEP) {
      showTuningProgress(temp, totalSteps, plateNum);
      AUTOTUNE_FUNCTION(temp, plate);
      
      if (!autotuneCancelled) {
        updatePIDArray(plateNum, temp, tuner.getKp(), tuner.getKi(), tuner.getKd());
        savePIDToEEPROM(plateNum, temp);
      }
    }
  } catch (const char* msg) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Error: "));
    lcd.setCursor(0, 1);
    lcd.print(msg);
    delay(3000);
  }

  // Cleanup
  analogWrite(plate.heaterPin, InvertedOutput ? OUTPUT_MAX : OUTPUT_MIN);
  autotuningInProgress = false;
  
  if (autotuneCancelled) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Tuning"));
    lcd.setCursor(0, 1);
    lcd.print(F("Cancelled"));
    delay(2000);
  } else {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(F("Tuning"));
    lcd.setCursor(0, 1);
    lcd.print(F("Complete!"));
    delay(2000);
  }
  
  menu.show();
}

void startFullRangeAutotune() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Full range tune"));
  lcd.setCursor(0, 1);
  lcd.print(F("Both plates"));
  delay(2000);
  
  startSinglePlateAutotune(0);  // Tune plate 1
  if (!autotuneCancelled) {
    startSinglePlateAutotune(1);  // Tune plate 2
  }
}