/*
  Smart Solar Tracking and Self-Cleaning Solar Panel System
  Board: Arduino Uno

  FINAL PIN MAP
  -------------
  A0  Bottom-left LDR
  A1  Top-left LDR
  A2  Top-right LDR
  A3  Bottom-right LDR
  A4  LCD SDA (I2C)
  A5  LCD SCL (I2C)

  D2  Horizontal / azimuth servo
  D3  YF-S201 flow sensor signal (interrupt pin)
  D4  HX711 SCK
  D5  HX711 DT / DOUT
  D6  Vertical / tilt servo
  D7  DHT22 data
  D8  Pump relay IN1
  D9  TCRT5000 digital output DO
  D10-D13 Unused (LEDs and buzzer were removed from the final prototype)

  IMPORTANT HARDWARE NOTES
  ------------------------
  1. The pump and both servos must use suitable external power supplies.
  2. Arduino GND, servo-supply GND, relay GND, and sensor GND must be common.
  3. Never power the pump directly from an Arduino pin.
  4. This sketch assumes an ACTIVE-LOW relay. Change RELAY_ACTIVE_LOW if needed.
  5. The TCRT5000 is connected through DO, not AO. Adjust its potentiometer so
     the digital output changes at the desired dust threshold.
  6. The load cell uses the multi-mass calibration reported in Section 11,
     Table 4. The total measured mass is converted from the raw HX711 reading,
     then the 354 g empty-tank mass is subtracted to obtain the water mass.
*/

#include <Wire.h>
#include <Servo.h>
#include <DHT.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

// ============================= PINS =============================
const uint8_t LDR_BL_PIN = A0;
const uint8_t LDR_TL_PIN = A1;
const uint8_t LDR_TR_PIN = A2;
const uint8_t LDR_BR_PIN = A3;

const uint8_t HORIZONTAL_SERVO_PIN = 2;
const uint8_t FLOW_SENSOR_PIN      = 3;
const uint8_t HX711_SCK_PIN        = 4;
const uint8_t HX711_DOUT_PIN       = 5;
const uint8_t VERTICAL_SERVO_PIN   = 6;
const uint8_t DHT_PIN              = 7;
const uint8_t RELAY_PIN            = 8;
const uint8_t DUST_SENSOR_PIN      = 9;
// D10-D13 are intentionally unused in the final prototype.

// ======================== DEVICE SETTINGS =======================
#define DHTTYPE DHT22

const uint8_t LCD_ADDRESS = 0x27;  // Change to 0x3F only if the LCD remains blank.
const uint8_t LCD_COLUMNS = 16;
const uint8_t LCD_ROWS    = 2;

const bool RELAY_ACTIVE_LOW = true;
const bool DUST_ACTIVE_LOW  = true;

// Change either value if that servo moves away from the stronger light.
const bool HORIZONTAL_REVERSED = false;
const bool VERTICAL_REVERSED   = false;

// ======================== TRACKING SETTINGS =====================
const int LDR_TOLERANCE = 50;
const int SERVO_STEP_DEG = 1;

const int HORIZONTAL_MIN_ANGLE = 10;
const int HORIZONTAL_MAX_ANGLE = 170;
const int VERTICAL_MIN_ANGLE   = 15;
const int VERTICAL_MAX_ANGLE   = 165;

const int HORIZONTAL_START_ANGLE = 90;
const int VERTICAL_START_ANGLE   = 90;

const unsigned long LDR_SAMPLE_INTERVAL_MS = 100UL;
const unsigned long TRACKING_INTERVAL_MS   = 150UL;
const unsigned long POST_CLEANING_SETTLE_MS = 10000UL;

// ======================== CLEANING SETTINGS =====================
const float MIN_WATER_TO_CLEAN_L = 0.50f;
const float CRITICAL_WATER_L     = 0.30f;

const float HIGH_TEMPERATURE_C = 35.0f;
const float HIGH_HUMIDITY_PCT  = 70.0f;

const unsigned long CLEANING_DURATION_MS  = 5000UL;
const unsigned long FLOW_CHECK_TIME_MS    = 3000UL;
const float MIN_ACCEPTABLE_FLOW_L_MIN     = 1.00f;

// YF-S201 nominal relation: frequency (Hz) = 7.5 x flow rate (L/min)
const float FLOW_HZ_PER_L_MIN = 7.5f;
const float FLOW_PULSES_PER_LITER = 450.0f;

const uint8_t DUST_CONFIRMATION_COUNT = 3;
const uint8_t DUST_CLEAR_COUNT        = 5;
const unsigned long DUST_SAMPLE_INTERVAL_MS = 500UL;

// ====================== LOAD-CELL CALIBRATION ===================
// Multi-mass calibration from Section 11, Table 4:
//   76 g  ->  65,480 counts
//   209 g -> 179,540 counts
//   354 g -> 304,760 counts
//   62 g  ->  53,220 counts
//
// These points give approximately 860 HX711 counts per gram, which also
// reproduces the calculated masses shown in the report:
//   measured total mass (g) = raw HX711 reading / 860
//
// The tank itself has a measured mass of 354 g, so:
//   water mass (g) = measured total mass (g) - 354 g
const float LOAD_CELL_COUNTS_PER_GRAM = 860.0f;
const float LOAD_CELL_INTERCEPT_G     = 0.0f;
const float EMPTY_TANK_MASS_G         = 354.0f;

const uint32_t LDR_CAL_MAGIC  = 0x4C445243UL;  // "LDRC"
const int EEPROM_LDR_CAL_ADDRESS  = 32;

struct LdrCalibrationData {
  uint32_t magic;
  float kTL;
  float kTR;
  float kBL;
  float kBR;
};

// =========================== TIMING =============================
const unsigned long DHT_INTERVAL_MS       = 2500UL;
const unsigned long HX711_INTERVAL_MS     = 500UL;
const unsigned long FLOW_INTERVAL_MS      = 1000UL;
const unsigned long LCD_PAGE_INTERVAL_MS  = 2500UL;
const unsigned long LCD_ALARM_INTERVAL_MS = 500UL;
const unsigned long SERIAL_INTERVAL_MS    = 1000UL;
const unsigned long SENSOR_STALE_LIMIT_MS = 10000UL;

// =========================== OBJECTS =============================
Servo horizontalServo;
Servo verticalServo;
DHT dht(DHT_PIN, DHTTYPE);
HX711 scale;
LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);

// ======================= GLOBAL VARIABLES =======================
int rawTL = 0;
int rawTR = 0;
int rawBL = 0;
int rawBR = 0;

int ldrTL = 0;
int ldrTR = 0;
int ldrBL = 0;
int ldrBR = 0;

float ldrKTL = 1.0f;
float ldrKTR = 1.0f;
float ldrKBL = 1.0f;
float ldrKBR = 1.0f;
bool ldrCalibrationValid = false;
bool ldrReadingsValid = false;

int horizontalError = 0;
int verticalError   = 0;
int horizontalAngle = HORIZONTAL_START_ANGLE;
int verticalAngle   = VERTICAL_START_ANGLE;

float temperatureC = NAN;
float humidityPct  = NAN;
bool dhtValid = false;
unsigned long lastValidDhtMs = 0;

float waterMassG  = NAN;
float waterLiters = NAN;
bool loadCellValid = false;
bool loadCalibrationValid = true;  // Fixed multi-mass calibration from the report.
float loadCalibrationFactor = LOAD_CELL_COUNTS_PER_GRAM;
long loadCalibrationOffset = 0;
long loadCellRaw = 0;
float measuredTotalMassG = NAN;
unsigned long lastValidLoadCellMs = 0;

volatile unsigned long totalFlowPulses = 0;
unsigned long previousFlowPulseSnapshot = 0;
unsigned long pumpStartPulseSnapshot = 0;
float flowRateLMin = 0.0f;
float lastCleaningAverageFlowLMin = 0.0f;
float lastCleaningVolumeL = 0.0f;

bool dustConfirmed = false;
uint8_t consecutiveDustReadings = 0;
uint8_t consecutiveClearReadings = 0;

bool cleaningActive = false;
bool cleaningLatched = false;
bool flowCheckCompleted = false;
bool flowFaultLatched = false;

unsigned long cleaningStartMs = 0;
unsigned long cleaningFinishedMs = 0;
unsigned long lastLdrMs = 0;
unsigned long lastTrackingMs = 0;
unsigned long lastDustSampleMs = 0;
unsigned long lastDhtMs = 0;
unsigned long lastHx711Ms = 0;
unsigned long lastFlowMs = 0;
unsigned long lastLcdPageMs = 0;
unsigned long lastLcdAlarmMs = 0;
unsigned long lastSerialMs = 0;
uint8_t lcdPage = 0;

// ======================= FUNCTION PROTOTYPES =====================
void flowPulseISR();
void pumpOn();
void pumpOff();
void updateLdrSensors();
int readAveragedAnalog(uint8_t pin, uint8_t samples);
int applyLdrCorrection(int rawValue, float correctionFactor);
void calculateTrackingErrors();
void updateTracking();
void updateDustDetection();
void updateDhtSensor();
void updateLoadCell();
void updateFlowMeasurement();
void evaluateCleaningRequest();
void startCleaning();
void updateCleaningCycle();
void stopCleaningSuccessfully();
void stopCleaningLowWater();
void triggerFlowFault(float measuredFlowLMin);
bool environmentalCleaningBlocked();
bool waterLevelSufficient();
bool waterLevelWarning();
bool waterLevelCritical();
void updateLcd();
void displayStatusPage();
void displayEnvironmentPage();
void displayWaterFlowPage();
void displayTrackingPage();
void displayCleaningPage();
void clearLcdLine(uint8_t row);
void printSerialStatus();
void printSerialHelp();
void handleSerialCommands();
void loadLoadCalibration();
void loadLdrCalibration();
void saveLdrCalibration();
void calibrateLdrsUnderUniformLight();
void eraseCalibrations();
const __FlashStringHelper *currentStatusText();

// ============================== SETUP =============================
void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  // Set the safe relay state before changing the pin to OUTPUT.
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
  pinMode(RELAY_PIN, OUTPUT);
  pumpOff();

  pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);
  pinMode(DUST_SENSOR_PIN, INPUT_PULLUP);

  horizontalServo.attach(HORIZONTAL_SERVO_PIN);
  verticalServo.attach(VERTICAL_SERVO_PIN);
  horizontalServo.write(horizontalAngle);
  verticalServo.write(verticalAngle);

  dht.begin();

  scale.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  loadLoadCalibration();
  loadLdrCalibration();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("SMART SOLAR SYS"));
  lcd.setCursor(0, 1);
  lcd.print(F("Starting..."));

  attachInterrupt(digitalPinToInterrupt(FLOW_SENSOR_PIN), flowPulseISR, RISING);

  delay(1000);

  printSerialHelp();
  Serial.println(F("ms,TL,TR,BL,BR,eH,eV,servoH,servoV,dust,tempC,humidityPct,waterL,flowLmin,totalPulses,status"));

  Serial.println(F("Load-cell calibration loaded from Section 11, Table 4."));
  Serial.println(F("Equation: totalMass_g = raw/860.0; waterMass_g = totalMass_g - 354.0"));

  if (!ldrCalibrationValid) {
    Serial.println(F("NOTE: LDR correction factors are 1.0. Send 'l' under uniform light to calibrate them."));
  }

  const unsigned long now = millis();
  lastFlowMs = now;
  lastLcdPageMs = now;
}

// =============================== LOOP =============================
void loop() {
  handleSerialCommands();

  // Check the pump timer first so the pump cannot remain on because of a
  // slower sensor read elsewhere in the loop.
  if (cleaningActive) {
    updateCleaningCycle();
  }

  updateLdrSensors();
  updateDustDetection();
  updateDhtSensor();
  updateLoadCell();
  updateFlowMeasurement();

  if (cleaningActive) {
    updateCleaningCycle();
  } else {
    updateTracking();
    evaluateCleaningRequest();
  }

  updateLcd();
  printSerialStatus();
}

// ============================ INTERRUPT ============================
void flowPulseISR() {
  totalFlowPulses++;
}

// ========================== PUMP CONTROL ===========================
void pumpOn() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? LOW : HIGH);
}

void pumpOff() {
  digitalWrite(RELAY_PIN, RELAY_ACTIVE_LOW ? HIGH : LOW);
}

// =========================== LDR TRACKING ==========================
int readAveragedAnalog(uint8_t pin, uint8_t samples) {
  unsigned long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
  }
  return (int)(sum / samples);
}

int applyLdrCorrection(int rawValue, float correctionFactor) {
  int corrected = (int)(rawValue * correctionFactor + 0.5f);
  return constrain(corrected, 0, 1023);
}

void updateLdrSensors() {
  const unsigned long now = millis();
  if (now - lastLdrMs < LDR_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastLdrMs = now;

  rawTL = readAveragedAnalog(LDR_TL_PIN, 4);
  rawTR = readAveragedAnalog(LDR_TR_PIN, 4);
  rawBL = readAveragedAnalog(LDR_BL_PIN, 4);
  rawBR = readAveragedAnalog(LDR_BR_PIN, 4);

  ldrTL = applyLdrCorrection(rawTL, ldrKTL);
  ldrTR = applyLdrCorrection(rawTR, ldrKTR);
  ldrBL = applyLdrCorrection(rawBL, ldrKBL);
  ldrBR = applyLdrCorrection(rawBR, ldrKBR);

  ldrReadingsValid = true;
  calculateTrackingErrors();
}

void calculateTrackingErrors() {
  const int topAverage    = (ldrTL + ldrTR) / 2;
  const int bottomAverage = (ldrBL + ldrBR) / 2;
  const int leftAverage   = (ldrTL + ldrBL) / 2;
  const int rightAverage  = (ldrTR + ldrBR) / 2;

  verticalError   = topAverage - bottomAverage;
  horizontalError = leftAverage - rightAverage;
}

void updateTracking() {
  const unsigned long now = millis();

  if (!ldrReadingsValid || now - lastTrackingMs < TRACKING_INTERVAL_MS) {
    return;
  }
  lastTrackingMs = now;

  // Pause movement briefly after spraying so the water can settle.
  if (cleaningFinishedMs != 0 &&
      now - cleaningFinishedMs < POST_CLEANING_SETTLE_MS) {
    return;
  }

  if (abs(horizontalError) > LDR_TOLERANCE) {
    int direction = (horizontalError > 0) ? 1 : -1;
    if (HORIZONTAL_REVERSED) {
      direction = -direction;
    }

    const int newAngle = constrain(
      horizontalAngle + direction * SERVO_STEP_DEG,
      HORIZONTAL_MIN_ANGLE,
      HORIZONTAL_MAX_ANGLE
    );

    if (newAngle != horizontalAngle) {
      horizontalAngle = newAngle;
      horizontalServo.write(horizontalAngle);
    }
  }

  if (abs(verticalError) > LDR_TOLERANCE) {
    int direction = (verticalError > 0) ? 1 : -1;
    if (VERTICAL_REVERSED) {
      direction = -direction;
    }

    const int newAngle = constrain(
      verticalAngle + direction * SERVO_STEP_DEG,
      VERTICAL_MIN_ANGLE,
      VERTICAL_MAX_ANGLE
    );

    if (newAngle != verticalAngle) {
      verticalAngle = newAngle;
      verticalServo.write(verticalAngle);
    }
  }
}

// =========================== DUST SENSOR ===========================
void updateDustDetection() {
  const unsigned long now = millis();
  if (now - lastDustSampleMs < DUST_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastDustSampleMs = now;

  const int rawState = digitalRead(DUST_SENSOR_PIN);
  const bool dustNow = DUST_ACTIVE_LOW ? (rawState == LOW) : (rawState == HIGH);

  if (dustNow) {
    consecutiveClearReadings = 0;
    if (consecutiveDustReadings < DUST_CONFIRMATION_COUNT) {
      consecutiveDustReadings++;
    }

    if (consecutiveDustReadings >= DUST_CONFIRMATION_COUNT) {
      dustConfirmed = true;
    }
  } else {
    consecutiveDustReadings = 0;
    if (consecutiveClearReadings < DUST_CLEAR_COUNT) {
      consecutiveClearReadings++;
    }

    if (consecutiveClearReadings >= DUST_CLEAR_COUNT) {
      dustConfirmed = false;
      // A completed cycle is allowed to re-arm only after the panel reads clean.
      cleaningLatched = false;
    }
  }
}

// ============================== DHT22 ===============================
void updateDhtSensor() {
  const unsigned long now = millis();
  if (now - lastDhtMs < DHT_INTERVAL_MS) {
    return;
  }
  lastDhtMs = now;

  const float newHumidity = dht.readHumidity();
  const float newTemperature = dht.readTemperature();

  if (!isnan(newHumidity) && !isnan(newTemperature)) {
    humidityPct = newHumidity;
    temperatureC = newTemperature;
    dhtValid = true;
    lastValidDhtMs = now;
  } else if (lastValidDhtMs == 0 || now - lastValidDhtMs > SENSOR_STALE_LIMIT_MS) {
    dhtValid = false;
  }
}

// ========================== HX711 / WATER ==========================
void updateLoadCell() {
  const unsigned long now = millis();
  if (now - lastHx711Ms < HX711_INTERVAL_MS) {
    return;
  }
  lastHx711Ms = now;

  if (!loadCalibrationValid || !scale.is_ready()) {
    if (lastValidLoadCellMs == 0 || now - lastValidLoadCellMs > SENSOR_STALE_LIMIT_MS) {
      loadCellValid = false;
    }
    return;
  }

  const uint8_t samples = cleaningActive ? 1 : 3;
  loadCellRaw = scale.read_average(samples);

  measuredTotalMassG =
    (loadCellRaw / LOAD_CELL_COUNTS_PER_GRAM) + LOAD_CELL_INTERCEPT_G;

  float newMassG = measuredTotalMassG - EMPTY_TANK_MASS_G;

  // Small negative values can occur because of vibration and measurement noise.
  if (newMassG < 0.0f && newMassG > -20.0f) {
    newMassG = 0.0f;
  }

  // Valid range for the one-liter prototype plus a reasonable margin.
  if (newMassG >= -20.0f && newMassG <= 2000.0f) {
    if (isnan(waterMassG)) {
      waterMassG = newMassG;
    } else {
      waterMassG = 0.75f * waterMassG + 0.25f * newMassG;
    }

    if (waterMassG < 0.0f) {
      waterMassG = 0.0f;
    }

    waterLiters = waterMassG / 1000.0f;
    loadCellValid = true;
    lastValidLoadCellMs = now;
  } else if (lastValidLoadCellMs == 0 || now - lastValidLoadCellMs > SENSOR_STALE_LIMIT_MS) {
    loadCellValid = false;
  }
}

bool waterLevelSufficient() {
  return loadCalibrationValid && loadCellValid &&
         waterLiters >= MIN_WATER_TO_CLEAN_L;
}

bool waterLevelWarning() {
  return loadCalibrationValid && loadCellValid &&
         waterLiters > CRITICAL_WATER_L &&
         waterLiters < MIN_WATER_TO_CLEAN_L;
}

bool waterLevelCritical() {
  return loadCalibrationValid && loadCellValid &&
         waterLiters <= CRITICAL_WATER_L;
}

// ============================ FLOW SENSOR ===========================
void updateFlowMeasurement() {
  const unsigned long now = millis();
  const unsigned long elapsedMs = now - lastFlowMs;
  if (elapsedMs < FLOW_INTERVAL_MS) {
    return;
  }

  unsigned long pulseSnapshot;
  noInterrupts();
  pulseSnapshot = totalFlowPulses;
  interrupts();

  const unsigned long intervalPulses = pulseSnapshot - previousFlowPulseSnapshot;
  previousFlowPulseSnapshot = pulseSnapshot;

  const float elapsedSeconds = elapsedMs / 1000.0f;
  const float pulseFrequencyHz = intervalPulses / elapsedSeconds;
  flowRateLMin = pulseFrequencyHz / FLOW_HZ_PER_L_MIN;

  if (!cleaningActive && flowRateLMin < 0.01f) {
    flowRateLMin = 0.0f;
  }

  lastFlowMs = now;
}

// =========================== CLEANING LOGIC =========================
bool environmentalCleaningBlocked() {
  if (!dhtValid) {
    return true;  // Fail-safe if temperature or humidity is unavailable.
  }

  // Report rule: cleaning is blocked only when BOTH values are high.
  return temperatureC >= HIGH_TEMPERATURE_C &&
         humidityPct >= HIGH_HUMIDITY_PCT;
}

void evaluateCleaningRequest() {
  if (!dustConfirmed || cleaningLatched || flowFaultLatched) {
    return;
  }

  if (!loadCalibrationValid || !loadCellValid || !dhtValid) {
    return;
  }

  if (!waterLevelSufficient()) {
    return;
  }

  if (environmentalCleaningBlocked()) {
    return;
  }

  startCleaning();
}

void startCleaning() {
  unsigned long pulseSnapshot;
  noInterrupts();
  pulseSnapshot = totalFlowPulses;
  interrupts();

  pumpStartPulseSnapshot = pulseSnapshot;
  previousFlowPulseSnapshot = pulseSnapshot;
  cleaningStartMs = millis();
  lastFlowMs = cleaningStartMs;
  cleaningActive = true;
  cleaningLatched = true;
  flowCheckCompleted = false;
  flowRateLMin = 0.0f;
  lastCleaningAverageFlowLMin = 0.0f;
  lastCleaningVolumeL = 0.0f;

  pumpOn();
  Serial.println(F("EVENT,CLEANING_STARTED"));
}

void updateCleaningCycle() {
  if (!cleaningActive) {
    return;
  }

  const unsigned long now = millis();
  const unsigned long elapsedMs = now - cleaningStartMs;

  // Absolute maximum-time fail-safe.
  if (elapsedMs >= CLEANING_DURATION_MS) {
    stopCleaningSuccessfully();
    return;
  }

  // Extra protection if the tank becomes critically low during spraying.
  if (loadCellValid && waterLevelCritical()) {
    stopCleaningLowWater();
    return;
  }

  if (!flowCheckCompleted && elapsedMs >= FLOW_CHECK_TIME_MS) {
    unsigned long pulseSnapshot;
    noInterrupts();
    pulseSnapshot = totalFlowPulses;
    interrupts();

    const unsigned long cleaningPulses = pulseSnapshot - pumpStartPulseSnapshot;
    const float elapsedSeconds = elapsedMs / 1000.0f;
    const float averageFrequencyHz = cleaningPulses / elapsedSeconds;
    const float averageFlowLMin = averageFrequencyHz / FLOW_HZ_PER_L_MIN;

    flowCheckCompleted = true;
    lastCleaningAverageFlowLMin = averageFlowLMin;
    lastCleaningVolumeL = cleaningPulses / FLOW_PULSES_PER_LITER;

    if (averageFlowLMin < MIN_ACCEPTABLE_FLOW_L_MIN) {
      triggerFlowFault(averageFlowLMin);
      return;
    }
  }
}

void stopCleaningSuccessfully() {
  unsigned long pulseSnapshot;
  noInterrupts();
  pulseSnapshot = totalFlowPulses;
  interrupts();

  const unsigned long cleaningPulses = pulseSnapshot - pumpStartPulseSnapshot;
  const unsigned long elapsedMs = millis() - cleaningStartMs;
  const float elapsedSeconds = elapsedMs / 1000.0f;

  pumpOff();
  cleaningActive = false;
  cleaningFinishedMs = millis();

  lastCleaningVolumeL = cleaningPulses / FLOW_PULSES_PER_LITER;
  if (elapsedSeconds > 0.0f) {
    lastCleaningAverageFlowLMin =
      (cleaningPulses / elapsedSeconds) / FLOW_HZ_PER_L_MIN;
  }

  Serial.print(F("EVENT,CLEANING_COMPLETED,flowLmin="));
  Serial.print(lastCleaningAverageFlowLMin, 3);
  Serial.print(F(",volumeL="));
  Serial.println(lastCleaningVolumeL, 4);
}

void stopCleaningLowWater() {
  pumpOff();
  cleaningActive = false;
  cleaningFinishedMs = millis();
  Serial.println(F("EVENT,CLEANING_STOPPED_LOW_WATER"));
}

void triggerFlowFault(float measuredFlowLMin) {
  pumpOff();
  cleaningActive = false;
  flowFaultLatched = true;
  cleaningFinishedMs = millis();

  Serial.print(F("EVENT,FLOW_ERROR,flowLmin="));
  Serial.println(measuredFlowLMin, 3);
  Serial.println(F("Fix the water path, then send 'f' to clear the fault."));
}

// ================================ LCD ===============================
const __FlashStringHelper *currentStatusText() {
  if (flowFaultLatched) {
    return F("FLOW ERROR");
  }
  if (!loadCellValid) {
    return F("HX711 ERROR");
  }
  if (!dhtValid) {
    return F("DHT22 ERROR");
  }
  if (cleaningActive) {
    return F("CLEANING");
  }
  if (waterLevelCritical()) {
    return F("CRITICAL WATER");
  }
  if (waterLevelWarning()) {
    return F("WATER WARNING");
  }
  if (dustConfirmed && environmentalCleaningBlocked()) {
    return F("ENV BLOCK");
  }
  if (dustConfirmed && cleaningLatched) {
    return F("WAIT DUST CLEAR");
  }
  if (dustConfirmed) {
    return F("DUST DETECTED");
  }
  if (cleaningFinishedMs != 0 &&
      millis() - cleaningFinishedMs < POST_CLEANING_SETTLE_MS) {
    return F("SETTLING");
  }
  return F("TRACKING");
}

void clearLcdLine(uint8_t row) {
  lcd.setCursor(0, row);
  lcd.print(F("                "));
  lcd.setCursor(0, row);
}

void displayStatusPage() {
  clearLcdLine(0);
  lcd.print(F("SMART SOLAR SYS"));
  clearLcdLine(1);
  lcd.print(currentStatusText());
}

void displayEnvironmentPage() {
  clearLcdLine(0);
  lcd.print(F("T:"));
  if (dhtValid) {
    lcd.print(temperatureC, 1);
    lcd.write((uint8_t)223);
    lcd.print(F("C"));
  } else {
    lcd.print(F("ERR"));
  }

  lcd.print(F(" H:"));
  if (dhtValid) {
    lcd.print(humidityPct, 1);
    lcd.print(F("%"));
  } else {
    lcd.print(F("ERR"));
  }

  clearLcdLine(1);
  lcd.print(F("Environment:"));
  lcd.print(environmentalCleaningBlocked() ? F("BLOCK") : F("OK"));
}

void displayWaterFlowPage() {
  clearLcdLine(0);
  lcd.print(F("Water:"));
  if (loadCellValid) {
    lcd.print(waterLiters, 3);
    lcd.print(F("L"));
  } else {
    lcd.print(F("ERR"));
  }

  clearLcdLine(1);
  lcd.print(F("Flow:"));
  lcd.print(flowRateLMin, 2);
  lcd.print(F("L/m"));
}

void displayTrackingPage() {
  clearLcdLine(0);
  lcd.print(F("H:"));
  lcd.print(horizontalAngle);
  lcd.print(F(" V:"));
  lcd.print(verticalAngle);

  clearLcdLine(1);
  lcd.print(F("Dust:"));
  lcd.print(dustConfirmed ? F("YES") : F("NO"));
  lcd.print(F(" e:"));
  lcd.print(horizontalError);
}

void displayCleaningPage() {
  const unsigned long elapsedMs = millis() - cleaningStartMs;
  const float remainingSeconds =
    (elapsedMs < CLEANING_DURATION_MS)
      ? (CLEANING_DURATION_MS - elapsedMs) / 1000.0f
      : 0.0f;

  clearLcdLine(0);
  lcd.print(F("CLEANING "));
  lcd.print(remainingSeconds, 1);
  lcd.print(F("s"));

  clearLcdLine(1);
  lcd.print(F("Flow:"));
  lcd.print(flowRateLMin, 2);
  lcd.print(F("L/m"));
}

void updateLcd() {
  const unsigned long now = millis();

  if (cleaningActive || flowFaultLatched || waterLevelCritical()) {
    if (now - lastLcdAlarmMs < LCD_ALARM_INTERVAL_MS) {
      return;
    }
    lastLcdAlarmMs = now;

    if (cleaningActive) {
      displayCleaningPage();
    } else {
      displayStatusPage();
    }
    return;
  }

  if (now - lastLcdPageMs < LCD_PAGE_INTERVAL_MS) {
    return;
  }
  lastLcdPageMs = now;

  switch (lcdPage) {
    case 0:
      displayStatusPage();
      break;
    case 1:
      displayEnvironmentPage();
      break;
    case 2:
      displayWaterFlowPage();
      break;
    default:
      displayTrackingPage();
      break;
  }

  lcdPage = (lcdPage + 1) % 4;
}

// =========================== SERIAL OUTPUT ==========================
void printSerialStatus() {
  const unsigned long now = millis();
  if (now - lastSerialMs < SERIAL_INTERVAL_MS) {
    return;
  }
  lastSerialMs = now;

  unsigned long pulseSnapshot;
  noInterrupts();
  pulseSnapshot = totalFlowPulses;
  interrupts();

  Serial.print(now);
  Serial.print(',');
  Serial.print(ldrTL);
  Serial.print(',');
  Serial.print(ldrTR);
  Serial.print(',');
  Serial.print(ldrBL);
  Serial.print(',');
  Serial.print(ldrBR);
  Serial.print(',');
  Serial.print(horizontalError);
  Serial.print(',');
  Serial.print(verticalError);
  Serial.print(',');
  Serial.print(horizontalAngle);
  Serial.print(',');
  Serial.print(verticalAngle);
  Serial.print(',');
  Serial.print(dustConfirmed ? 1 : 0);
  Serial.print(',');

  if (dhtValid) {
    Serial.print(temperatureC, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(',');

  if (dhtValid) {
    Serial.print(humidityPct, 1);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(',');

  if (loadCellValid) {
    Serial.print(waterLiters, 3);
  } else {
    Serial.print(F("NA"));
  }
  Serial.print(',');
  Serial.print(flowRateLMin, 3);
  Serial.print(',');
  Serial.print(pulseSnapshot);
  Serial.print(',');
  Serial.println(currentStatusText());
}

void printSerialHelp() {
  Serial.println(F("\nSmart Solar Tracking and Self-Cleaning System"));
  Serial.println(F("Serial Monitor: 115200 baud, Newline ending"));
  Serial.println(F("Commands:"));
  Serial.println(F("  h = show this command list"));
  Serial.println(F("  l = calibrate all four LDRs under uniform centered light"));
  Serial.println(F("  p = print load-cell equation and saved LDR calibration"));
  Serial.println(F("  f = clear a latched flow fault after fixing the problem"));
  Serial.println(F("  x = emergency pump stop"));
  Serial.println(F("  e = erase the saved LDR calibration"));
}

void handleSerialCommands() {
  while (Serial.available() > 0) {
    const char command = Serial.read();

    if (command == '\n' || command == '\r' || command == ' ') {
      continue;
    }

    switch (command) {
      case 'h':
      case 'H':
        printSerialHelp();
        break;


      case 'l':
      case 'L':
        calibrateLdrsUnderUniformLight();
        break;

      case 'p':
      case 'P':
        Serial.println(F("Load calibration: Section 11, Table 4 (multi-mass)"));
        Serial.print(F("Counts per gram: "));
        Serial.println(LOAD_CELL_COUNTS_PER_GRAM, 3);
        Serial.print(F("Mass intercept (g): "));
        Serial.println(LOAD_CELL_INTERCEPT_G, 3);
        Serial.print(F("Empty tank mass (g): "));
        Serial.println(EMPTY_TANK_MASS_G, 1);
        Serial.print(F("Current raw HX711: "));
        Serial.println(loadCellRaw);
        Serial.print(F("Current total measured mass (g): "));
        if (loadCellValid) Serial.println(measuredTotalMassG, 2);
        else Serial.println(F("NA"));
        Serial.print(F("LDR calibration valid: "));
        Serial.println(ldrCalibrationValid ? F("YES") : F("NO"));
        Serial.print(F("LDR factors TL/TR/BL/BR: "));
        Serial.print(ldrKTL, 5);
        Serial.print('/');
        Serial.print(ldrKTR, 5);
        Serial.print('/');
        Serial.print(ldrKBL, 5);
        Serial.print('/');
        Serial.println(ldrKBR, 5);
        break;

      case 'f':
      case 'F':
        flowFaultLatched = false;
        cleaningLatched = false;
        Serial.println(F("Flow fault cleared."));
        break;

      case 'x':
      case 'X':
        pumpOff();
        cleaningActive = false;
        cleaningLatched = true;
        cleaningFinishedMs = millis();
        Serial.println(F("Emergency pump stop executed."));
        break;

      case 'e':
      case 'E':
        eraseCalibrations();
        break;

      default:
        Serial.print(F("Unknown command: "));
        Serial.println(command);
        break;
    }
  }
}

// ============================ CALIBRATION ============================
void loadLoadCalibration() {
  // The final report uses a fixed multi-mass calibration rather than a
  // single-point calibration or runtime tare.
  loadCalibrationFactor = LOAD_CELL_COUNTS_PER_GRAM;
  loadCalibrationOffset = 0;
  loadCalibrationValid = true;

  // Keep the HX711 object configured consistently, although updateLoadCell()
  // uses the raw average directly to apply the report equation.
  scale.set_scale(loadCalibrationFactor);
  scale.set_offset(loadCalibrationOffset);
}

void loadLdrCalibration() {
  LdrCalibrationData data;
  EEPROM.get(EEPROM_LDR_CAL_ADDRESS, data);

  const bool factorsValid =
    isfinite(data.kTL) && isfinite(data.kTR) &&
    isfinite(data.kBL) && isfinite(data.kBR) &&
    data.kTL > 0.2f && data.kTL < 5.0f &&
    data.kTR > 0.2f && data.kTR < 5.0f &&
    data.kBL > 0.2f && data.kBL < 5.0f &&
    data.kBR > 0.2f && data.kBR < 5.0f;

  if (data.magic == LDR_CAL_MAGIC && factorsValid) {
    ldrKTL = data.kTL;
    ldrKTR = data.kTR;
    ldrKBL = data.kBL;
    ldrKBR = data.kBR;
    ldrCalibrationValid = true;
  } else {
    ldrKTL = 1.0f;
    ldrKTR = 1.0f;
    ldrKBL = 1.0f;
    ldrKBR = 1.0f;
    ldrCalibrationValid = false;
  }
}

void saveLdrCalibration() {
  LdrCalibrationData data;
  data.magic = LDR_CAL_MAGIC;
  data.kTL = ldrKTL;
  data.kTR = ldrKTR;
  data.kBL = ldrKBL;
  data.kBR = ldrKBR;
  EEPROM.put(EEPROM_LDR_CAL_ADDRESS, data);
}

void calibrateLdrsUnderUniformLight() {
  pumpOff();
  cleaningActive = false;

  Serial.println(F("LDR calibration: expose all four LDRs to uniform centered light."));
  Serial.println(F("Keep the panel still while 50 samples are recorded."));

  unsigned long sumTL = 0;
  unsigned long sumTR = 0;
  unsigned long sumBL = 0;
  unsigned long sumBR = 0;

  const uint8_t samples = 50;
  for (uint8_t i = 0; i < samples; i++) {
    sumTL += analogRead(LDR_TL_PIN);
    sumTR += analogRead(LDR_TR_PIN);
    sumBL += analogRead(LDR_BL_PIN);
    sumBR += analogRead(LDR_BR_PIN);
    delay(5);
  }

  const float averageTL = sumTL / (float)samples;
  const float averageTR = sumTR / (float)samples;
  const float averageBL = sumBL / (float)samples;
  const float averageBR = sumBR / (float)samples;

  if (averageTL < 10.0f || averageTR < 10.0f ||
      averageBL < 10.0f || averageBR < 10.0f) {
    Serial.println(F("LDR calibration failed: one or more readings are too low."));
    return;
  }

  const float overallAverage =
    (averageTL + averageTR + averageBL + averageBR) / 4.0f;

  ldrKTL = overallAverage / averageTL;
  ldrKTR = overallAverage / averageTR;
  ldrKBL = overallAverage / averageBL;
  ldrKBR = overallAverage / averageBR;

  ldrCalibrationValid = true;
  saveLdrCalibration();

  Serial.print(F("LDR calibration saved. Factors TL/TR/BL/BR="));
  Serial.print(ldrKTL, 5);
  Serial.print('/');
  Serial.print(ldrKTR, 5);
  Serial.print('/');
  Serial.print(ldrKBL, 5);
  Serial.print('/');
  Serial.println(ldrKBR, 5);
}

void eraseCalibrations() {
  LdrCalibrationData ldrData = {0, 1.0f, 1.0f, 1.0f, 1.0f};
  EEPROM.put(EEPROM_LDR_CAL_ADDRESS, ldrData);

  // The report-based load-cell calibration remains active and is not erased.
  loadLoadCalibration();

  ldrCalibrationValid = false;
  ldrKTL = 1.0f;
  ldrKTR = 1.0f;
  ldrKBL = 1.0f;
  ldrKBR = 1.0f;

  Serial.println(F("Saved LDR calibration erased."));
  Serial.println(F("The fixed Table 4 load-cell calibration remains active."));
}
