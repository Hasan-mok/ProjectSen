#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>
#include <DHT.h>
#include <HX711.h>

// --- PIN DEFINITIONS ---
const int ldrlt = A0;  // Top Left LDR (Swapped to A0)
const int ldrld = A1;  // Bottom Left LDR (Swapped to A1)
const int ldrrd = A2;  // Bottom Right LDR (Swapped to A2)
const int ldrrt = A3;  // Top Right LDR (Swapped to A3)

const int servoH_Pin = 2;  // Horizontal Servo (Azimuth)
const int servoV_Pin = 6;  // Vertical Servo (Tilt)
const int PIN_PUMP = 8;    // Pump Regulator Module Relay
const int PIN_DHT = 7;     // DHT22 Temperature & Humidity Sensor
const int PIN_HX711_CLK = 4; // Load Cell SCK
const int PIN_HX711_DT = 5;  // Load Cell DT
const int PIN_FLOW = 3;    // Flow Sensor Signal (Interrupt Pin)
const int PIN_DUST = 9;    // IR Dust Sensor

// --- CONFIGURATION PARAMETERS ---
const int limitH_Low = 10;
const int limitH_High = 170;
const int limitV_Low = 20;
const int limitV_High = 160;

const int tolerance = 20; 
const int trackingSpeed = 30; // Servo movement evaluation step (ms)
const int printInterval = 2000; // LCD & Serial refresh interval (ms)
const float calibrationFactor = 43.0; // Flowmeter scaling constant

// --- OBJECT INITIALIZATION ---
Servo servoHorizontal; 
Servo servoVertical;   
DHT dht(PIN_DHT, DHT22);
HX711 scale;
LiquidCrystal_I2C lcd(0x27, 20, 4); // Initialized for a 20x4 I2C LCD

// --- GLOBAL STATE VARIABLES ---
int angleH = 90; 
int angleV = 90;

volatile unsigned long pulseCounter = 0;
volatile unsigned long lastPulseTime = 0;
float flowRate = 0.0;

unsigned long lastTrackingTime = 0;
unsigned long lastPrintTime = 0;

// Interrupt Service Routine for Flow Meter with 2ms Software Debounce
void pulseCountISR() {
  unsigned long currentTime = micros();
  // Filter out electrical micro-bounces caused by mechanical relay clicks
  if (currentTime - lastPulseTime > 2000) { 
    pulseCounter++;
    lastPulseTime = currentTime;
  }
}

void setup() {
  Serial.begin(9600);
  Serial.println("--- System Initializing All Modules ---");
  
  // Initialize I2C LCD Screen
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Booting...");

  // Pin Configurations
  pinMode(PIN_DUST, INPUT);
  pinMode(PIN_PUMP, OUTPUT);
  digitalWrite(PIN_PUMP, LOW); // Force pump off by default safety rules

  // Attach and position Servos based on your baseline code parameters
  servoHorizontal.attach(servoH_Pin);
  servoVertical.attach(servoV_Pin);
  servoHorizontal.write(angleH);
  servoVertical.write(angleV);
  
  // Initialize Advanced Sensors
  dht.begin();
  scale.begin(PIN_HX711_DT, PIN_HX711_CLK);
  scale.set_scale(1.0); // Replace 1.0 with your exact calibration factor
  scale.tare();         // Zero the load cell on startup

  // Setup Flow Sensor hardware interrupt on Pin D3
  pinMode(PIN_FLOW, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_FLOW), pulseCountISR, FALLING);

  delay(1000); 
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  // --- 1. SOLAR TRACKING ENGINE (NON-BLOCKING TIMED INTERVAL) ---
  if (currentMillis - lastTrackingTime >= trackingSpeed) {
    lastTrackingTime = currentMillis;

    int ld = analogRead(ldrld); 
    int lt = analogRead(ldrlt); 
    int rt = analogRead(ldrrt); 
    int rd = analogRead(ldrrd); 
    
    int avgTop = (lt + rt) / 2;    
    int avgBottom = (ld + rd) / 2; 
    int avgLeft = (lt + ld) / 2;   
    int avgRight = (rt + rd) / 2;  
    
    int diffVertical = avgTop - avgBottom;
    int diffHorizontal = avgLeft - avgRight;

    // Vertical Control Tracking Axis
    if (abs(diffVertical) > tolerance) {
      if (avgTop > avgBottom) {
        angleV += 1; 
      } else {
        angleV -= 1; 
      }
      angleV = constrain(angleV, limitV_Low, limitV_High);
      servoVertical.write(angleV);
    }
    
    // Horizontal Control Tracking Axis
    if (abs(diffHorizontal) > tolerance) {
      if (avgLeft > avgRight) {
        angleH -= 1; 
      } else {
        angleH += 1; 
      }
      angleH = constrain(angleH, limitH_Low, limitH_High);
      servoHorizontal.write(angleH);
    }
  }

  // --- 2. SENSOR CONCURRENT SAMPLING ---
  float temp = dht.readTemperature();
  float humid = dht.readHumidity();
  
  // Fetch stable weight reading from scale (averaged over 5 cycles)
  float weight = scale.get_units(5); 
  if (weight < 0) weight = 0.0; // Omit small negative drift anomalies

  // Standard digital IR sensors pull LOW (0) when detecting an obstruction/dust layer
  bool dustDetected = (digitalRead(PIN_DUST) == LOW); 

  // --- 3. LOGIC CONTROLLER & CONDITIONAL PUMP INTERLOCK ---
  if (dustDetected) {
    if (weight > 650.0) {
      // Condition Met: Turn on the mechanical pump regulator module
      digitalWrite(PIN_PUMP, HIGH); 
      
      // Thread-safe copy of interrupt pulse data for flow evaluation
      unsigned long pulseCountCopy;
      noInterrupts();
      pulseCountCopy = pulseCounter;
      pulseCounter = 0;
      interrupts();
      
      // Flow Rate Equation: Calculates Liters/minute based on measurement window duration
      flowRate = ((pulseCountCopy / calibrationFactor) / (printInterval / 1000.0));
    } else {
      // Safety Fault Protection: Instantly cut pump power if weight is <= 650 grams
      digitalWrite(PIN_PUMP, LOW);
      flowRate = 0.0;
    }
  } else {
    // No dust detected: Maintain system standby
    digitalWrite(PIN_PUMP, LOW);
    flowRate = 0.0;
  }

  // --- 4. DATA PRESENTATION (LCD & SERIAL MONITOR OUT) ---
  if (currentMillis - lastPrintTime >= printInterval) {
    lastPrintTime = currentMillis;

    // --- Update 20x4 I2C LCD Screen Layout ---
    lcd.clear();
    
    // Line 0: Temperature & Humidity Status
    lcd.setCursor(0, 0);
    lcd.print("T:"); lcd.print(isnan(temp) ? 0.0 : temp, 1); lcd.print("C ");
    lcd.print("H:"); lcd.print(isnan(humid) ? 0.0 : humid, 1); lcd.print("%");

    // Line 1: Weight Scale Feedback
    lcd.setCursor(0, 1);
    lcd.print("Weight: "); lcd.print(weight, 1); lcd.print("g");

    // Line 2: Dust Assessment State
    lcd.setCursor(0, 2);
    if (dustDetected) {
      lcd.print("Dust: DETECTED");
    } else {
      lcd.print("Dust: CLEAN");
    }

    // Line 3: Conditional Operational Logic Screen Output
    lcd.setCursor(0, 3);
    if (dustDetected) {
      if (weight > 650.0) {
        lcd.print("Flow: "); lcd.print(flowRate, 2); lcd.print(" L/m");
      } else {
        lcd.print("no enough water"); // Matches requested error string exactly
      }
    } else {
      lcd.print("Pump: STANDBY");
    }

    // --- Diagnostic Output Streamed to Serial Monitor ---
    Serial.println("=========================================");
    Serial.print("Temp: "); Serial.print(temp); Serial.print("C | Humid: "); Serial.print(humid); Serial.println("%");
    Serial.print("Weight: "); Serial.print(weight); Serial.print("g | Dust Detected: "); Serial.println(dustDetected ? "YES" : "NO");
    Serial.print("Pump Relay State: "); Serial.print(digitalRead(PIN_PUMP) ? "ON" : "OFF");
    Serial.print(" | Flow Rate: "); Serial.print(flowRate); Serial.println(" L/min");
    Serial.print("Servos -> H: "); Serial.print(angleH); Serial.print("° | V: "); Serial.print(angleV); Serial.println("°");
  }
}