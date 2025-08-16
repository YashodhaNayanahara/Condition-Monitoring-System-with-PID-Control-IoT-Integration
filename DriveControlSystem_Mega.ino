/*
------------------------------------------------------------
 Project:   Condition Monitoring System with PID Control
 Author:    Yashodha Nayanahara
 Contact:   [yashodha.abc@gmail.com] | [https://www.linkedin.com/in/yashodha-nayanahara]
 Date:      [16-08-2025]
------------------------------------------------------------
 Description:
 This project implements a condition monitoring system for DC motors with PID-based speed control. The system integrates multiple sensors (vibration, temperature, current, smoke) connected to Arduino Mega, with ESP32 for IoT cloud connectivity. Manual PID tuning was performed for stable control under real-world conditions.

 Key Features:
 - Motor speed control using PID
 - Real-time monitoring of vibration, temperature, current, smoke
 - Serial communication between Mega and ESP32
 - Cloud upload via Arduino IoT Cloud
 - Modular and expandable design

 Notes:
 Developed as part of an academic/industrial project. Use industrial-grade sensors for improved accuracy in real-world applications.
------------------------------------------------------------
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <dht.h>

#define DHT11_PIN 16
dht DHT;

LiquidCrystal_I2C lcd(0x27, 16, 2);

int ENA = 9;
int IN1 = 4;
int IN2 = 5;
int encoderANo = 3;

int RGBLED = 7;        // Temperature alert
int Blue = 11;   // Vibration alert
int Green = 12;  // Current alert
int Red = 13;    // Temperature alert

const int currentPinNo = A0;
const int gasSensorPinNo = A1;

const int gasInterruptPinNo = 2;
volatile bool gasAlertTriggered = false;
const int gasThreshold = 200;

volatile long pulseCount = 0;
unsigned long lastTime = 0;
const unsigned long interval = 1000;
const int countsPerRevolution = 180;

int pwmValue;
float setpointRPM1 = 50.0;
float setpointRPM2 = 120.0;
float currentSetpoint = setpointRPM1;

float Kp = 0.41, Ki = 0.001, Kd = -0.0001;  //Manual Values

float integral = 0.0;
float previousError = 0.0;

float previousZPosition, currentZPosition;
float previousTemp, currentTemp;
float previousCurrent, currentCurrent;

const unsigned long changeInterval = 15000;
unsigned long lastChangeTime = 0;

const int MPU6050_ADDR = 0x68;
int16_t AccX, AccY, AccZ;

int sensitivity = 185;
int adcValue = 0;
int offsetVoltage = 2500;
double adcVoltage = 0;
double currentValue = 0;

bool SensorsCalib = false;
unsigned long calibStartTime = 0;

const int emergencyButtonNo = 8;
bool emergencyPressed = false;
bool lastButtonState = LOW;

bool tempAlertTriggered = false;
bool currentAlertTriggered = false;
bool vibrationAlertTriggered = false;
bool Status = false;

const float TempChange = 3;
const float CurrentChange = 2000;  // Current Change 2 Amps
const long VibrationThreshold = 5000;

void setup() {
  Serial.begin(115200);
  Serial1.begin(115200);

  lcd.init();
  lcd.backlight();

  Wire.begin();
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(encoderANo, INPUT);
  pinMode(Blue, OUTPUT);
  pinMode(Green, OUTPUT);
  pinMode(Red, OUTPUT);
  pinMode(RGBLED, OUTPUT);
  pinMode(emergencyButtonNo, INPUT_PULLUP);
  pinMode(gasSensorPinNo, INPUT);
  pinMode(gasInterruptPinNo, INPUT);

  attachInterrupt(digitalPinToInterrupt(encoderANo), updateEncoder, CHANGE);
  lcd.setCursor(0, 0);
  lcd.print(" Drive  Control");
  lcd.setCursor(0, 1);
  lcd.print("     System");
  delay(2000);
  lcd.clear();
  calibStartTime = millis();
}

void loop() {

  if (emergencyPressed) {
    analogWrite(ENA, 0);
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("EMERGENCY STOP");
    lcd.setCursor(0, 1);
    lcd.print("System Halted");

    digitalWrite(Red, HIGH);
    digitalWrite(RGBLED, HIGH);
    digitalWrite(Blue, LOW);
    digitalWrite(Green, LOW);

    return;
  }

  unsigned long currentTime = millis();
  int chk = DHT.read11(DHT11_PIN);
  int gasValue = analogRead(gasSensorPinNo);

  currentTemp = DHT.temperature;
  currentCurrent = currentValue * 10;

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  AccZ = (Wire.read() << 8 | Wire.read());
  previousZPosition = currentZPosition;
  currentZPosition = AccZ;

  if (!SensorsCalib && (currentTime - calibStartTime >= 10000)) {
    previousTemp = currentTemp;
    previousCurrent = currentCurrent;
    SensorsCalib = true;
  }

  checkTemperatureChange();
  checkCurrentChange();
  checkVibrationChange();
  checkGasLevelInterrupt();

  if (currentTime - lastChangeTime >= changeInterval) {
    lastChangeTime = currentTime;
    currentSetpoint = (currentSetpoint == setpointRPM1) ? setpointRPM2 : setpointRPM1;
  }

  analogWrite(ENA, pwmValue);
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;

    lcd.setCursor(4, 0);
    lcd.print("     ");
    lcd.setCursor(13, 0);
    lcd.print("   ");

    float actualRPM = (pulseCount / (float)countsPerRevolution) * 60.0;
    float error = currentSetpoint - actualRPM;
    integral += error * (interval / 1000.0);
    float derivative = (error - previousError) / (interval / 1000.0);
    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    previousError = error;

    pwmValue = constrain(pwmValue + (int)output, 0, 150);
    pulseCount = 0;

    lcd.setCursor(0, 0);
    lcd.print("Set:");
    lcd.setCursor(9, 0);
    lcd.print("Act:");
    lcd.setCursor(0, 1);
    lcd.print("Status: ");

    unsigned int currentSetpointDisplay = int(trunc(currentSetpoint));
    unsigned int actualRPMDisplay = int(trunc(actualRPM));
    unsigned int currentTempDisplay = int(trunc(currentTemp));

    Serial.print("SetpointRPM:");
    Serial.println(currentSetpointDisplay);
    Serial.print("ActualRPM:");
    Serial.println(actualRPM, 1);
    Serial.print("error:");
    Serial.println(error);

    lcd.setCursor(4, 0);
    lcd.print(currentSetpointDisplay);
    lcd.setCursor(13, 0);
    lcd.print(actualRPMDisplay);

    Serial1.print("RPM:");
    Serial1.print(actualRPMDisplay);
    Serial1.print(",Setpoint:");
    Serial1.print(currentSetpointDisplay);
    Serial1.print(",Gas:");
    Serial1.print(gasValue);
    Serial1.print(",Err:");
    Serial1.print(error);
    Serial1.print(",VibZ:");
    Serial1.print(abs(currentZPosition));
    Serial1.print(",Temp:");
    Serial1.print(currentTemp, 1);
    Serial1.print(",Current:");
    Serial1.println(currentCurrent, 2);

    adcValue = analogRead(currentPinNo);
    adcVoltage = (adcValue / 1024.0) * 5000;
    currentValue = ((adcVoltage - offsetVoltage) / sensitivity);
  }

  if (Status == false) {
    lcd.setCursor(8, 1);
    lcd.print("Normal");
    digitalWrite(Green, HIGH);
    digitalWrite(RGBLED, LOW);
  }
  if (Status == true) {
    lcd.setCursor(8, 1);
    lcd.print("Abnormal");
    digitalWrite(Green, LOW);
    digitalWrite(RGBLED, HIGH);
  }

  bool currentButtonState = digitalRead(emergencyButtonNo);

  if (currentButtonState == LOW && lastButtonState == HIGH) {
    emergencyPressed = true;
  }

  lastButtonState = currentButtonState;
}

void updateEncoder() {
  pulseCount++;
}

void checkTemperatureChange() {
  if (!SensorsCalib) return;

  float tempDiff = currentTemp - previousTemp;

  if (tempDiff >= TempChange && !tempAlertTriggered) {
    digitalWrite(Red, HIGH);
    digitalWrite(Blue, HIGH);
    tempAlertTriggered = true;
    Status = true;
  }

  if (tempDiff < TempChange && tempAlertTriggered) {
    digitalWrite(Red, LOW);
    tempAlertTriggered = false;
  }
}

void checkCurrentChange() {
  if (!SensorsCalib) return;

  float currentDiff = currentCurrent - previousCurrent;

  if (currentDiff >= CurrentChange && !currentAlertTriggered) {
    digitalWrite(Green, HIGH);
    digitalWrite(Red, HIGH);
    currentAlertTriggered = true;
  }

  if (currentDiff < CurrentChange && currentAlertTriggered) {
    digitalWrite(Green, LOW);
    currentAlertTriggered = false;
    Status = true;
  }
}

void checkVibrationChange() {
  if (!SensorsCalib) return;

  long vibrationDiff = abs(currentZPosition - previousZPosition);

  if (vibrationDiff >= VibrationThreshold && !vibrationAlertTriggered) {
    digitalWrite(Blue, HIGH);
    vibrationAlertTriggered = true;
    Status = true;
  }
}

void checkGasLevelInterrupt() {
  int gasValue = analogRead(gasSensorPinNo);

  if (gasValue >= gasThreshold && !gasAlertTriggered) {
    digitalWrite(Red, HIGH);
    gasAlertTriggered = true;
    Status = true;
  }
}
