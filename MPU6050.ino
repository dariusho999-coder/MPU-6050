#include <Wire.h>
#include <LiquidCrystal.h>

// ================= LCD =================
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// ================= Motors =================
const int ENA = 3;
const int IN1 = 1;
const int IN2 = 2;

const int ENB = 11;
const int IN3 = 12;
const int IN4 = 13;

// ================= MPU6050 =================
const int MPU = 0x68;
long accX, accY, accZ;
long gyroX, gyroY, gyroZ;

float pitch = 0;
float elapsedTime, currentTime, previousTime;

// ================= TIMERS =================
unsigned long lcdTimer = 0;
unsigned long stateTimer = 0;

// ================= STATES =================
enum RobotState { MOVING_FORWARD, ASCENDING_RAMP, STOP_TOP, ROTATE_360, MOVING_DOWN, FINAL_STOP };
RobotState robotState = MOVING_FORWARD;

bool hasClimbed = false;

// ===== NEW VARIABLE: max angle =====
float maxAngle = 0;

// ============ SMOOTHING ============
float smoothAngle(float input) {
  static float buffer[10] = {0};
  static int index = 0;

  buffer[index] = input;
  index = (index + 1) % 10;

  float sum = 0;
  for (int i = 0; i < 10; i++) sum += buffer[i];

  return sum / 10.0;
}

// ============ GET TILT ANGLE (UNCHANGED) ============
float getTiltAngle() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 14, true);

  accX = (Wire.read() << 8) | Wire.read();
  accY = (Wire.read() << 8) | Wire.read();
  accZ = (Wire.read() << 8) | Wire.read();
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();

  float accAngleY = atan(-accX / sqrt(pow(accY, 2) + pow(accZ, 2))) * 180.0 / PI;
  float gyroRateY = gyroY / 131.0;
  pitch = 0.95 * (pitch + gyroRateY * elapsedTime) + 0.05 * accAngleY;

  currentTime = millis();
  elapsedTime = (currentTime - previousTime) * 0.001;
  if (elapsedTime <= 0) elapsedTime = 0.001;
  previousTime = currentTime;

  return pitch;
}

// ============ MOTOR FUNCTIONS ============
void moveForwardFunc() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 240);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 220);
}

void moveSlowFunc() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 180);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 160);
}

void ascendingFunc() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 200);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW); analogWrite(ENB, 255);
}

void stopMotorsFunc() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

void rotate360Func() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW); analogWrite(ENA, 220);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, 220);
}

// ================= SETUP =================
void setup() {
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  lcd.begin(16, 2);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  previousTime = millis();

  lcd.print("System Ready");
  delay(1000);
  lcd.clear();
}

// ============ LOOP ============
void loop() {
  float rawAngle = getTiltAngle();
  float angle = smoothAngle(rawAngle);

  // ====== UPDATE MAX ANGLE ======
  if (angle > maxAngle) maxAngle = angle;

  // ===== LCD Update =====
  if (millis() - lcdTimer > 250) {
    lcdTimer = millis();
    
    lcd.setCursor(0, 0);
    lcd.print("Tilt:");
    lcd.print(angle, 1);
    lcd.print(" deg   ");

    lcd.setCursor(0, 1);
    switch (robotState) {
      case MOVING_FORWARD: lcd.print("MOVING        "); break;
      case ASCENDING_RAMP: lcd.print("ASCENDING     "); break;
      case STOP_TOP:       lcd.print("TOP 4s WAIT   "); break;
      case ROTATE_360:     lcd.print("ROTATING      "); break;
      case FINAL_STOP:     lcd.print("STOPPED       "); break;
    }
  }

  // ===== STATE MACHINE =====
  switch (robotState) {

    case MOVING_FORWARD:
      moveForwardFunc();
      if (angle > 20.0) {
        robotState = ASCENDING_RAMP;
        hasClimbed = false;
      }
      break;

    case ASCENDING_RAMP:
      ascendingFunc();
      if (angle > 20.0) hasClimbed = true;
      if (hasClimbed && angle < 2.0) {
        robotState = STOP_TOP;
        stateTimer = millis();
      }
      break;

    case STOP_TOP:
      stopMotorsFunc();
      if (millis() - stateTimer >= 4000) {
        robotState = ROTATE_360;
        stateTimer = millis();
      }
      break;

    case ROTATE_360:
      rotate360Func();
      if (millis() - stateTimer >= 2400) {
        robotState = FINAL_STOP;
      }
      break;

    case FINAL_STOP:
      stopMotorsFunc();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("MAX ANGLE:");
        lcd.print(maxAngle-7, 1);
      break;
  }
}
