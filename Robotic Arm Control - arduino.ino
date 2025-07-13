// =====================================================================
//  SynArm
//  Robotic Arm Control – Firmware
// ---------------------------------------------------------------------
//  Douglas Santana (SPIDOUG)
// =====================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>

// --- Constants & Configuration ---
constexpr uint8_t PCA9685_ADDR = 0x40;
constexpr uint8_t STEP_PIN = 6;
constexpr uint8_t DIR_PIN  = 5;

constexpr float PWM_FREQUENCY = 50.0f;     // 50 Hz – standard RC-servo rate
constexpr uint8_t ANGLE_DEADBAND = 2;      // ° – ignore micro movements
constexpr uint8_t MAX_SERVOS = 16;
constexpr uint16_t SERVOMIN = 150;
constexpr uint16_t SERVOMAX = 600;

constexpr unsigned long SERIAL_BAUD_RATE = 115200;
constexpr uint8_t SERIAL_BUFFER_SIZE = 32;
constexpr unsigned long SENSOR_PERIOD_MS = 4; // ms (≈ 250 Hz)

// --- Global State ---
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
MPU6050 mpu;

uint16_t currentPulse[MAX_SERVOS] = {0};
uint8_t currentAngle[MAX_SERVOS] = {0xFF};
bool mpuOk = false;
bool systemActivated = false;
unsigned long lastSensorTxMillis = 0;

// --- Utility Functions ---

uint16_t safeMap(uint8_t angle) {
  long pulse = map((long)angle, 0, 180, (long)SERVOMIN, (long)SERVOMAX);
  return constrain(pulse, 1, 4095);
}

void writeServo(uint8_t ch, uint8_t angle) {
  if (ch >= MAX_SERVOS || angle > 180) {
    Serial.println(F("writeServo: invalid channel or angle."));
    return;
  }
  if (abs(angle - currentAngle[ch]) < ANGLE_DEADBAND) return;

  uint16_t pulse = safeMap(angle);
  pwm.setPWM(ch, 0, pulse);
  currentPulse[ch] = pulse;
  currentAngle[ch] = angle;
}

void pcaSafeInit() {
  Wire.setClock(400000);
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);
}

void moveStepperMotor(long steps) {
  if (steps == 0) return;

  digitalWrite(DIR_PIN, steps > 0);
  for (unsigned long i = 0; i < abs(steps); ++i) {
    digitalWrite(STEP_PIN, HIGH); delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);  delayMicroseconds(800);
  }
}

void sendSensorData() {
  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (mpuOk) mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  Serial.print(F("#SENS|MPU:"));
  Serial.print(ax); Serial.print(F(",")); Serial.print(ay); Serial.print(F(",")); Serial.print(az); Serial.print(F(","));
  Serial.print(gx); Serial.print(F(",")); Serial.print(gy); Serial.print(F(",")); Serial.print(gz);

  Serial.print(F("|AN:"));
  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(analogRead(i));
    if (i < 5) Serial.print(',');
  }

  Serial.println(F("|EX1:0,0,0|EX2:0,0,0"));
}

// --- Serial Command Parser ---

void handleServoCommand(const char* command) {
  uint8_t ch = 0, angle = 0;
  if (sscanf(command, "S%hhuP%hhu", &ch, &angle) == 2) {
    writeServo(ch, angle);
  } else {
    Serial.println(F("Invalid servo command. Use S<ch>P<angle>."));
  }
}

void handleStepperCommand(const char* command) {
  long steps = atol(command + 1);
  moveStepperMotor(steps);
}

void processSerialCommand() {
  static char buffer[SERIAL_BUFFER_SIZE];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buffer[idx] = '\0';
      idx = 0;

      if (strncmp(buffer, "S", 1) == 0 && strchr(buffer, 'P')) {
        handleServoCommand(buffer);
      } else if (buffer[0] == 'M') {
        handleStepperCommand(buffer);
      } else if (strcmp(buffer, "READY?") == 0) {
        Serial.println(F("READY!"));
        systemActivated = true;
      } else {
        Serial.println(F("Unknown command."));
      }
    } else if (idx < SERIAL_BUFFER_SIZE - 1) {
      buffer[idx++] = c;
    }
  }
}

// --- Setup & Loop ---

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.println();
  delay(500);
  Serial.println(F("SynArm\n\nVersion 1.00\n"));

  pcaSafeInit();
  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.println(mpuOk ? F("✅ MPU6050 detected.") : F("MPU6050 not found!"));
  Serial.println();
}

void loop() {
  processSerialCommand();

  if (systemActivated && (millis() - lastSensorTxMillis >= SENSOR_PERIOD_MS)) {
    lastSensorTxMillis = millis();
    sendSensorData();
  }
}
