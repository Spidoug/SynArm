// =====================================================================
//  SynArm
//  Robotic Arm Control – Firmware
// ---------------------------------------------------------------------
//  ✉ Douglas Santana (SPIDOUG)
// =====================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h> // https://github.com/jrowberg/i2cdevlib

// --- Constants & Configuration ---
#define PCA9685_ADDR    0x40
#define STEP_PIN        6
#define DIR_PIN         5

const float PWM_FREQUENCY = 50.0f;     // 50 Hz – standard RC-servo rate
const uint8_t ANGLE_DEADBAND = 2;      // ° – ignore micro movements
const uint8_t MAX_SERVOS = 16;         // Max servos supported by PCA9685
const uint16_t SERVOMIN = 150;         // Min PWM pulse (0 degrees)
const uint16_t SERVOMAX = 600;         // Max PWM pulse (180 degrees)

// Serial communication settings
const unsigned long SERIAL_BAUD_RATE = 115200;
const uint8_t SERIAL_BUFFER_SIZE = 32;

// Timing for sensor data transmission
const unsigned long SENSOR_PERIOD_MS = 4; // ms (≈ 250 Hz)

// --- Global Variables / Drivers ---
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
MPU6050 mpu;

uint16_t currentPulse[MAX_SERVOS] = {0};
uint8_t currentAngle[MAX_SERVOS] = {0xFF}; // 0xFF = "impossible" -> forces first write
bool mpuOk = false;
bool systemActivated = false;

unsigned long lastSensorTxMillis = 0;

// --- Helper Functions ---

/**
 * @brief Safely maps an angle (0-180) to a PWM pulse width.
 * Ensures the pulse is never full-off (0) or full-on (4096) to prevent issues.
 * @param angle The desired angle in degrees (0-180).
 * @return The corresponding PWM pulse value (1-4095).
 */
uint16_t safeMap(uint8_t angle) {
  long pulse = map((long)angle, 0, 180, (long)SERVOMIN, (long)SERVOMAX);
  return (uint16_t)constrain(pulse, 1, 4095); // Ensure pulse is always valid
}

/**
 * @brief Writes a target angle to a specific servo channel with dead-band filtering.
 * Prevents minor angle changes from causing jittering.
 * @param ch The servo channel (0 to MAX_SERVOS-1).
 * @param angle The target angle in degrees (0-180).
 */
void writeServo(uint8_t ch, uint8_t angle) {
  if (ch >= MAX_SERVOS || angle > 180) {
    Serial.println(F("⚠️ writeServo: Invalid channel or angle."));
    return;
  }

  // Apply dead-band filtering
  if (abs(angle - currentAngle[ch]) < ANGLE_DEADBAND) {
    return;
  }

  uint16_t pulse = safeMap(angle);
  pwm.setPWM(ch, 0, pulse);
  currentPulse[ch] = pulse;
  currentAngle[ch] = angle;
}

/**
 * @brief Initializes the PCA9685 PWM driver in fast-mode I2C.
 */
void pcaSafeInit() {
  Wire.setClock(400000); // I²C fast-mode (400 kHz)
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY); // Library already sets MODE1 / MODE2
}

/**
 * @brief Controls a stepper motor by generating step pulses.
 * This function is blocking; consider a non-blocking alternative for complex tasks.
 * @param steps The number of steps to move. Positive for one direction, negative for the other.
 */
void moveStepperMotor(long steps) {
  if (steps == 0) return;

  digitalWrite(DIR_PIN, steps > 0 ? HIGH : LOW);
  unsigned long absSteps = abs(steps);

  for (unsigned long i = 0; i < absSteps; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800); // Adjust for desired speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800); // Adjust for desired speed
  }
}

/**
 * @brief Sends formatted sensor data over serial.
 * Includes MPU6050 (accelerometer/gyro) and analog pin readings.
 * Format: #SENS|MPU:ax,ay,az,gx,gy,gz|AN:A0,A1,A2,A3,A4,A5|EX1:0,0,0|EX2:0,0,0\n
 */
void sendSensorData() {
  int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (mpuOk) {
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  }

  Serial.print(F("#SENS|MPU:"));
  Serial.print(ax); Serial.print(F(","));
  Serial.print(ay); Serial.print(F(","));
  Serial.print(az); Serial.print(F(","));
  Serial.print(gx); Serial.print(F(","));
  Serial.print(gy); Serial.print(F(","));
  Serial.print(gz);

  Serial.print(F("|AN:"));
  for (uint8_t i = 0; i < 6; ++i) {
    Serial.print(analogRead(i));
    if (i < 5) Serial.print(F(","));
  }
  Serial.print(F("|EX1:0,0,0|EX2:0,0,0\n")); // Placeholders for future expansion
}

// --- Serial Command Processing ---

/**
 * @brief Processes incoming serial commands in a non-blocking manner.
 * Supports servo control (S<ch>P<angle>), stepper motor control (M<steps>),
 * and system activation (READY?).
 */
void processSerialCommand() {
  static char serialBuffer[SERIAL_BUFFER_SIZE];
  static uint8_t bufferIndex = 0;

  while (Serial.available()) {
    char receivedChar = Serial.read();

    if (receivedChar == '\n' || receivedChar == '\r') {
      // End of command
      serialBuffer[bufferIndex] = '\0'; // Null-terminate the string
      bufferIndex = 0; // Reset for next command

      // --- PARSE COMMAND ---
      if (serialBuffer[0] == 'S' && strchr(serialBuffer, 'P')) { // S<ch>P<angle>
        char* token = strtok(serialBuffer + 1, "P");
        if (token != NULL) {
          uint8_t servoChannel = atoi(token);
          token = strtok(NULL, "P");
          if (token != NULL) {
            uint8_t angle = atoi(token);
            writeServo(servoChannel, angle);
          } else {
            Serial.println(F("⚠️ S: Missing angle."));
          }
        } else {
          Serial.println(F("⚠️ S: Missing channel."));
        }
      } else if (serialBuffer[0] == 'M') { // M<steps>
        long steps = atol(serialBuffer + 1);
        moveStepperMotor(steps);
      } else if (strcmp(serialBuffer, "READY?") == 0) {
        Serial.println(F("READY!"));
        systemActivated = true;
      } else {
        Serial.println(F("⚠️ Invalid command."));
      }
    } else if (bufferIndex < (SERIAL_BUFFER_SIZE - 1)) {
      // Add character to buffer if space is available
      serialBuffer[bufferIndex++] = receivedChar;
    }
    // If bufferIndex is >= SERIAL_BUFFER_SIZE - 1, the char is ignored (buffer overflow protection)
  }
}

// --- Arduino Setup & Loop ---

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);

  Serial.println();
  delay(500);
  Serial.println(F("SynArm"));
  delay(500);
  Serial.println(F("Version 1.00"));
  Serial.println();

  pcaSafeInit();

  // Force first write on every channel by setting to an "impossible" value
  for (uint8_t i = 0; i < MAX_SERVOS; ++i) {
    currentAngle[i] = 0xFF;
  }

  // MPU6050 Initialization
  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.println(mpuOk ? F("✅ MPU6050 detected.") : F("❌ MPU6050 not found!"));
  Serial.println();
}

void loop() {
  processSerialCommand();

  if (systemActivated && (millis() - lastSensorTxMillis >= SENSOR_PERIOD_MS)) {
    lastSensorTxMillis = millis();
    sendSensorData();
  }
}
