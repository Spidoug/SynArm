;// =====================================================================
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
const uint8_t ANGLE_DEADBAND = 2;      // ° – ignore micro-movements for direct servos
const uint8_t MAX_SERVOS = 16;         // Max servos supported by PCA9685
const uint16_t SERVOMIN = 102;         // Min PWM pulse (0 degrees)
const uint16_t SERVOMAX = 512;         // Max PWM pulse (180 degrees)

// Serial communication settings
const unsigned long SERIAL_BAUD_RATE = 115200;
const uint8_t SERIAL_BUFFER_SIZE = 32;

// Timing for sensor data transmission
const unsigned long SENSOR_PERIOD_MS = 4; // ms (≈ 250 Hz)

// --- Struct for Individual Servo Control ---
struct ServoControl {
  float currentAngle;          // Current servo angle (float for interpolation)
  uint8_t targetAngle;         // Target angle for the servo (integer)
  unsigned long lastStepMillis; // Last time the servo took a step
  float stepDegree;            // Degrees per step (0.0f for instantaneous movement)
  unsigned long stepPeriodMs;  // Interval in ms between each step (0 for instantaneous movement)
};

// --- Struct for Non-Blocking Stepper Motor Control ---
struct StepperControl {
  long targetSteps;            // Total number of steps to take (positive or negative)
  long currentStepsCount;      // Count of steps already taken for the current movement
  unsigned long lastStepMicros; // Last time a step pulse was sent (in microseconds)
  unsigned long stepPulseMicros; // Total pulse cycle duration in microseconds (for speed)
  bool isMoving;               // Indicates if the stepper motor is currently moving
};


// --- Global Variables / Drivers ---
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
MPU6050 mpu;

// Control array for each servo
ServoControl servos[MAX_SERVOS];

// Control for the stepper motor
StepperControl stepper;

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
 * @brief Writes a target angle to a specific servo channel.
 * Depending on the servo's 'stepDegree' and 'stepPeriodMs' configuration,
 * it either sets the target angle for interpolation or writes directly.
 * @param ch The servo channel (0 to MAX_SERVOS-1).
 * @param angle The target angle in degrees (0-180).
 */
void writeServo(uint8_t ch, uint8_t angle) {
  if (ch >= MAX_SERVOS || angle > 180) {
    Serial.println(F("⚠️ writeServo: Invalid channel or angle."));
    return;
  }

  // Set the target angle for the specific servo
  servos[ch].targetAngle = angle;

  // If the servo is not configured for smooth movement (instantaneous),
  // apply dead-band and update directly.
  // Otherwise, the update will occur in updateAllServos().
  if (servos[ch].stepDegree == 0.0f || servos[ch].stepPeriodMs == 0) {
    // Apply dead-band only for instantaneous movement servos
    // round() is used to prevent small float errors from causing repeated writes
    if (abs(angle - (uint8_t)round(servos[ch].currentAngle)) < ANGLE_DEADBAND) {
      return;
    }
    uint16_t pulse = safeMap(angle);
    pwm.setPWM(ch, 0, pulse);
    servos[ch].currentAngle = (float)angle; // Update the current angle
  }
}

/**
 * @brief Initializes the PCA9685 PWM driver in I2C fast-mode.
 */
void pcaSafeInit() {
  Wire.setClock(400000); // I²C fast-mode (400 kHz)
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY); // Library already sets MODE1 / MODE2
}

/**
 * @brief Initiates a non-blocking stepper motor movement.
 * @param steps The number of steps to move. Positive for one direction, negative for the other.
 * @param speedPulseMicros The total pulse cycle duration (HIGH + LOW) in microseconds.
 * A smaller value means faster movement. E.g., 1600us (800 high + 800 low).
 */
void startStepperMotorMove(long steps, unsigned long speedPulseMicros) {
  if (steps == 0) {
    stepper.isMoving = false;
    return;
  }
  digitalWrite(DIR_PIN, steps > 0 ? HIGH : LOW);
  stepper.targetSteps = abs(steps);
  stepper.currentStepsCount = 0;
  stepper.lastStepMicros = micros(); // Use micros() for higher precision
  stepper.stepPulseMicros = speedPulseMicros;
  stepper.isMoving = true;
}

/**
 * @brief Updates the stepper motor state, generating pulses in a non-blocking manner.
 * This function should be called repeatedly in the main loop().
 */
void updateStepperMotor() {
  if (!stepper.isMoving) return;

  if (micros() - stepper.lastStepMicros >= stepper.stepPulseMicros / 2) { // Half-cycle for HIGH/LOW
    // Toggle the STEP pin state
    if (digitalRead(STEP_PIN) == LOW) {
      digitalWrite(STEP_PIN, HIGH);
    } else {
      digitalWrite(STEP_PIN, LOW);
      // If it just went from HIGH to LOW, a complete step has been performed
      stepper.currentStepsCount++;
    }

    stepper.lastStepMicros = micros();

    // Check if all steps are completed
    if (stepper.currentStepsCount >= stepper.targetSteps) {
      stepper.isMoving = false;
      digitalWrite(STEP_PIN, LOW); // Ensure STEP pin is LOW at the end
    }
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
        // The M<steps> command now initiates the stepper motor movement
        // We'll use a default speed here, but you can extend the serial
        // protocol to include speed (e.g., M<steps>V<speed>)
        long steps = atol(serialBuffer + 1);
        const unsigned long DEFAULT_STEPPER_SPEED_US = 1600; // 800us HIGH + 800us LOW = 1600us per step
        startStepperMotorMove(steps, DEFAULT_STEPPER_SPEED_US);
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
  }
}

/**
 * @brief Gradually updates the angles of servos configured for smooth movement.
 * Iterates through all servos and applies interpolation if 'stepDegree' > 0.
 */
void updateAllServos() {
  for (uint8_t i = 0; i < MAX_SERVOS; ++i) {
    // Only process servos configured for smooth movement
    if (servos[i].stepDegree > 0.0f && servos[i].stepPeriodMs > 0) {
      if (millis() - servos[i].lastStepMillis >= servos[i].stepPeriodMs) {
        if (abs(servos[i].targetAngle - servos[i].currentAngle) > 0.1f) { // If target not reached
          if (servos[i].targetAngle > servos[i].currentAngle) {
            servos[i].currentAngle += servos[i].stepDegree;
            if (servos[i].currentAngle > servos[i].targetAngle) { // Don't overshoot
              servos[i].currentAngle = (float)servos[i].targetAngle;
            }
          } else {
            servos[i].currentAngle -= servos[i].stepDegree;
            if (servos[i].currentAngle < servos[i].targetAngle) { // Don't overshoot
              servos[i].currentAngle = (float)servos[i].targetAngle;
            }
          }

          // Ensure the current angle is within limits
          servos[i].currentAngle = constrain(servos[i].currentAngle, 0.0f, 180.0f);

          // Send the new pulse to the servo
          uint16_t pulse = safeMap((uint8_t)round(servos[i].currentAngle));
          pwm.setPWM(i, 0, pulse);
        }
        servos[i].lastStepMillis = millis();
      }
    }
  }
}

// --- Arduino Setup & Loop ---

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  Wire.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(STEP_PIN, LOW); // Ensure STEP pin starts LOW

  Serial.println();
  Serial.println(F("SynArm"));
  Serial.println(F("Version 1.00 - Non-Blocking Control"));
  Serial.println();

  pcaSafeInit();

  // --- Servo Speed Configuration ---
  // Initialize all servos for instantaneous movement (stepDegree = 0, stepPeriodMs = 0)
  // and set their initial position.
  for (uint8_t i = 0; i < MAX_SERVOS; ++i) {
    servos[i].currentAngle = 90.0f; // Initial position
    servos[i].targetAngle = 90;
    servos[i].lastStepMillis = 0;
    servos[i].stepDegree = 0.0f;     // Instantaneous movement by default
    servos[i].stepPeriodMs = 0;     // Instantaneous movement by default

    // Set initial pulse for all servos
    uint16_t initialPulse = safeMap(90);
    pwm.setPWM(i, 0, initialPulse);
  }

  // --- Adjust specific servos here ---
  // Example: Shoulder Servo (channel 1) - slower and smoother
  servos[1].stepDegree = 1.0f;          // Move 1 degree per step
  servos[1].stepPeriodMs = 15;          // Every 15 ms (slower and smoother)

  // Example: "Lift" Servo (e.g., channel 2) - slightly faster than shoulder, but still smooth
  servos[2].stepDegree = 3.0f;          // Move 3 degrees per step (faster)
  servos[2].stepPeriodMs = 8;           // Every 8 ms (more frequent)

  // Example: "Rotate" Servo (e.g., channel 3) - quite fast, almost instantaneous
  servos[3].stepDegree = 7.0f;          // Move 7 degrees per step
  servos[3].stepPeriodMs = 3;           // Every 3 ms (almost instantaneous, but still smooth)

  // Initialize stepper motor control
  stepper.isMoving = false;
  stepper.currentStepsCount = 0;
  stepper.targetSteps = 0;
  stepper.lastStepMicros = 0;
  stepper.stepPulseMicros = 0;


  // MPU6050 Initialization
  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.println(mpuOk ? F("✅ MPU6050 detected.") : F("❌ MPU6050 not found!"));
  Serial.println();
}

void loop() {
  processSerialCommand();
  updateAllServos();    // Update all servos
  updateStepperMotor(); // Update the stepper motor (if moving)

  if (systemActivated && (millis() - lastSensorTxMillis >= SENSOR_PERIOD_MS)) {
    lastSensorTxMillis = millis();
    sendSensorData();
  }
}
