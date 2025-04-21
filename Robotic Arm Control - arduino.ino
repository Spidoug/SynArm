// =====================================================================
//  SynArm
//  Robotic Arm Control – Leap Motion · Joystick · Keyboard · WebSockets
// ---------------------------------------------------------------------
//  ✉ Douglas Santana (SPIDOUG)
// =====================================================================

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050.h>          // https://github.com/jrowberg/i2cdevlib

// ------------------------------------------------------------------
// 🏷️  Constants & configuration
// ------------------------------------------------------------------
#define PCA9685_ADDR      0x40
#define STEP_PIN          6
#define DIR_PIN           5

const float   PWM_FREQUENCY   = 50.0f;   // 50 Hz – standard RC‑servo rate
const uint8_t ANGLE_DEADBAND  = 2;       // ° – ignore micro movements
const uint8_t MAX_SERVOS      = 16;

// Single pulse range (0 – 180 °) applied to every servo
#define SERVOMIN 102   
#define SERVOMAX 512
// ------------------------------------------------------------------
// 🌐  Drivers / global variables
// ------------------------------------------------------------------
Adafruit_PWMServoDriver pwm(PCA9685_ADDR);
MPU6050 mpu;

uint16_t currentPulse[MAX_SERVOS]  = {0};
uint8_t  currentAngle[MAX_SERVOS]  = {255};  // 255 = “impossible” → forces first write
bool     mpuOk           = false;
bool     systemActivated = false;

// Timing
unsigned long lastSensorTx = 0;
const unsigned long SENSOR_PERIOD = 4;   // ms  (≈ 250 Hz)

// ------------------------------------------------------------------
// 🛠️  Helper – safe map (never 0 or 4096)
// ------------------------------------------------------------------
uint16_t safeMap(uint8_t angle) {
  long pulse = map((long)angle, 0, 180, (long)SERVOMIN, (long)SERVOMAX);
  pulse = constrain(pulse, 1, 4095);         // never full‑off / full‑on
  return (uint16_t)pulse;
}

// ------------------------------------------------------------------
// 🔧  Write a servo with dead‑band filtering
// ------------------------------------------------------------------
void writeServo(uint8_t ch, uint8_t angle) {
  if (ch >= MAX_SERVOS || angle > 180) return;

  if (abs(angle - currentAngle[ch]) < ANGLE_DEADBAND) return;

  uint16_t pulse = safeMap(angle);
  pwm.setPWM(ch, 0, pulse);
  currentPulse[ch] = pulse;
  currentAngle[ch] = angle;
}

// ------------------------------------------------------------------
// 🚀  PCA9685 initialisation
// ------------------------------------------------------------------
void pcaSafeInit() {
  Wire.setClock(400000);          // I²C fast‑mode (400 kHz)
  pwm.begin();
  pwm.setPWMFreq(PWM_FREQUENCY);  // Library already sets MODE1 / MODE2
}

// ------------------------------------------------------------------
// 📡  Non‑blocking serial parser
// ------------------------------------------------------------------
void processSerialCommand() {
  static char buf[32];
  static uint8_t idx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {            // end of command
      buf[idx] = '\0';
      idx = 0;

      // ------------ PARSE ------------
      if (buf[0] == 'S' && strchr(buf, 'P')) {       // S<n>P<angle>
        uint8_t servo = atoi(strtok(buf + 1, "P"));
        uint8_t angle = atoi(strtok(NULL,   "P"));
        writeServo(servo, angle);
      }
      else if (buf[0] == 'M') {                     // M<steps>
        long steps = atol(buf + 1);
        moveStepperMotor(steps);
      }
      else if (strcmp(buf, "READY?") == 0) {
        Serial.println("READY!");
        systemActivated = true;
      }
      else {
        Serial.println(F("⚠️ Invalid command."));
      }
    }
    else if (idx < sizeof(buf) - 1) {
      buf[idx++] = c;
    }
  }
}

// ------------------------------------------------------------------
// 🔄  Stepper‑motor control (DIR / STEP)
// ------------------------------------------------------------------
void moveStepperMotor(long steps) {
  if (steps == 0) return;
  digitalWrite(DIR_PIN, steps > 0 ? HIGH : LOW);
  steps = abs(steps);
  for (long i = 0; i < steps; ++i) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(800);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(800);
  }
}

// ------------------------------------------------------------------
// 📤  Sensor packet  #SENS|MPU:…|AN:…
// ------------------------------------------------------------------
void sendSensorData() {
  int16_t ax=0, ay=0, az=0, gx=0, gy=0, gz=0;
  if (mpuOk) mpu.getMotion6(&ax,&ay,&az,&gx,&gy,&gz);

  Serial.print(F("#SENS|MPU:"));
  Serial.print(ax); Serial.print(','); Serial.print(ay); Serial.print(','); Serial.print(az); Serial.print(',');
  Serial.print(gx); Serial.print(','); Serial.print(gy); Serial.print(','); Serial.print(gz);
  Serial.print(F("|AN:"));
  for (uint8_t i=0;i<6;++i){
    Serial.print(analogRead(i));
    if (i<5) Serial.print(',');
  }
  Serial.print(F("|EX1:0,0,0|EX2:0,0,0\n"));
}

// ------------------------------------------------------------------
// 🏁  setup()
// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN,  OUTPUT);

  Serial.println();
  Serial.println(F("SynArm"));
  Serial.println(F("version 1.00"));
  Serial.println();

  pcaSafeInit();

  // Force first write on every channel
  for (uint8_t i = 0; i < MAX_SERVOS; ++i) currentAngle[i] = 255;

  // MPU6050
  mpu.initialize();
  mpuOk = mpu.testConnection();
  Serial.println(mpuOk ? F("✅ MPU6050 detected.") : F("❌ MPU6050 not found!"));
  Serial.println();
}

// ------------------------------------------------------------------
// 🔁  loop()
// ------------------------------------------------------------------
void loop() {
  processSerialCommand();

  if (systemActivated && (millis() - lastSensorTx >= SENSOR_PERIOD)) {
    lastSensorTx = millis();
    sendSensorData();
  }
}
