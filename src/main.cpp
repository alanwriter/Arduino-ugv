#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <util/atomic.h>
#include <avr/wdt.h>

// Follower 2 closed-loop differential-drive calibration profile.
//
// Safety rule: setup explicitly turns all L298N outputs off. A motor moves
// only after an explicit serial manual command (F/B/M...) or preset-path
// command (G1/G2/G3). External ENA/ENB pull-downs are still required to make
// reset/brownout safe before setup executes. See the demo wiring document.

// ---------------------------------------------------------------------------
// Temporary Follower 1 reference wiring. Confirm every Follower 2 connection
// before uploading this profile, then replace values here if its wiring differs.
// ---------------------------------------------------------------------------
constexpr uint8_t LEFT_ENCODER_A_PIN = 2;   // INT0 / PD2
constexpr uint8_t LEFT_ENCODER_B_PIN = 8;   // PCINT0 / PB0
constexpr uint8_t RIGHT_ENCODER_A_PIN = 7;  // PCINT23 / PD7
constexpr uint8_t RIGHT_ENCODER_B_PIN = 12; // PCINT4 / PB4

constexpr uint8_t LEFT_IN1_PIN = 3;
constexpr uint8_t LEFT_IN2_PIN = 4;
constexpr uint8_t RIGHT_IN1_PIN = 5;
constexpr uint8_t RIGHT_IN2_PIN = 6;
constexpr uint8_t LEFT_PWM_PIN = 9;   // L298N ENA; remove its jumper.
constexpr uint8_t RIGHT_PWM_PIN = 10; // L298N ENB; remove its jumper.

// MPU6050 is connected directly to the drive Nano's hardware I2C pins.
constexpr uint8_t MPU6050_SDA_PIN = A4;
constexpr uint8_t MPU6050_SCL_PIN = A5;

// ---------------------------------------------------------------------------
// Follower 2 has enough measured data for a cautious first straight G1 test.
// Its geometry is still approximate, so G2/G3 remain blocked until repeatable
// ground results have confirmed the wheel diameter and track width. Tick counts
// use this program's 4x AB-quadrature decoder.
// ---------------------------------------------------------------------------
constexpr float PI_F = 3.14159265358979323846f;
constexpr bool FOLLOWER2_G1_GROUND_TEST_ENABLED = true;
constexpr bool FOLLOWER2_FULL_PATH_CALIBRATION_APPROVED = false;

// follower2, 2026-09-06: exact 10 marked forward wheel revolutions yielded
// 12,162 left and 12,374 right decoded counts. These are 4x AB-quadrature
// counts, not an encoder-data-sheet pulse count.
constexpr float LEFT_TICKS_PER_WHEEL_REVOLUTION = 1216.2f;
constexpr float RIGHT_TICKS_PER_WHEEL_REVOLUTION = 1237.4f;

// Provisional Follower 2 geometry from the first physical measurement. Refine
// from repeated G1 ground tests before enabling paths containing turns.
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float WHEEL_TRACK_MM = 128.0f;

constexpr float LEFT_TICKS_PER_MM =
    LEFT_TICKS_PER_WHEEL_REVOLUTION / (PI_F * WHEEL_DIAMETER_MM);
constexpr float RIGHT_TICKS_PER_MM =
    RIGHT_TICKS_PER_WHEEL_REVOLUTION / (PI_F * WHEEL_DIAMETER_MM);

// Follower 2 starts with no assumed motor/encoder sign. Establish each value
// from its raised-wheel F test, then update this profile.
constexpr bool LEFT_MOTOR_REVERSED = false;
constexpr bool RIGHT_MOTOR_REVERSED = false;

constexpr bool LEFT_ENCODER_REVERSED = false;
// Follower 2: physical F motion is forward on both wheels; left raw count is
// positive while right raw count is negative, so reverse the right decoder.
constexpr bool RIGHT_ENCODER_REVERSED = true;

// Make a logical left turn increase pose heading. Test with M-80,80.
constexpr bool GYRO_Z_REVERSED = false;

// ---------------------------------------------------------------------------
// Conservative first-pass control tuning. These are intentionally low-speed
// demonstration values; tune them only after basic motor/encoder tests pass.
// ---------------------------------------------------------------------------
constexpr int MOTOR_MAX_PWM = 165;
constexpr int DEFAULT_MANUAL_PWM = 80;
constexpr unsigned long MANUAL_COMMAND_TIMEOUT_MS = 1200;

constexpr float MOTOR_STATIC_PWM = 55.0f;
constexpr float MOTOR_PWM_PER_TICKS_PER_SECOND = 0.14f;
constexpr float WHEEL_PID_KP = 0.10f;
constexpr float WHEEL_PID_KI = 0.08f;
constexpr float WHEEL_PID_INTEGRAL_LIMIT = 220.0f;
constexpr float MAX_WHEEL_TICKS_PER_SECOND = 650.0f;
constexpr float MAX_TARGET_ACCELERATION_TICKS_PER_SECOND_SQUARED = 500.0f;

constexpr float DRIVE_CRUISE_MM_PER_SECOND = 75.0f;
constexpr float DRIVE_MIN_MM_PER_SECOND = 20.0f;
constexpr float DRIVE_SLOWDOWN_GAIN = 1.20f;
constexpr float DRIVE_TOLERANCE_MM = 8.0f;
constexpr float HEADING_CORRECTION_MM_PER_SECOND_PER_RADIAN = 60.0f;
constexpr float MAX_HEADING_CORRECTION_MM_PER_SECOND = 35.0f;

constexpr float TURN_CRUISE_WHEEL_MM_PER_SECOND = 50.0f;
constexpr float TURN_MIN_WHEEL_MM_PER_SECOND = 15.0f;
constexpr float TURN_SLOWDOWN_GAIN = 80.0f;
constexpr float TURN_TOLERANCE_RAD = 2.5f * PI_F / 180.0f;
constexpr float TURN_FINAL_TOLERANCE_RAD = 1.0f * PI_F / 180.0f;
constexpr float TURN_FINAL_WHEEL_MM_PER_SECOND = 12.0f;

constexpr unsigned long CONTROL_INTERVAL_US = 10000UL; // 100 Hz wheel loop.
constexpr unsigned long IMU_INTERVAL_US = 10000UL;     // 100 Hz gyro read.
constexpr unsigned long REPORT_INTERVAL_MS = 500UL;
constexpr unsigned long ATTITUDE_REPORT_INTERVAL_MS = 200UL; // 5 Hz Serial view.
constexpr unsigned long COMMAND_IDLE_MS = 80UL;
constexpr unsigned long PATH_SETTLE_MS = 180UL;
constexpr unsigned long PATH_TURN_CORRECTION_TIMEOUT_MS = 1000UL;
constexpr unsigned long DRIVE_STEP_TIMEOUT_MS = 15000UL;
constexpr unsigned long TURN_STEP_TIMEOUT_MS = 7000UL;
constexpr unsigned long STALL_TIMEOUT_MS = 1300UL;
constexpr float DRIVE_PROGRESS_FOR_STALL_MM = 3.0f;
constexpr float TURN_PROGRESS_FOR_STALL_RAD = 2.0f * PI_F / 180.0f;
constexpr long WHEEL_PROGRESS_FOR_STALL_TICKS = 8;
constexpr unsigned long WHEEL_ENCODER_TIMEOUT_MS = 1300UL;
constexpr unsigned long PREFLIGHT_MIN_VALID_EDGES = 40UL;
constexpr unsigned long PREFLIGHT_MIN_PHASE_EDGES = 20UL;
// A healthy quadrature encoder produces nearly equal A/B edge totals. This
// deliberately leaves generous tolerance for short manual tests while
// rejecting a disconnected phase that otherwise makes count cancel near zero.
constexpr float PREFLIGHT_MIN_PHASE_EDGE_RATIO = 0.75f;
constexpr unsigned int PREFLIGHT_MAX_INVALID_TRANSITIONS = 10U;

constexpr float IMU_GYRO_BLEND = 0.70f;
constexpr float GYRO_SCALE_LSB_PER_DEGREE_PER_SECOND = 131.0f;
constexpr float ACCEL_SCALE_LSB_PER_G = 16384.0f;
constexpr float GYRO_DEADBAND_DEGREES_PER_SECOND = 0.20f;
constexpr uint16_t GYRO_CALIBRATION_SAMPLES = 300;
constexpr uint32_t WIRE_TIMEOUT_US = 25000UL;

// ---------------------------------------------------------------------------
// Utility helpers.
// ---------------------------------------------------------------------------
float clampFloat(float value, float minimum, float maximum) {
  if (value < minimum) {
    return minimum;
  }
  if (value > maximum) {
    return maximum;
  }
  return value;
}

float absoluteFloat(float value) {
  return value < 0.0f ? -value : value;
}

float wrapAngleRadians(float angle) {
  while (angle > PI_F) {
    angle -= 2.0f * PI_F;
  }
  while (angle < -PI_F) {
    angle += 2.0f * PI_F;
  }
  return angle;
}

// ---------------------------------------------------------------------------
// Full 4x AB quadrature decoding for both wheel encoders.
// ---------------------------------------------------------------------------
struct EncoderState {
  volatile long count;
  volatile unsigned long validEdges;
  volatile unsigned long aEdges;
  volatile unsigned long bEdges;
  volatile unsigned int invalidTransitions;
  volatile uint8_t previousState;
};

struct EncoderSnapshot {
  long count;
  unsigned long validEdges;
  unsigned long aEdges;
  unsigned long bEdges;
  unsigned int invalidTransitions;
  uint8_t state;
};

volatile EncoderState leftEncoder;
volatile EncoderState rightEncoder;
volatile uint8_t previousEncoderPortB = 0;

const int8_t QUADRATURE_DELTA[16] = {
     0,  1, -1,  0,
    -1,  0,  0,  1,
     1,  0,  0, -1,
     0, -1,  1,  0,
};

inline uint8_t readLeftEncoderState() {
  const uint8_t a = (PIND & _BV(PD2)) ? 1 : 0;
  const uint8_t b = (PINB & _BV(PB0)) ? 1 : 0;
  return static_cast<uint8_t>((a << 1) | b);
}

inline uint8_t readRightEncoderState() {
  const uint8_t a = (PIND & _BV(PD7)) ? 1 : 0;
  const uint8_t b = (PINB & _BV(PB4)) ? 1 : 0;
  return static_cast<uint8_t>((a << 1) | b);
}

inline void processEncoderTransition(volatile EncoderState &encoder,
                                     uint8_t currentState,
                                     bool reverseDirection) {
  const uint8_t previousState = encoder.previousState;
  if (currentState == previousState) {
    return;
  }

  const uint8_t changedBits = currentState ^ previousState;
  if ((changedBits & 0x02) != 0) {
    ++encoder.aEdges;
  }
  if ((changedBits & 0x01) != 0) {
    ++encoder.bEdges;
  }

  const uint8_t transition =
      static_cast<uint8_t>((previousState << 2) | currentState);
  int8_t delta = QUADRATURE_DELTA[transition];
  if (reverseDirection) {
    delta = -delta;
  }

  if (delta != 0) {
    encoder.count += delta;
    ++encoder.validEdges;
  } else {
    // Both phases changed together, or an edge was missed.
    ++encoder.invalidTransitions;
  }

  encoder.previousState = currentState;
}

void onLeftEncoderAChange() {
  processEncoderTransition(leftEncoder, readLeftEncoderState(),
                           LEFT_ENCODER_REVERSED);
}

// D8 (left B) and D12 (right B) share the PCINT0 interrupt vector.
// Only call a decoder for the B pin that actually changed.
ISR(PCINT0_vect) {
  const uint8_t currentPortB = PINB;
  const uint8_t watchedMask = _BV(PB0) | _BV(PB4);
  const uint8_t changedPins =
      static_cast<uint8_t>((currentPortB ^ previousEncoderPortB) & watchedMask);
  previousEncoderPortB = currentPortB;

  if ((changedPins & _BV(PB0)) != 0) {
    processEncoderTransition(leftEncoder, readLeftEncoderState(),
                             LEFT_ENCODER_REVERSED);
  }
  if ((changedPins & _BV(PB4)) != 0) {
    processEncoderTransition(rightEncoder, readRightEncoderState(),
                             RIGHT_ENCODER_REVERSED);
  }
}

// D7 (right A) is PCINT23, on the PCINT2 vector.
ISR(PCINT2_vect) {
  processEncoderTransition(rightEncoder, readRightEncoderState(),
                           RIGHT_ENCODER_REVERSED);
}

void resetEncoderCounters() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    leftEncoder.count = 0;
    leftEncoder.validEdges = 0;
    leftEncoder.aEdges = 0;
    leftEncoder.bEdges = 0;
    leftEncoder.invalidTransitions = 0;
    leftEncoder.previousState = readLeftEncoderState();

    rightEncoder.count = 0;
    rightEncoder.validEdges = 0;
    rightEncoder.aEdges = 0;
    rightEncoder.bEdges = 0;
    rightEncoder.invalidTransitions = 0;
    rightEncoder.previousState = readRightEncoderState();

    previousEncoderPortB = PINB;
  }
}

void readEncoderSnapshots(EncoderSnapshot &left, EncoderSnapshot &right) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    left.count = leftEncoder.count;
    left.validEdges = leftEncoder.validEdges;
    left.aEdges = leftEncoder.aEdges;
    left.bEdges = leftEncoder.bEdges;
    left.invalidTransitions = leftEncoder.invalidTransitions;
    left.state = leftEncoder.previousState;

    right.count = rightEncoder.count;
    right.validEdges = rightEncoder.validEdges;
    right.aEdges = rightEncoder.aEdges;
    right.bEdges = rightEncoder.bEdges;
    right.invalidTransitions = rightEncoder.invalidTransitions;
    right.state = rightEncoder.previousState;
  }
}

void setupEncoders() {
  pinMode(LEFT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENCODER_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_B_PIN, INPUT_PULLUP);

  noInterrupts();
  // This firmware owns only these pin-change masks on Nano #1.
  PCICR &= static_cast<uint8_t>(~(_BV(PCIE0) | _BV(PCIE2)));
  EIMSK &= static_cast<uint8_t>(~_BV(INT0));

  leftEncoder.previousState = readLeftEncoderState();
  rightEncoder.previousState = readRightEncoderState();
  previousEncoderPortB = PINB;

  PCMSK0 = _BV(PB0) | _BV(PB4); // D8 and D12.
  PCMSK2 = _BV(PD7);             // D7 / PCINT23.
  PCIFR = _BV(PCIF0) | _BV(PCIF2);     // Clear pending PCINTs.
  EIFR = _BV(INTF0);                   // Clear pending INT0.

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_A_PIN),
                  onLeftEncoderAChange, CHANGE);
  PCICR |= _BV(PCIE0) | _BV(PCIE2);
  interrupts();
}

// ---------------------------------------------------------------------------
// L298N motor output. Signed commands are in the logical vehicle frame.
// ---------------------------------------------------------------------------
int appliedLeftPwm = 0;
int appliedRightPwm = 0;

void writeOneMotor(uint8_t in1Pin, uint8_t in2Pin, uint8_t pwmPin,
                   int logicalPwm, bool reverseMotor) {
  int command = constrain(logicalPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  if (reverseMotor) {
    command = -command;
  }

  if (command > 0) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else if (command < 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  } else {
    // Coast stop: no L298N bridge output is enabled.
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  }
  analogWrite(pwmPin, abs(command));
}

void setMotorOutputs(int leftPwm, int rightPwm) {
  appliedLeftPwm = constrain(leftPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  appliedRightPwm = constrain(rightPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  writeOneMotor(LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN, appliedLeftPwm,
                LEFT_MOTOR_REVERSED);
  writeOneMotor(RIGHT_IN1_PIN, RIGHT_IN2_PIN, RIGHT_PWM_PIN, appliedRightPwm,
                RIGHT_MOTOR_REVERSED);
}

void configureMotorPinsAndStop() {
  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_IN1_PIN, OUTPUT);
  pinMode(RIGHT_IN2_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  setMotorOutputs(0, 0);
}

// ---------------------------------------------------------------------------
// MPU6050: minimal motion driver. No external Arduino library is required.
// ---------------------------------------------------------------------------
constexpr uint8_t MPU6050_REG_WHO_AM_I = 0x75;
constexpr uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
constexpr uint8_t MPU6050_REG_CONFIG = 0x1A;
constexpr uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
constexpr uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

bool imuPresent = false;
bool imuCalibrated = false;
uint8_t imuAddress = 0x68;
float gyroZBiasRaw = 0.0f;
float gyroXDegreesPerSecondRaw = 0.0f;
float gyroYDegreesPerSecondRaw = 0.0f;
float gyroZDegreesPerSecond = 0.0f;
float attitudeYawRelativeDegrees = 0.0f;
float accelerometerXG = 0.0f;
float accelerometerYG = 0.0f;
float accelerometerZG = 0.0f;
float accelerometerRollDegrees = 0.0f;
float accelerometerPitchDegrees = 0.0f;
float gyroDeltaSinceOdometry = 0.0f;
unsigned int imuReadFailures = 0;
unsigned int consecutiveImuReadFailures = 0;
unsigned long lastImuUpdateUs = 0;
bool imuBusTimeoutOccurred = false;
bool attitudeMonitorEnabled = false;
unsigned long lastAttitudeReportMs = 0;

bool consumeWireTimeoutFlag() {
  if (!Wire.getWireTimeoutFlag()) {
    return false;
  }
  Wire.clearWireTimeoutFlag();
  imuBusTimeoutOccurred = true;
  return true;
}

bool writeMpuRegister(uint8_t reg, uint8_t value) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  Wire.write(value);
  const uint8_t transmissionStatus = Wire.endTransmission();
  return !consumeWireTimeoutFlag() && transmissionStatus == 0;
}

bool readMpuRegisters(uint8_t reg, uint8_t *buffer, uint8_t length) {
  Wire.clearWireTimeoutFlag();
  Wire.beginTransmission(imuAddress);
  Wire.write(reg);
  const uint8_t transmissionStatus = Wire.endTransmission(false);
  if (consumeWireTimeoutFlag() || transmissionStatus != 0) {
    return false;
  }

  const uint8_t received = Wire.requestFrom(imuAddress, length);
  if (consumeWireTimeoutFlag() || received != length) {
    while (Wire.available() > 0) {
      Wire.read();
    }
    return false;
  }

  for (uint8_t index = 0; index < length; ++index) {
    if (Wire.available() <= 0) {
      return false;
    }
    buffer[index] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool readMpuMotionRaw(int16_t &rawAccelX, int16_t &rawAccelY,
                      int16_t &rawAccelZ, int16_t &rawGyroX,
                      int16_t &rawGyroY, int16_t &rawGyroZ) {
  uint8_t bytes[14];
  if (!readMpuRegisters(MPU6050_REG_ACCEL_XOUT_H, bytes, sizeof(bytes))) {
    return false;
  }
  rawAccelX = static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                   static_cast<uint16_t>(bytes[1]));
  rawAccelY = static_cast<int16_t>((static_cast<uint16_t>(bytes[2]) << 8) |
                                   static_cast<uint16_t>(bytes[3]));
  rawAccelZ = static_cast<int16_t>((static_cast<uint16_t>(bytes[4]) << 8) |
                                   static_cast<uint16_t>(bytes[5]));
  rawGyroX = static_cast<int16_t>((static_cast<uint16_t>(bytes[8]) << 8) |
                                  static_cast<uint16_t>(bytes[9]));
  rawGyroY = static_cast<int16_t>((static_cast<uint16_t>(bytes[10]) << 8) |
                                  static_cast<uint16_t>(bytes[11]));
  rawGyroZ = static_cast<int16_t>((static_cast<uint16_t>(bytes[12]) << 8) |
                                  static_cast<uint16_t>(bytes[13]));
  return true;
}

bool initialiseMpu6050() {
  imuBusTimeoutOccurred = false;
  const uint8_t possibleAddresses[] = {0x68, 0x69};
  for (uint8_t index = 0; index < sizeof(possibleAddresses); ++index) {
    imuAddress = possibleAddresses[index];
    uint8_t whoAmI = 0;
    if (!readMpuRegisters(MPU6050_REG_WHO_AM_I, &whoAmI, 1)) {
      continue;
    }
    if (whoAmI != 0x68 && whoAmI != 0x69) {
      continue;
    }

    if (!writeMpuRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
      continue;
    }
    delay(100);
    if (!writeMpuRegister(MPU6050_REG_CONFIG, 0x03) ||
        !writeMpuRegister(MPU6050_REG_GYRO_CONFIG, 0x00)) {
      continue;
    }

    imuPresent = true;
    imuCalibrated = false;
    imuReadFailures = 0;
    consecutiveImuReadFailures = 0;
    lastImuUpdateUs = micros();
    return true;
  }

  imuPresent = false;
  imuCalibrated = false;
  return false;
}

bool calibrateGyroZ() {
  if (!imuPresent) {
    Serial.println(F("MPU6050 not detected; cannot calibrate."));
    return false;
  }

  Serial.println(F("Keep vehicle completely still: calibrating gyro Z..."));
  long sum = 0;
  uint16_t validSamples = 0;
  for (uint16_t index = 0; index < GYRO_CALIBRATION_SAMPLES; ++index) {
    int16_t rawAccelX = 0;
    int16_t rawAccelY = 0;
    int16_t rawAccelZ = 0;
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;
    if (readMpuMotionRaw(rawAccelX, rawAccelY, rawAccelZ, rawX, rawY,
                         rawZ)) {
      sum += rawZ;
      ++validSamples;
    }
    // Calibration runs only while motors are stopped, but it is deliberately
    // longer than the normal watchdog interval.
    wdt_reset();
    delay(5);
  }

  if (validSamples < (GYRO_CALIBRATION_SAMPLES * 9U) / 10U) {
    imuCalibrated = false;
    ++imuReadFailures;
    Serial.println(F("Gyro calibration failed: too many I2C read errors."));
    return false;
  }

  gyroZBiasRaw = static_cast<float>(sum) / validSamples;
  gyroZDegreesPerSecond = 0.0f;
  attitudeYawRelativeDegrees = 0.0f;
  gyroDeltaSinceOdometry = 0.0f;
  imuCalibrated = true;
  consecutiveImuReadFailures = 0;
  imuBusTimeoutOccurred = false;
  lastImuUpdateUs = micros();

  Serial.print(F("Gyro calibrated. Z raw bias="));
  Serial.println(gyroZBiasRaw, 2);
  return true;
}

void updateImu() {
  if (!imuPresent) {
    return;
  }

  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastImuUpdateUs;
  if (elapsedUs < IMU_INTERVAL_US) {
    return;
  }
  lastImuUpdateUs = nowUs;

  int16_t rawAccelX = 0;
  int16_t rawAccelY = 0;
  int16_t rawAccelZ = 0;
  int16_t rawX = 0;
  int16_t rawY = 0;
  int16_t rawZ = 0;
  if (!readMpuMotionRaw(rawAccelX, rawAccelY, rawAccelZ, rawX, rawY,
                        rawZ)) {
    ++imuReadFailures;
    ++consecutiveImuReadFailures;
    return;
  }
  consecutiveImuReadFailures = 0;

  accelerometerXG = static_cast<float>(rawAccelX) / ACCEL_SCALE_LSB_PER_G;
  accelerometerYG = static_cast<float>(rawAccelY) / ACCEL_SCALE_LSB_PER_G;
  accelerometerZG = static_cast<float>(rawAccelZ) / ACCEL_SCALE_LSB_PER_G;
  accelerometerRollDegrees =
      atan2f(accelerometerYG, accelerometerZG) * 180.0f / PI_F;
  accelerometerPitchDegrees =
      atan2f(-accelerometerXG,
             sqrtf(accelerometerYG * accelerometerYG +
                   accelerometerZG * accelerometerZG)) *
      180.0f / PI_F;

  gyroXDegreesPerSecondRaw =
      static_cast<float>(rawX) / GYRO_SCALE_LSB_PER_DEGREE_PER_SECOND;
  gyroYDegreesPerSecondRaw =
      static_cast<float>(rawY) / GYRO_SCALE_LSB_PER_DEGREE_PER_SECOND;
  float rate = (static_cast<float>(rawZ) - gyroZBiasRaw) /
               GYRO_SCALE_LSB_PER_DEGREE_PER_SECOND;
  if (GYRO_Z_REVERSED) {
    rate = -rate;
  }
  if (absoluteFloat(rate) < GYRO_DEADBAND_DEGREES_PER_SECOND) {
    rate = 0.0f;
  }
  gyroZDegreesPerSecond = rate;

  if (imuCalibrated) {
    const float boundedDt =
        clampFloat(static_cast<float>(elapsedUs) / 1000000.0f, 0.0f, 0.05f);
    gyroDeltaSinceOdometry += rate * PI_F / 180.0f * boundedDt;
    attitudeYawRelativeDegrees += rate * boundedDt;
  }
}

// ---------------------------------------------------------------------------
// Pose, velocity PID and high-level motion state.
// ---------------------------------------------------------------------------
struct Pose {
  float xMm;
  float yMm;
  float headingRad;
};

struct WheelController {
  long lastCount;
  float measuredTicksPerSecond;
  float requestedTicksPerSecond;
  float targetTicksPerSecond;
  float integral;
  int commandPwm;
};

enum MotionMode : uint8_t {
  MOTION_IDLE,
  MOTION_MANUAL,
  MOTION_PATH,
  MOTION_FAULT,
};

enum FaultCode : uint8_t {
  FAULT_NONE,
  FAULT_DRIVE_TIMEOUT,
  FAULT_TURN_TIMEOUT,
  FAULT_DRIVE_STALL,
  FAULT_TURN_STALL,
  FAULT_LEFT_ENCODER_STALL,
  FAULT_RIGHT_ENCODER_STALL,
  FAULT_IMU_READ,
  FAULT_IMU_BUS_TIMEOUT,
};

Pose pose = {0.0f, 0.0f, 0.0f};
WheelController leftController = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0};
WheelController rightController = {0, 0.0f, 0.0f, 0.0f, 0.0f, 0};
long odometryLastLeftCount = 0;
long odometryLastRightCount = 0;
MotionMode motionMode = MOTION_IDLE;
FaultCode faultCode = FAULT_NONE;
int manualLeftPwm = 0;
int manualRightPwm = 0;
unsigned long manualExpiresMs = 0;

void resetWheelController(WheelController &controller, long count) {
  controller.lastCount = count;
  controller.measuredTicksPerSecond = 0.0f;
  controller.requestedTicksPerSecond = 0.0f;
  controller.targetTicksPerSecond = 0.0f;
  controller.integral = 0.0f;
  controller.commandPwm = 0;
}

void resetWheelControllersFromSnapshot(const EncoderSnapshot &left,
                                       const EncoderSnapshot &right) {
  resetWheelController(leftController, left.count);
  resetWheelController(rightController, right.count);
}

void updateWheelMeasurement(WheelController &controller, long count,
                            float dtSeconds) {
  const long deltaTicks = count - controller.lastCount;
  controller.lastCount = count;
  controller.measuredTicksPerSecond =
      static_cast<float>(deltaTicks) / dtSeconds;
}

int calculateWheelPwm(WheelController &controller, float dtSeconds) {
  const float target = controller.targetTicksPerSecond;
  if (absoluteFloat(target) < 0.5f) {
    controller.integral = 0.0f;
    controller.commandPwm = 0;
    return 0;
  }

  const float error = target - controller.measuredTicksPerSecond;
  controller.integral = clampFloat(controller.integral + error * dtSeconds,
                                   -WHEEL_PID_INTEGRAL_LIMIT,
                                   WHEEL_PID_INTEGRAL_LIMIT);

  const float direction = target >= 0.0f ? 1.0f : -1.0f;
  float command = direction *
                      (MOTOR_STATIC_PWM +
                       MOTOR_PWM_PER_TICKS_PER_SECOND * absoluteFloat(target)) +
                  WHEEL_PID_KP * error + WHEEL_PID_KI * controller.integral;

  // Do not reverse a wheel merely because its velocity overshot its target.
  if (command * direction < 0.0f) {
    command = 0.0f;
  }
  command = clampFloat(command, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  controller.commandPwm =
      static_cast<int>(command + (command >= 0.0f ? 0.5f : -0.5f));
  return controller.commandPwm;
}

void setWheelTargetsFromMmPerSecond(float leftMmPerSecond,
                                    float rightMmPerSecond) {
  leftController.requestedTicksPerSecond = clampFloat(
      leftMmPerSecond * LEFT_TICKS_PER_MM, -MAX_WHEEL_TICKS_PER_SECOND,
      MAX_WHEEL_TICKS_PER_SECOND);
  rightController.requestedTicksPerSecond = clampFloat(
      rightMmPerSecond * RIGHT_TICKS_PER_MM, -MAX_WHEEL_TICKS_PER_SECOND,
      MAX_WHEEL_TICKS_PER_SECOND);
}

void immediatelyStopWheelControllers() {
  leftController.requestedTicksPerSecond = 0.0f;
  leftController.targetTicksPerSecond = 0.0f;
  leftController.integral = 0.0f;
  leftController.commandPwm = 0;
  rightController.requestedTicksPerSecond = 0.0f;
  rightController.targetTicksPerSecond = 0.0f;
  rightController.integral = 0.0f;
  rightController.commandPwm = 0;
}

void slewWheelTarget(WheelController &controller, float dtSeconds) {
  const float requested = controller.requestedTicksPerSecond;
  const float active = controller.targetTicksPerSecond;

  // Deceleration is immediate; only acceleration is rate-limited. If a
  // correction reverses direction, first slew to zero before accelerating
  // in the other direction.
  if (active * requested < 0.0f) {
    const float step =
        MAX_TARGET_ACCELERATION_TICKS_PER_SECOND_SQUARED * dtSeconds;
    if (absoluteFloat(active) <= step) {
      controller.targetTicksPerSecond = 0.0f;
    } else {
      controller.targetTicksPerSecond =
          active > 0.0f ? active - step : active + step;
    }
    return;
  }
  if (absoluteFloat(requested) <= absoluteFloat(active)) {
    controller.targetTicksPerSecond = requested;
    return;
  }

  const float step =
      MAX_TARGET_ACCELERATION_TICKS_PER_SECOND_SQUARED * dtSeconds;
  const float difference = requested - active;
  if (absoluteFloat(difference) <= step) {
    controller.targetTicksPerSecond = requested;
  } else {
    controller.targetTicksPerSecond =
        active + (difference > 0.0f ? step : -step);
  }
}

void updateOdometry(long leftCount, long rightCount) {
  const long leftDeltaTicks = leftCount - odometryLastLeftCount;
  const long rightDeltaTicks = rightCount - odometryLastRightCount;
  odometryLastLeftCount = leftCount;
  odometryLastRightCount = rightCount;

  const float leftDeltaMm =
      static_cast<float>(leftDeltaTicks) / LEFT_TICKS_PER_MM;
  const float rightDeltaMm =
      static_cast<float>(rightDeltaTicks) / RIGHT_TICKS_PER_MM;
  const float forwardDeltaMm = (leftDeltaMm + rightDeltaMm) * 0.5f;
  const float encoderHeadingDelta =
      (rightDeltaMm - leftDeltaMm) / WHEEL_TRACK_MM;

  float headingDelta = encoderHeadingDelta;
  if (imuCalibrated) {
    headingDelta = (1.0f - IMU_GYRO_BLEND) * encoderHeadingDelta +
                   IMU_GYRO_BLEND * gyroDeltaSinceOdometry;
  }
  gyroDeltaSinceOdometry = 0.0f;

  const float midpointHeading = pose.headingRad + headingDelta * 0.5f;
  pose.xMm += forwardDeltaMm * cosf(midpointHeading);
  pose.yMm += forwardDeltaMm * sinf(midpointHeading);
  pose.headingRad += headingDelta;
}

void resetPoseAndOdometry(const EncoderSnapshot &left,
                          const EncoderSnapshot &right) {
  pose.xMm = 0.0f;
  pose.yMm = 0.0f;
  pose.headingRad = 0.0f;
  odometryLastLeftCount = left.count;
  odometryLastRightCount = right.count;
  gyroDeltaSinceOdometry = 0.0f;
}

// ---------------------------------------------------------------------------
// Preset paths. Positive turns are logical left / counter-clockwise turns.
// ---------------------------------------------------------------------------
enum PathStepKind : uint8_t {
  PATH_DRIVE_MM,
  PATH_TURN_DEGREES,
};

struct PathStep {
  PathStepKind kind;
  int16_t value;
};

constexpr int16_t DEMO_STRAIGHT_MM = 500;
constexpr int16_t DEMO_SQUARE_SIDE_MM = 400;
constexpr int16_t DEMO_L_LEG_MM = 350;

const PathStep PATH_STRAIGHT[] = {
    {PATH_DRIVE_MM, DEMO_STRAIGHT_MM},
};

const PathStep PATH_SQUARE[] = {
    {PATH_DRIVE_MM, DEMO_SQUARE_SIDE_MM}, {PATH_TURN_DEGREES, 90},
    {PATH_DRIVE_MM, DEMO_SQUARE_SIDE_MM}, {PATH_TURN_DEGREES, 90},
    {PATH_DRIVE_MM, DEMO_SQUARE_SIDE_MM}, {PATH_TURN_DEGREES, 90},
    {PATH_DRIVE_MM, DEMO_SQUARE_SIDE_MM}, {PATH_TURN_DEGREES, 90},
};

const PathStep PATH_L_SHAPE[] = {
    {PATH_DRIVE_MM, DEMO_L_LEG_MM}, {PATH_TURN_DEGREES, 90},
    {PATH_DRIVE_MM, DEMO_L_LEG_MM},
};

const PathStep *activePath = 0;
uint8_t activePathLength = 0;
uint8_t activePathNumber = 0;
uint8_t pathStepIndex = 0;
long pathStartLeftCount = 0;
long pathStartRightCount = 0;
float pathTargetDistanceMm = 0.0f;
float pathDriveDirection = 1.0f;
float pathTargetHeadingRad = 0.0f;
float pathLastProgressMetric = 0.0f;
bool pathSettling = false;
bool pathSettlingForTurn = false;
unsigned long pathStepStartedMs = 0;
unsigned long pathLastProgressMs = 0;
unsigned long pathSettleStartedMs = 0;
long pathLastLeftEncoderProgressCount = 0;
long pathLastRightEncoderProgressCount = 0;
unsigned long pathLastLeftEncoderProgressMs = 0;
unsigned long pathLastRightEncoderProgressMs = 0;
bool encoderPreflightPassed = false;

#ifdef FOLLOWER2_AUTORUN_G1
// Follower 2 demo is a separate build target. It waits for the vehicle to be
// still, calibrates gyro Z, runs the approved first G1 test once, and then
// remains stopped until the next power cycle.
constexpr unsigned long FOLLOWER2_AUTORUN_STILLNESS_DELAY_MS = 3000UL;

enum Follower2AutoRunState : uint8_t {
  FOLLOWER2_WAITING_FOR_STILLNESS,
  FOLLOWER2_RUNNING_G1,
  FOLLOWER2_COMPLETE,
  FOLLOWER2_FAULT,
};

Follower2AutoRunState follower2AutoRunState = FOLLOWER2_WAITING_FOR_STILLNESS;
unsigned long follower2AutoRunStartedMs = 0;
#endif

void printFaultCode() {
  switch (faultCode) {
    case FAULT_NONE:
      Serial.print(F("none"));
      break;
    case FAULT_DRIVE_TIMEOUT:
      Serial.print(F("drive timeout"));
      break;
    case FAULT_TURN_TIMEOUT:
      Serial.print(F("turn timeout"));
      break;
    case FAULT_DRIVE_STALL:
      Serial.print(F("drive stall"));
      break;
    case FAULT_TURN_STALL:
      Serial.print(F("turn stall"));
      break;
    case FAULT_LEFT_ENCODER_STALL:
      Serial.print(F("left encoder/wheel stall"));
      break;
    case FAULT_RIGHT_ENCODER_STALL:
      Serial.print(F("right encoder/wheel stall"));
      break;
    case FAULT_IMU_READ:
      Serial.print(F("MPU6050 read failure"));
      break;
    case FAULT_IMU_BUS_TIMEOUT:
      Serial.print(F("MPU6050 I2C timeout"));
      break;
  }
}

void stopMotion(bool clearFault) {
  motionMode = MOTION_IDLE;
  activePath = 0;
  activePathLength = 0;
  pathSettling = false;
  pathSettlingForTurn = false;
  manualLeftPwm = 0;
  manualRightPwm = 0;
  setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
  immediatelyStopWheelControllers();
  setMotorOutputs(0, 0);
  if (clearFault) {
    faultCode = FAULT_NONE;
  }
}

void enterFault(FaultCode code) {
  faultCode = code;
  motionMode = MOTION_FAULT;
  activePath = 0;
  activePathLength = 0;
  pathSettling = false;
  pathSettlingForTurn = false;
  setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
  immediatelyStopWheelControllers();
  setMotorOutputs(0, 0);
  Serial.print(F("FAULT: "));
  printFaultCode();
  Serial.println(F(". Motors stopped."));
}

void completePath() {
  motionMode = MOTION_IDLE;
  activePath = 0;
  activePathLength = 0;
  pathSettling = false;
  pathSettlingForTurn = false;
  setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
  immediatelyStopWheelControllers();
  setMotorOutputs(0, 0);
  Serial.println(F("Preset path complete. Motors stopped."));
}

bool encodersPassPreflight(const EncoderSnapshot &left,
                           const EncoderSnapshot &right) {
  const unsigned long leftLowerPhaseEdges =
      left.aEdges < left.bEdges ? left.aEdges : left.bEdges;
  const unsigned long leftHigherPhaseEdges =
      left.aEdges < left.bEdges ? left.bEdges : left.aEdges;
  const unsigned long rightLowerPhaseEdges =
      right.aEdges < right.bEdges ? right.aEdges : right.bEdges;
  const unsigned long rightHigherPhaseEdges =
      right.aEdges < right.bEdges ? right.bEdges : right.aEdges;

  return left.validEdges >= PREFLIGHT_MIN_VALID_EDGES &&
         right.validEdges >= PREFLIGHT_MIN_VALID_EDGES &&
         left.aEdges >= PREFLIGHT_MIN_PHASE_EDGES &&
         left.bEdges >= PREFLIGHT_MIN_PHASE_EDGES &&
         right.aEdges >= PREFLIGHT_MIN_PHASE_EDGES &&
         right.bEdges >= PREFLIGHT_MIN_PHASE_EDGES &&
         static_cast<float>(leftLowerPhaseEdges) >=
             static_cast<float>(leftHigherPhaseEdges) *
                 PREFLIGHT_MIN_PHASE_EDGE_RATIO &&
         static_cast<float>(rightLowerPhaseEdges) >=
             static_cast<float>(rightHigherPhaseEdges) *
                 PREFLIGHT_MIN_PHASE_EDGE_RATIO &&
         left.invalidTransitions <= PREFLIGHT_MAX_INVALID_TRANSITIONS &&
         right.invalidTransitions <= PREFLIGHT_MAX_INVALID_TRANSITIONS;
}

void updateEncoderPreflight(const EncoderSnapshot &left,
                            const EncoderSnapshot &right) {
  if (encoderPreflightPassed || !encodersPassPreflight(left, right)) {
    return;
  }
  encoderPreflightPassed = true;
  Serial.println(
      F("Encoder preflight passed: both A/B phases have been observed."));
}

bool updatePathWheelWatchdog(unsigned long nowMs,
                             const EncoderSnapshot &left,
                             const EncoderSnapshot &right,
                             float expectedLeftDirection,
                             float expectedRightDirection) {
  if (expectedLeftDirection *
          static_cast<float>(left.count - pathLastLeftEncoderProgressCount) >=
      WHEEL_PROGRESS_FOR_STALL_TICKS) {
    pathLastLeftEncoderProgressCount = left.count;
    pathLastLeftEncoderProgressMs = nowMs;
  }
  if (expectedRightDirection *
          static_cast<float>(right.count - pathLastRightEncoderProgressCount) >=
      WHEEL_PROGRESS_FOR_STALL_TICKS) {
    pathLastRightEncoderProgressCount = right.count;
    pathLastRightEncoderProgressMs = nowMs;
  }

  if (nowMs - pathLastLeftEncoderProgressMs > WHEEL_ENCODER_TIMEOUT_MS) {
    enterFault(FAULT_LEFT_ENCODER_STALL);
    return false;
  }
  if (nowMs - pathLastRightEncoderProgressMs > WHEEL_ENCODER_TIMEOUT_MS) {
    enterFault(FAULT_RIGHT_ENCODER_STALL);
    return false;
  }
  return true;
}

void startPathStep();

void startPathSettle(bool isTurn) {
  pathSettling = true;
  pathSettlingForTurn = isTurn;
  pathSettleStartedMs = millis();
  setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
  immediatelyStopWheelControllers();
}

void startPathStep() {
  if (pathStepIndex >= activePathLength) {
    completePath();
    return;
  }

  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);
  resetWheelControllersFromSnapshot(left, right);

  pathStartLeftCount = left.count;
  pathStartRightCount = right.count;
  pathStepStartedMs = millis();
  pathLastProgressMs = pathStepStartedMs;
  pathLastLeftEncoderProgressCount = left.count;
  pathLastRightEncoderProgressCount = right.count;
  pathLastLeftEncoderProgressMs = pathStepStartedMs;
  pathLastRightEncoderProgressMs = pathStepStartedMs;
  pathSettling = false;
  pathSettlingForTurn = false;

  const PathStep &step = activePath[pathStepIndex];
  if (step.kind == PATH_DRIVE_MM) {
    pathTargetDistanceMm = absoluteFloat(static_cast<float>(step.value));
    pathDriveDirection = step.value >= 0 ? 1.0f : -1.0f;
    pathTargetHeadingRad = pose.headingRad;
    pathLastProgressMetric = 0.0f;
    Serial.print(F("Path step "));
    Serial.print(pathStepIndex + 1);
    Serial.print(F(": drive "));
    Serial.print(step.value);
    Serial.println(F(" mm"));
  } else {
    pathTargetHeadingRad =
        pose.headingRad + static_cast<float>(step.value) * PI_F / 180.0f;
    pathLastProgressMetric =
        absoluteFloat(pathTargetHeadingRad - pose.headingRad);
    Serial.print(F("Path step "));
    Serial.print(pathStepIndex + 1);
    Serial.print(F(": turn "));
    Serial.print(step.value);
    Serial.println(F(" deg"));
  }
}

bool selectPresetPath(uint8_t pathNumber) {
  switch (pathNumber) {
    case 1:
      activePath = PATH_STRAIGHT;
      activePathLength = sizeof(PATH_STRAIGHT) / sizeof(PATH_STRAIGHT[0]);
      break;
    case 2:
      activePath = PATH_SQUARE;
      activePathLength = sizeof(PATH_SQUARE) / sizeof(PATH_SQUARE[0]);
      break;
    case 3:
      activePath = PATH_L_SHAPE;
      activePathLength = sizeof(PATH_L_SHAPE) / sizeof(PATH_L_SHAPE[0]);
      break;
    default:
      return false;
  }
  activePathNumber = pathNumber;
  return true;
}

void startPresetPath(uint8_t pathNumber) {
  if (!FOLLOWER2_G1_GROUND_TEST_ENABLED) {
    Serial.println(F("Path blocked: Follower 2 needs G1 calibration approval first."));
    return;
  }
  if (pathNumber != 1 && !FOLLOWER2_FULL_PATH_CALIBRATION_APPROVED) {
    Serial.println(F("Path blocked: Follower 2 provisional geometry permits G1 only."));
    return;
  }
  if (!imuPresent || !imuCalibrated) {
    Serial.println(F("Path blocked: connect MPU6050 and send C while still first."));
    return;
  }
  if (!encoderPreflightPassed) {
    Serial.println(
        F("Path blocked: first verify both encoder A/B phases with D and F."));
    return;
  }
  if (!selectPresetPath(pathNumber)) {
    Serial.println(F("Unknown path. Use G1, G2, or G3."));
    return;
  }

  faultCode = FAULT_NONE;
  motionMode = MOTION_PATH;
  pathStepIndex = 0;
  pathSettling = false;
  startPathStep();
}

void updatePathControl() {
  if (motionMode != MOTION_PATH || activePath == 0) {
    return;
  }

  const unsigned long nowMs = millis();
  if (pathSettling) {
    if (pathSettlingForTurn) {
      const float headingError = pathTargetHeadingRad - pose.headingRad;
      const float remainingRadians = absoluteFloat(headingError);
      const unsigned long settleElapsedMs = nowMs - pathSettleStartedMs;
      if (remainingRadians > TURN_FINAL_TOLERANCE_RAD &&
          settleElapsedMs < PATH_TURN_CORRECTION_TIMEOUT_MS) {
        const float correctionDirection =
            headingError >= 0.0f ? 1.0f : -1.0f;
        setWheelTargetsFromMmPerSecond(
            -correctionDirection * TURN_FINAL_WHEEL_MM_PER_SECOND,
            correctionDirection * TURN_FINAL_WHEEL_MM_PER_SECOND);
        return;
      }
    }
    setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
    immediatelyStopWheelControllers();
    if (nowMs - pathSettleStartedMs >= PATH_SETTLE_MS) {
      ++pathStepIndex;
      startPathStep();
    }
    return;
  }

  const PathStep &step = activePath[pathStepIndex];
  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);

  if (step.kind == PATH_DRIVE_MM) {
    const float leftTravelMm =
        static_cast<float>(left.count - pathStartLeftCount) / LEFT_TICKS_PER_MM;
    const float rightTravelMm =
        static_cast<float>(right.count - pathStartRightCount) /
        RIGHT_TICKS_PER_MM;
    const float progressMm =
        pathDriveDirection * (leftTravelMm + rightTravelMm) * 0.5f;
    const float remainingMm = pathTargetDistanceMm - progressMm;

    if (remainingMm <= DRIVE_TOLERANCE_MM) {
      startPathSettle(false);
      return;
    }
    if (nowMs - pathStepStartedMs > DRIVE_STEP_TIMEOUT_MS) {
      enterFault(FAULT_DRIVE_TIMEOUT);
      return;
    }
    if (progressMm > pathLastProgressMetric + DRIVE_PROGRESS_FOR_STALL_MM) {
      pathLastProgressMetric = progressMm;
      pathLastProgressMs = nowMs;
    } else if (nowMs - pathLastProgressMs > STALL_TIMEOUT_MS) {
      enterFault(FAULT_DRIVE_STALL);
      return;
    }
    if (!updatePathWheelWatchdog(nowMs, left, right, pathDriveDirection,
                                 pathDriveDirection)) {
      return;
    }

    const float baseSpeedMmPerSecond = pathDriveDirection * clampFloat(
        remainingMm * DRIVE_SLOWDOWN_GAIN, DRIVE_MIN_MM_PER_SECOND,
        DRIVE_CRUISE_MM_PER_SECOND);
    const float headingError =
        wrapAngleRadians(pathTargetHeadingRad - pose.headingRad);
    const float headingCorrectionMmPerSecond = clampFloat(
        headingError * HEADING_CORRECTION_MM_PER_SECOND_PER_RADIAN,
        -MAX_HEADING_CORRECTION_MM_PER_SECOND,
        MAX_HEADING_CORRECTION_MM_PER_SECOND);

    setWheelTargetsFromMmPerSecond(
        baseSpeedMmPerSecond - headingCorrectionMmPerSecond,
        baseSpeedMmPerSecond + headingCorrectionMmPerSecond);
    return;
  }

  const float headingError = pathTargetHeadingRad - pose.headingRad;
  const float remainingRadians = absoluteFloat(headingError);
  if (remainingRadians <= TURN_TOLERANCE_RAD) {
    startPathSettle(true);
    return;
  }
  if (nowMs - pathStepStartedMs > TURN_STEP_TIMEOUT_MS) {
    enterFault(FAULT_TURN_TIMEOUT);
    return;
  }
  const float turnDirection = headingError >= 0.0f ? 1.0f : -1.0f;
  if (remainingRadians < pathLastProgressMetric - TURN_PROGRESS_FOR_STALL_RAD) {
    pathLastProgressMetric = remainingRadians;
    pathLastProgressMs = nowMs;
  } else if (nowMs - pathLastProgressMs > STALL_TIMEOUT_MS) {
    enterFault(FAULT_TURN_STALL);
    return;
  }
  if (!updatePathWheelWatchdog(nowMs, left, right, -turnDirection,
                               turnDirection)) {
    return;
  }

  const float wheelSpeedMmPerSecond = clampFloat(
      remainingRadians * TURN_SLOWDOWN_GAIN, TURN_MIN_WHEEL_MM_PER_SECOND,
      TURN_CRUISE_WHEEL_MM_PER_SECOND);
  setWheelTargetsFromMmPerSecond(-turnDirection * wheelSpeedMmPerSecond,
                                 turnDirection * wheelSpeedMmPerSecond);
}

void startManualMotorCommand(int leftPwm, int rightPwm) {
  faultCode = FAULT_NONE;
  motionMode = MOTION_MANUAL;
  activePath = 0;
  activePathLength = 0;
  pathSettling = false;
  manualLeftPwm = constrain(leftPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  manualRightPwm = constrain(rightPwm, -MOTOR_MAX_PWM, MOTOR_MAX_PWM);
  manualExpiresMs = millis() + MANUAL_COMMAND_TIMEOUT_MS;
  setWheelTargetsFromMmPerSecond(0.0f, 0.0f);
  immediatelyStopWheelControllers();
  Serial.print(F("Manual PWM L="));
  Serial.print(manualLeftPwm);
  Serial.print(F(",R="));
  Serial.print(manualRightPwm);
  Serial.println(F(" (auto-stops in 1.2 s)"));
}

void controlStep(float dtSeconds) {
  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);

  updateWheelMeasurement(leftController, left.count, dtSeconds);
  updateWheelMeasurement(rightController, right.count, dtSeconds);
  updateOdometry(left.count, right.count);
  updateEncoderPreflight(left, right);

  if (motionMode == MOTION_PATH) {
    if (consecutiveImuReadFailures > 20U) {
      enterFault(FAULT_IMU_READ);
      return;
    }
    updatePathControl();
    if (motionMode == MOTION_PATH) {
      slewWheelTarget(leftController, dtSeconds);
      slewWheelTarget(rightController, dtSeconds);
      const int leftPwm = calculateWheelPwm(leftController, dtSeconds);
      const int rightPwm = calculateWheelPwm(rightController, dtSeconds);
      setMotorOutputs(leftPwm, rightPwm);
    }
    return;
  }

  if (motionMode == MOTION_MANUAL) {
    if (static_cast<long>(millis() - manualExpiresMs) >= 0) {
      stopMotion(false);
      Serial.println(F("Manual command timed out. Motors stopped."));
      return;
    }
    setMotorOutputs(manualLeftPwm, manualRightPwm);
    return;
  }

  setMotorOutputs(0, 0);
}

// ---------------------------------------------------------------------------
// Serial diagnostics and commands.
// ---------------------------------------------------------------------------
void printMotionMode() {
  switch (motionMode) {
    case MOTION_IDLE:
      Serial.print(F("idle"));
      break;
    case MOTION_MANUAL:
      Serial.print(F("manual"));
      break;
    case MOTION_PATH:
      Serial.print(F("path"));
      break;
    case MOTION_FAULT:
      Serial.print(F("fault"));
      break;
  }
}

void printEncoderLine(const __FlashStringHelper *name,
                      const EncoderSnapshot &encoder, float ticksPerRev) {
  Serial.print(name);
  Serial.print(F(" count="));
  Serial.print(encoder.count);
  Serial.print(F(",turns="));
  Serial.print(static_cast<float>(encoder.count) / ticksPerRev, 3);
  Serial.print(F(",edges="));
  Serial.print(encoder.validEdges);
  Serial.print(F(",A_edges="));
  Serial.print(encoder.aEdges);
  Serial.print(F(",B_edges="));
  Serial.print(encoder.bEdges);
  Serial.print(F(",invalid="));
  Serial.println(encoder.invalidTransitions);
}

void printEncoderDiagnostics() {
  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);
  Serial.print(F("pins LA="));
  Serial.print(digitalRead(LEFT_ENCODER_A_PIN));
  Serial.print(F(",LB="));
  Serial.print(digitalRead(LEFT_ENCODER_B_PIN));
  Serial.print(F(",RA="));
  Serial.print(digitalRead(RIGHT_ENCODER_A_PIN));
  Serial.print(F(",RB="));
  Serial.println(digitalRead(RIGHT_ENCODER_B_PIN));
  Serial.print(F("encoder_preflight="));
  Serial.println(encoderPreflightPassed ? F("passed") : F("pending"));
  printEncoderLine(F("left"), left, LEFT_TICKS_PER_WHEEL_REVOLUTION);
  printEncoderLine(F("right"), right, RIGHT_TICKS_PER_WHEEL_REVOLUTION);
}

void printImuStatus() {
  Serial.print(F("MPU6050 present="));
  Serial.print(imuPresent ? F("yes") : F("no"));
  Serial.print(F(",address=0x"));
  Serial.print(imuAddress, HEX);
  Serial.print(F(",calibrated="));
  Serial.print(imuCalibrated ? F("yes") : F("no"));
  Serial.print(F(",roll_deg="));
  Serial.print(accelerometerRollDegrees, 1);
  Serial.print(F(",pitch_deg="));
  Serial.print(accelerometerPitchDegrees, 1);
  Serial.print(F(",gyro_x_dps_raw="));
  Serial.print(gyroXDegreesPerSecondRaw, 2);
  Serial.print(F(",gyro_y_dps_raw="));
  Serial.print(gyroYDegreesPerSecondRaw, 2);
  Serial.print(F(",gyro_z_dps="));
  Serial.print(gyroZDegreesPerSecond, 2);
  Serial.print(F(",bias_raw="));
  Serial.print(gyroZBiasRaw, 2);
  Serial.print(F(",read_failures="));
  Serial.print(imuReadFailures);
  Serial.print(F(",consecutive_failures="));
  Serial.println(consecutiveImuReadFailures);
}

void printAttitudeTelemetry() {
  if (!attitudeMonitorEnabled) {
    return;
  }
  if (motionMode != MOTION_IDLE) {
    attitudeMonitorEnabled = false;
    Serial.println(F("Attitude monitor stopped while vehicle is moving."));
    return;
  }

  const unsigned long nowMs = millis();
  if (nowMs - lastAttitudeReportMs < ATTITUDE_REPORT_INTERVAL_MS) {
    return;
  }
  lastAttitudeReportMs = nowMs;

  Serial.print(F("ATT roll="));
  Serial.print(accelerometerRollDegrees, 1);
  Serial.print(F(",pitch="));
  Serial.print(accelerometerPitchDegrees, 1);
  Serial.print(F(",yaw_rel="));
  Serial.print(attitudeYawRelativeDegrees, 1);
  Serial.print(F(",accel_g=("));
  Serial.print(accelerometerXG, 2);
  Serial.print(F(","));
  Serial.print(accelerometerYG, 2);
  Serial.print(F(","));
  Serial.print(accelerometerZG, 2);
  Serial.print(F("),gyro_dps=("));
  Serial.print(gyroXDegreesPerSecondRaw, 1);
  Serial.print(F(","));
  Serial.print(gyroYDegreesPerSecondRaw, 1);
  Serial.print(F(","));
  Serial.print(gyroZDegreesPerSecond, 1);
  Serial.println(F(")"));
}

void printConfiguration() {
  Serial.print(F("ticks/rev L="));
  Serial.print(LEFT_TICKS_PER_WHEEL_REVOLUTION, 1);
  Serial.print(F(",R="));
  Serial.print(RIGHT_TICKS_PER_WHEEL_REVOLUTION, 1);
  Serial.print(F(",wheel_diameter_mm="));
  Serial.print(WHEEL_DIAMETER_MM, 1);
  Serial.print(F(",track_mm="));
  Serial.println(WHEEL_TRACK_MM, 1);
}

void printStatus() {
  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);

  Serial.print(F("mode="));
  printMotionMode();
  Serial.print(F(",pose_mm=("));
  Serial.print(pose.xMm, 1);
  Serial.print(F(","));
  Serial.print(pose.yMm, 1);
  Serial.print(F("),heading_deg="));
  Serial.print(pose.headingRad * 180.0f / PI_F, 1);
  Serial.print(F(",L[count="));
  Serial.print(left.count);
  Serial.print(F(",tps="));
  Serial.print(leftController.measuredTicksPerSecond, 1);
  Serial.print(F(",target="));
  Serial.print(leftController.targetTicksPerSecond, 1);
  Serial.print(F(",pwm="));
  Serial.print(appliedLeftPwm);
  Serial.print(F("],R[count="));
  Serial.print(right.count);
  Serial.print(F(",tps="));
  Serial.print(rightController.measuredTicksPerSecond, 1);
  Serial.print(F(",target="));
  Serial.print(rightController.targetTicksPerSecond, 1);
  Serial.print(F(",pwm="));
  Serial.print(appliedRightPwm);
  Serial.print(F("],fault="));
  printFaultCode();
  Serial.print(F(",encoder_preflight="));
  Serial.print(encoderPreflightPassed ? F("passed") : F("pending"));
  if (motionMode == MOTION_PATH) {
    Serial.print(F(",path="));
    Serial.print(activePathNumber);
    Serial.print(F(",step="));
    Serial.print(pathStepIndex + 1);
  }
  Serial.println();
}

void printHelp() {
  Serial.println(F("Follower 2 calibration controller ready. Motors are stopped after boot."));
  Serial.println(F("Follower 2 provisional geometry: G1 enabled; G2/G3 locked."));
  Serial.println(F("Commands:"));
  Serial.println(F("  F / B       manual forward / backward PWM 80 (1.2 s max)"));
  Serial.println(F("  M<L>,<R>    manual PWM, e.g. M80,80 or M-80,80"));
  Serial.println(F("  S           immediate coast stop and clear a fault"));
  Serial.println(F("  R           stop, reset both encoder counts and pose"));
  Serial.println(F("  C           calibrate MPU6050 gyro; vehicle must be still"));
  Serial.println(F("  G1          500 mm straight path"));
  Serial.println(F("  G2          400 mm square path"));
  Serial.println(F("  G3          350 mm L-shaped path"));
  Serial.println(F("  P           current pose, speeds, PWM, and fault"));
  Serial.println(F("  D           encoder pin levels and AB diagnostics"));
  Serial.println(F("  I           MPU6050 diagnostic"));
  Serial.println(F("  T           toggle live MPU6050 attitude monitor (motors stopped)"));
  Serial.println(F("  K           geometry/tick calibration constants"));
  Serial.println(F("  ?           this help"));
}

bool parseManualPwm(const char *text, int &leftPwm, int &rightPwm) {
  char *end = 0;
  const long leftValue = strtol(text, &end, 10);
  if (end == text || *end != ',') {
    return false;
  }
  const char *rightText = end + 1;
  const long rightValue = strtol(rightText, &end, 10);
  if (end == rightText || *end != '\0') {
    return false;
  }
  leftPwm =
      static_cast<int>(constrain(leftValue, -MOTOR_MAX_PWM, MOTOR_MAX_PWM));
  rightPwm =
      static_cast<int>(constrain(rightValue, -MOTOR_MAX_PWM, MOTOR_MAX_PWM));
  return true;
}

void resetCountersAndPose() {
  stopMotion(true);
  resetEncoderCounters();
  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);
  resetWheelControllersFromSnapshot(left, right);
  resetPoseAndOdometry(left, right);
  Serial.println(F("Encoder counts and relative pose reset. Motors stopped."));
}

#ifdef FOLLOWER2_AUTORUN_G1
void updateFollower2AutoRun() {
  switch (follower2AutoRunState) {
    case FOLLOWER2_WAITING_FOR_STILLNESS:
      if (millis() - follower2AutoRunStartedMs <
          FOLLOWER2_AUTORUN_STILLNESS_DELAY_MS) {
        return;
      }

      // The vehicle is stationary for the 300-sample gyro calibration.
      stopMotion(true);
      Serial.println(F("F2 demo: stillness delay complete; calibrating gyro Z."));
      if (!calibrateGyroZ()) {
        enterFault(FAULT_IMU_READ);
        follower2AutoRunState = FOLLOWER2_FAULT;
        return;
      }

      resetCountersAndPose();

      // The full Follower 2 profile has already verified motor direction,
      // encoder A/B health and logical encoder sign. An autonomous image
      // cannot repeat that human-observed check on every boot. This bypass is
      // limited to this demo build; runtime wheel/MPU/timeout protections stay.
      encoderPreflightPassed = true;
      Serial.println(F("F2 demo: using F2-certified encoder preflight; starting G1."));
      startPresetPath(1);
      if (motionMode != MOTION_PATH) {
        stopMotion(false);
        Serial.println(F("F2 demo: G1 could not start; motors stopped."));
        follower2AutoRunState = FOLLOWER2_FAULT;
        return;
      }
      follower2AutoRunState = FOLLOWER2_RUNNING_G1;
      return;

    case FOLLOWER2_RUNNING_G1:
      if (motionMode == MOTION_IDLE) {
        follower2AutoRunState = FOLLOWER2_COMPLETE;
        Serial.println(F("F2 demo complete. Motors remain stopped until power cycle."));
      } else if (motionMode == MOTION_FAULT) {
        follower2AutoRunState = FOLLOWER2_FAULT;
      }
      return;

    case FOLLOWER2_COMPLETE:
    case FOLLOWER2_FAULT:
      // One deliberate attempt per reset/power cycle.
      return;
  }
}
#endif

void handleCommand(char *command) {
  char op = command[0];
  if (op >= 'a' && op <= 'z') {
    op = static_cast<char>(op - 'a' + 'A');
  }

  switch (op) {
    case 'F':
      startManualMotorCommand(DEFAULT_MANUAL_PWM, DEFAULT_MANUAL_PWM);
      break;
    case 'B':
      startManualMotorCommand(-DEFAULT_MANUAL_PWM, -DEFAULT_MANUAL_PWM);
      break;
    case 'M': {
      int leftPwm = 0;
      int rightPwm = 0;
      if (!parseManualPwm(command + 1, leftPwm, rightPwm)) {
        Serial.println(F("Manual format: M<L>,<R>, e.g. M80,80"));
        break;
      }
      startManualMotorCommand(leftPwm, rightPwm);
      break;
    }
    case 'S':
      stopMotion(true);
      Serial.println(F("Motors stopped."));
      break;
    case 'R':
      resetCountersAndPose();
      break;
    case 'C':
      stopMotion(true);
      calibrateGyroZ();
      break;
    case 'G': {
      const uint8_t pathNumber =
          command[1] == '\0' ? 2 : static_cast<uint8_t>(atoi(command + 1));
      startPresetPath(pathNumber);
      break;
    }
    case 'P':
      printStatus();
      break;
    case 'D':
      printEncoderDiagnostics();
      break;
    case 'I':
      printImuStatus();
      break;
    case 'T':
      if (!imuPresent) {
        Serial.println(F("MPU6050 not detected; attitude monitor unavailable."));
        break;
      }
      attitudeMonitorEnabled = !attitudeMonitorEnabled;
      if (attitudeMonitorEnabled) {
        attitudeYawRelativeDegrees = 0.0f;
      }
      lastAttitudeReportMs = 0;
      Serial.println(attitudeMonitorEnabled
                         ? F("Attitude monitor on. Send T again to stop.")
                         : F("Attitude monitor off."));
      break;
    case 'K':
      printConfiguration();
      break;
    case '?':
    case 'H':
      printHelp();
      break;
    default:
      Serial.println(F("Invalid command. Send ? for help."));
      break;
  }
}

void readSerialCommands() {
  static char buffer[40];
  static uint8_t length = 0;
  static unsigned long lastInputMs = 0;

  while (Serial.available() > 0) {
    const char input = static_cast<char>(Serial.read());
    if (input == '\n' || input == '\r') {
      if (length > 0) {
        buffer[length] = '\0';
        handleCommand(buffer);
        length = 0;
      }
    } else if (input >= 32 && input <= 126 && length < sizeof(buffer) - 1) {
      buffer[length++] = input;
      lastInputMs = millis();
    } else if (input >= 32 && input <= 126) {
      length = 0;
      Serial.println(F("Command too long."));
    }
  }

  // Supports serial monitors that do not append a newline.
  if (length > 0 && millis() - lastInputMs >= COMMAND_IDLE_MS) {
    buffer[length] = '\0';
    handleCommand(buffer);
    length = 0;
  }
}

void reportTelemetry() {
  static unsigned long lastReportMs = 0;
  if (motionMode == MOTION_IDLE || motionMode == MOTION_FAULT) {
    return;
  }
  const unsigned long nowMs = millis();
  if (nowMs - lastReportMs < REPORT_INTERVAL_MS) {
    return;
  }
  lastReportMs = nowMs;
  printStatus();
}

void disableWatchdogAfterReset() {
  // If a previous I2C/loop lock-up caused a watchdog reset, disable the
  // watchdog immediately so setup can restore the hardware safe state.
  MCUSR &= static_cast<uint8_t>(~_BV(WDRF));
  wdt_disable();
}

void enableSafetyWatchdog() {
  // The loop kicks this only after it has serviced I2C, control and telemetry.
  // A hard lock-up therefore resets the Nano and removes software PWM output.
  wdt_enable(WDTO_250MS);
}

void setup() {
  disableWatchdogAfterReset();
  Serial.begin(115200);
#ifdef FOLLOWER2_AUTORUN_G1
  Serial.println(F("FIRMWARE_PROFILE=F2-DEMO (one-shot autonomous G1)"));
#else
  Serial.println(F("FIRMWARE_PROFILE=F2 (follower2 calibration pending)"));
#endif

  // Establish a physical safe state before enabling any sensor or controller.
  configureMotorPinsAndStop();
  setupEncoders();
  resetEncoderCounters();

  EncoderSnapshot left;
  EncoderSnapshot right;
  readEncoderSnapshots(left, right);
  resetWheelControllersFromSnapshot(left, right);
  resetPoseAndOdometry(left, right);

  Wire.begin();
  // 100 kHz is deliberately tolerant of breadboard jumpers and motor noise;
  // a 2-byte gyro read still fits easily within the 100 Hz control budget.
  Wire.setClock(100000UL);
  Wire.setWireTimeout(WIRE_TIMEOUT_US, true);
  Wire.clearWireTimeoutFlag();
  if (initialiseMpu6050()) {
#ifdef FOLLOWER2_AUTORUN_G1
    Serial.println(F("MPU6050 detected. F2 demo will calibrate while still, then run G1."));
#else
    Serial.println(F("MPU6050 detected. Send C with the vehicle still."));
#endif
  } else {
#ifdef FOLLOWER2_AUTORUN_G1
    Serial.println(F("MPU6050 not detected. F2 demo will remain stopped."));
#else
    Serial.println(F("MPU6050 not detected. Manual tests work; G paths are blocked."));
#endif
  }

#ifdef FOLLOWER2_AUTORUN_G1
  follower2AutoRunStartedMs = millis();
  Serial.println(F("F2 demo: keep vehicle completely still for 3 s after power-on."));
#else
  Serial.println(F("F2: verify pin map and record MPU/encoder data before enabling paths."));
  printHelp();
  printConfiguration();
#endif
  enableSafetyWatchdog();
}

void loop() {
#ifndef FOLLOWER2_AUTORUN_G1
  readSerialCommands();
#endif
  updateImu();
#ifndef FOLLOWER2_AUTORUN_G1
  printAttitudeTelemetry();
#endif
  if (imuBusTimeoutOccurred) {
    if (motionMode == MOTION_PATH || motionMode == MOTION_MANUAL) {
      enterFault(FAULT_IMU_BUS_TIMEOUT);
    }
    imuBusTimeoutOccurred = false;
  }

#ifdef FOLLOWER2_AUTORUN_G1
  updateFollower2AutoRun();
#endif

  static unsigned long lastControlUs = micros();
  const unsigned long nowUs = micros();
  const unsigned long elapsedUs = nowUs - lastControlUs;
  if (elapsedUs >= CONTROL_INTERVAL_US) {
    lastControlUs = nowUs;
    const float dtSeconds = clampFloat(
        static_cast<float>(elapsedUs) / 1000000.0f, 0.001f, 0.050f);
    controlStep(dtSeconds);
  }

#ifndef FOLLOWER2_AUTORUN_G1
  reportTelemetry();
#endif
  wdt_reset();
}
