#pragma once

// ---------------------------------------------------------------------------
// Follower 1 vehicle profile. This is the only vehicle-specific source file.
// The shared controller, PID, safety checks and demo state machine stay equal
// across Follower 1, Follower 2 and Leader 1.
// ---------------------------------------------------------------------------

#define VEHICLE_PROFILE_HELP_TITLE \
  "Follower 1 controller ready. Motors are stopped after boot."
#define VEHICLE_PROFILE_PATH_POLICY \
  "Follower 1 calibration is approved: G1/G2/G3 are available."
#define VEHICLE_PROFILE_BOOT_MESSAGE \
  "FIRMWARE_PROFILE=F1 (follower1 calibrated build)"
#define VEHICLE_PROFILE_DEMO_BOOT_MESSAGE \
  "FIRMWARE_PROFILE=F1-DEMO (one-shot autonomous preset path)"
#define VEHICLE_PROFILE_STARTUP_NOTICE \
  "F1: use C while still before a path; verify any changed wiring first."

// Current shared controller requires this standardized encoder interrupt map:
// left A/B = D2/D8, right A/B = D7/D12. The low-level AVR decoder uses these
// port/PCINT routes, so a future different encoder map needs an explicit ISR
// refactor rather than only changing these constants.
constexpr uint8_t LEFT_ENCODER_A_PIN = 2;
constexpr uint8_t LEFT_ENCODER_B_PIN = 8;
constexpr uint8_t RIGHT_ENCODER_A_PIN = 7;
constexpr uint8_t RIGHT_ENCODER_B_PIN = 12;
constexpr uint8_t LEFT_IN1_PIN = 3;
constexpr uint8_t LEFT_IN2_PIN = 4;
constexpr uint8_t RIGHT_IN1_PIN = 5;
constexpr uint8_t RIGHT_IN2_PIN = 6;
constexpr uint8_t LEFT_PWM_PIN = 9;
constexpr uint8_t RIGHT_PWM_PIN = 10;
constexpr uint8_t MPU6050_SDA_PIN = A4;
constexpr uint8_t MPU6050_SCL_PIN = A5;

// 2026-09-06 F1 measurement: 12,035 decoded ticks across 10 marked left
// wheel revolutions. Right remains a conservative provisional 1200 value.
constexpr float LEFT_TICKS_PER_WHEEL_REVOLUTION = 1203.5f;
constexpr float RIGHT_TICKS_PER_WHEEL_REVOLUTION = 1200.0f;
constexpr float WHEEL_DIAMETER_MM = 65.0f;
constexpr float WHEEL_TRACK_MM = 130.0f;

constexpr bool LEFT_MOTOR_REVERSED = false;
constexpr bool RIGHT_MOTOR_REVERSED = false;
constexpr bool LEFT_ENCODER_REVERSED = false;
constexpr bool RIGHT_ENCODER_REVERSED = true;
constexpr bool GYRO_Z_REVERSED = false;
// F1's calibrated turns need no additional path-turn output bias.
constexpr int TURN_PWM_BIAS = 0;

// F1's current full profile can use all supplied preset paths.
constexpr uint8_t MAX_APPROVED_PRESET_PATH = 3;
