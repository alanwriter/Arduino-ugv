#pragma once

// ---------------------------------------------------------------------------
// Follower 2 vehicle profile. This is the only vehicle-specific source file.
// The shared controller, PID, safety checks and demo state machine stay equal
// across Follower 1, Follower 2 and Leader 1.
// ---------------------------------------------------------------------------

#define VEHICLE_PROFILE_HELP_TITLE \
  "Follower 2 calibration controller ready. Motors are stopped after boot."
#define VEHICLE_PROFILE_PATH_POLICY \
  "Follower 2 supervised first 700 mm G2 square: G1/G2 enabled; G3 locked."
#define VEHICLE_PROFILE_BOOT_MESSAGE \
  "FIRMWARE_PROFILE=F2 (follower2 supervised G2 square trial ready)"
#define VEHICLE_PROFILE_DEMO_BOOT_MESSAGE \
  "FIRMWARE_PROFILE=F2-DEMO (one-shot autonomous preset path)"
#define VEHICLE_PROFILE_STARTUP_NOTICE \
  "F2: first 700 mm G2 square is supervised; G3 remains locked."

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

// 2026-09-06 F2 measurement: decoded 4x AB counts across 10 marked turns.
constexpr float LEFT_TICKS_PER_WHEEL_REVOLUTION = 1216.2f;
constexpr float RIGHT_TICKS_PER_WHEEL_REVOLUTION = 1237.4f;
// Shared 2026-09-06 geometry assumption for F1, F2 and L1.  Refine only from
// repeated floor tests; the same physical wheel/chassis dimensions are used.
constexpr float WHEEL_DIAMETER_MM = 65.0f; // Provisional.
constexpr float WHEEL_TRACK_MM = 130.0f;   // Provisional.

constexpr bool LEFT_MOTOR_REVERSED = false;
constexpr bool RIGHT_MOTOR_REVERSED = false;
constexpr bool LEFT_ENCODER_REVERSED = false;
constexpr bool RIGHT_ENCODER_REVERSED = true;
constexpr bool GYRO_Z_REVERSED = false;
// F2 needs a small extra signed output only while a path executes a turn.
// It does not change straight segments or serial manual PWM commands.
constexpr int TURN_PWM_BIAS = 10;

// This authorizes one supervised first G2 calibration run, not final square
// approval. Keep G3 locked until closure error and turn geometry are measured.
constexpr uint8_t MAX_APPROVED_PRESET_PATH = 2;
