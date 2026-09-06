#include <Arduino.h>

// Nano #1 fixed L298N wiring. IN1~IN4 are physically soldered to D3~D6.
constexpr uint8_t LEFT_IN1_PIN = 3;
constexpr uint8_t LEFT_IN2_PIN = 4;
constexpr uint8_t RIGHT_IN1_PIN = 5;
constexpr uint8_t RIGHT_IN2_PIN = 6;
constexpr uint8_t LEFT_PWM_PIN = 9;   // L298N ENA
constexpr uint8_t RIGHT_PWM_PIN = 10; // L298N ENB

constexpr uint8_t TEST_PWM =200;
constexpr unsigned long COMMAND_IDLE_MS = 80;

void stopMotors() {
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, LOW);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void driveForward(uint8_t pwm) {
  digitalWrite(LEFT_IN1_PIN, HIGH);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, HIGH);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, pwm);
  analogWrite(RIGHT_PWM_PIN, pwm);
}

void driveBackward(uint8_t pwm) {
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, HIGH);
  digitalWrite(RIGHT_IN1_PIN, LOW);
  digitalWrite(RIGHT_IN2_PIN, HIGH);
  analogWrite(LEFT_PWM_PIN, pwm);
  analogWrite(RIGHT_PWM_PIN, pwm);
}

void printHelp() {
  Serial.println(F("L298N motor test ready. Commands:"));
  Serial.println(F("  F  both motors forward, PWM 100"));
  Serial.println(F("  B  both motors backward, PWM 100"));
  Serial.println(F("  S  stop both motors"));
  Serial.println(F("  ?  show this help"));
}

void handleCommand(char *command) {
  switch (command[0]) {
    case 'F': case 'f':
      driveForward(TEST_PWM);
      Serial.println(F("Forward, PWM=100"));
      return;
    case 'B': case 'b':
      driveBackward(TEST_PWM);
      Serial.println(F("Backward, PWM=100"));
      return;
    case 'S': case 's':
      stopMotors();
      Serial.println(F("Motors stopped"));
      return;
    case '?': case 'H': case 'h':
      printHelp();
      return;
    default:
      Serial.println(F("Invalid command. Send ? for help."));
  }
}

void readSerialCommands() {
  static char buffer[12];
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
    }
  }

  // Supports serial monitor tools that do not append a newline.
  if (length > 0 && millis() - lastInputMs >= COMMAND_IDLE_MS) {
    buffer[length] = '\0';
    handleCommand(buffer);
    length = 0;
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_IN1_PIN, OUTPUT);
  pinMode(RIGHT_IN2_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  stopMotors();
  printHelp();
}

void loop() {
  readSerialCommands();
}
