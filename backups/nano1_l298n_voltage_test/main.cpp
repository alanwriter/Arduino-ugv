#include <Arduino.h>

// Nano #1 fixed L298N wiring. Do not change these pins.
constexpr uint8_t LEFT_IN1_PIN = 3;
constexpr uint8_t LEFT_IN2_PIN = 4;
constexpr uint8_t RIGHT_IN1_PIN = 5;
constexpr uint8_t RIGHT_IN2_PIN = 6;
constexpr uint8_t LEFT_PWM_PIN = 9;   // L298N ENA
constexpr uint8_t RIGHT_PWM_PIN = 10; // L298N ENB

constexpr unsigned long COMMAND_IDLE_MS = 80;

int pwmValue = 0;
bool forwardMode = false;

void stopMotors() {
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, LOW);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void applyMotorOutput() {
  if (!forwardMode || pwmValue == 0) {
    stopMotors();
    return;
  }

  // Forward mode: D3/D5 HIGH, D4/D6 LOW.
  digitalWrite(LEFT_IN1_PIN, HIGH);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN1_PIN, HIGH);
  digitalWrite(RIGHT_IN2_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, pwmValue);
  analogWrite(RIGHT_PWM_PIN, pwmValue);
}

void printStatus() {
  Serial.print(F("mode="));
  Serial.print(forwardMode ? F("F") : F("S"));
  Serial.print(F(", pwm="));
  Serial.println(pwmValue);
}

void printHelp() {
  Serial.println(F("L298N voltage test commands:"));
  Serial.println(F("  F        forward mode using current PWM"));
  Serial.println(F("  S        stop; all L298N control outputs LOW"));
  Serial.println(F("  0..255   set PWM, e.g. 255 or 128"));
  Serial.println(F("  ?        show this help"));
}

void handleCommand(char *command) {
  if (command[0] == 'F' || command[0] == 'f') {
    forwardMode = true;
    applyMotorOutput();
    printStatus();
    return;
  }

  if (command[0] == 'S' || command[0] == 's') {
    forwardMode = false;
    applyMotorOutput();
    printStatus();
    return;
  }

  if (command[0] == '?' || command[0] == 'H' || command[0] == 'h') {
    printHelp();
    return;
  }

  bool isNumber = true;
  for (char *character = command; *character != '\0'; ++character) {
    if (*character < '0' || *character > '9') {
      isNumber = false;
      break;
    }
  }

  if (isNumber) {
    pwmValue = constrain(atoi(command), 0, 255);
    applyMotorOutput();
    printStatus();
    return;
  }

  Serial.println(F("Invalid command. Send ? for help."));
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
  printStatus();
}

void loop() {
  readSerialCommands();
}
