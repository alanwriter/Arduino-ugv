#include <Arduino.h>
#include <Servo.h>
#include <string.h>

// SG90 signal wires: horizontal -> D9, vertical -> D10
constexpr uint8_t PAN_PIN = 9;
constexpr uint8_t TILT_PIN = 10;

constexpr int MIN_ANGLE = 0;
constexpr int MAX_ANGLE = 180;
constexpr int CENTER_ANGLE = 90;
constexpr unsigned long STEP_DELAY_MS = 15;
constexpr unsigned long COMMAND_IDLE_MS = 80;

Servo panServo;
Servo tiltServo;
int panAngle = CENTER_ANGLE;
int tiltAngle = CENTER_ANGLE;

// Moves one axis one degree at a time, so the motion is smooth.
void moveServo(Servo &servo, int &currentAngle, int targetAngle) {
  targetAngle = constrain(targetAngle, MIN_ANGLE, MAX_ANGLE);
  const int fromAngle = currentAngle;
  const int step = (targetAngle >= fromAngle) ? 1 : -1;

  for (int angle = fromAngle;; angle += step) {
    servo.write(angle);
    delay(STEP_DELAY_MS);

    if (angle == targetAngle) {
      break;
    }
  }

  currentAngle = targetAngle;
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  P<angle>     horizontal, e.g. P120"));
  Serial.println(F("  T<angle>     vertical, e.g. T45"));
  Serial.println(F("  <pan>,<tilt> set both in order, e.g. 120,45"));
  Serial.println(F("  C            center both axes (90,90)"));
  Serial.println(F("  ?            show this help"));
}

void printPosition() {
  Serial.print(F("PAN="));
  Serial.print(panAngle);
  Serial.print(F(", TILT="));
  Serial.println(tiltAngle);
}

void handleCommand(char *command) {
  if (command[0] == '?' || command[0] == 'h' || command[0] == 'H') {
    printHelp();
    return;
  }

  if (command[0] == 'c' || command[0] == 'C') {
    moveServo(panServo, panAngle, CENTER_ANGLE);
    moveServo(tiltServo, tiltAngle, CENTER_ANGLE);
    printPosition();
    return;
  }

  if (command[0] == 'p' || command[0] == 'P') {
    moveServo(panServo, panAngle, atoi(command + 1));
    printPosition();
    return;
  }

  if (command[0] == 't' || command[0] == 'T') {
    moveServo(tiltServo, tiltAngle, atoi(command + 1));
    printPosition();
    return;
  }

  char *separator = strchr(command, ',');
  if (separator != nullptr) {
    *separator = '\0';
    moveServo(panServo, panAngle, atoi(command));
    moveServo(tiltServo, tiltAngle, atoi(separator + 1));
    printPosition();
    return;
  }

  Serial.println(F("Invalid command. Send ? for help."));
}

void readSerialCommands() {
  static char buffer[24];
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
    } else if (length < sizeof(buffer) - 1) {
      buffer[length++] = input;
      lastInputMs = millis();
    } else {
      length = 0;
      Serial.println(F("Command too long."));
    }
  }

  // Some serial-monitor tools do not send a newline when transmitting.
  // Execute the buffered command after a brief quiet period in that case.
  if (length > 0 && millis() - lastInputMs >= COMMAND_IDLE_MS) {
    buffer[length] = '\0';
    handleCommand(buffer);
    length = 0;
  }
}

void setup() {
  Serial.begin(115200);

  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  panServo.write(panAngle);
  tiltServo.write(tiltAngle);
  Serial.println(F("2-axis gimbal ready."));
  printHelp();
  printPosition();
}

void loop() {
  readSerialCommands();
}
