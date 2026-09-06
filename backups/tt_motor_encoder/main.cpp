#include <Arduino.h>
#include <util/atomic.h>

// L298N connections. Remove the ENA jumper before connecting ENA to D10.
constexpr uint8_t MOTOR_IN1_PIN = 8;
constexpr uint8_t MOTOR_IN2_PIN = 9;
constexpr uint8_t MOTOR_ENA_PIN = 10;

// Verified encoder colors: orange (A) -> D2, green (B) -> D3.
// Yellow is encoder VCC (5V); white is encoder GND.
// D2 and D3 are Nano interrupt pins.
constexpr uint8_t ENCODER_A_PIN = 2;
constexpr uint8_t ENCODER_B_PIN = 3;

constexpr bool REVERSE_ENCODER = false;
constexpr int DEFAULT_MOTOR_SPEED = 50;  // PWM range: 0 to 255
constexpr unsigned long REPORT_INTERVAL_MS = 250;
constexpr unsigned long COMMAND_IDLE_MS = 80;

volatile long encoderCount = 0;
int motorCommand = 0;

void onEncoderAChange() {
  const bool encoderA = digitalRead(ENCODER_A_PIN);
  const bool encoderB = digitalRead(ENCODER_B_PIN);
  const int direction = (encoderA == encoderB) ? 1 : -1;
  encoderCount += REVERSE_ENCODER ? -direction : direction;
}

long getEncoderCount() {
  long count;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    count = encoderCount;
  }
  return count;
}

void resetEncoderCount() {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    encoderCount = 0;
  }
}

void setMotor(int command) {
  motorCommand = constrain(command, -255, 255);

  if (motorCommand > 0) {
    digitalWrite(MOTOR_IN1_PIN, HIGH);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  } else if (motorCommand < 0) {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_IN1_PIN, LOW);
    digitalWrite(MOTOR_IN2_PIN, LOW);
  }

  analogWrite(MOTOR_ENA_PIN, abs(motorCommand));
}

void printStatus() {
  Serial.print(F("count="));
  Serial.print(getEncoderCount());
  Serial.print(F(", motor="));
  Serial.println(motorCommand);
}

void printEncoderPins() {
  Serial.print(F("A(D2)="));
  Serial.print(digitalRead(ENCODER_A_PIN));
  Serial.print(F(", B(D3)="));
  Serial.println(digitalRead(ENCODER_B_PIN));
}

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  F       forward at default speed"));
  Serial.println(F("  B       backward at default speed"));
  Serial.println(F("  S       stop motor"));
  Serial.println(F("  M<number>  motor PWM -255 to 255, e.g. M120 or M-120"));
  Serial.println(F("  R       reset encoder count to zero"));
  Serial.println(F("  P       print encoder count now"));
  Serial.println(F("  D       print encoder A/B pin levels"));
  Serial.println(F("  ?       show this help"));
}

void handleCommand(char *command) {
  switch (command[0]) {
    case 'F': case 'f':
      setMotor(DEFAULT_MOTOR_SPEED);
      break;
    case 'B': case 'b':
      setMotor(-DEFAULT_MOTOR_SPEED);
      break;
    case 'S': case 's':
      setMotor(0);
      break;
    case 'M': case 'm':
      setMotor(atoi(command + 1));
      break;
    case 'R': case 'r':
      resetEncoderCount();
      break;
    case 'P': case 'p':
      printStatus();
      return;
    case 'D': case 'd':
      printEncoderPins();
      return;
    case '?': case 'H': case 'h':
      printHelp();
      return;
    default:
      Serial.println(F("Invalid command. Send ? for help."));
      return;
  }
  printStatus();
}

void readSerialCommands() {
  static char buffer[20];
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

  // Support serial-monitor tools that transmit without a newline.
  if (length > 0 && millis() - lastInputMs >= COMMAND_IDLE_MS) {
    buffer[length] = '\0';
    handleCommand(buffer);
    length = 0;
  }
}

void reportEncoderRate() {
  static long previousCount = 0;
  static unsigned long previousMs = 0;
  const unsigned long now = millis();

  if (now - previousMs < REPORT_INTERVAL_MS) {
    return;
  }

  const long count = getEncoderCount();
  const unsigned long elapsedMs = now - previousMs;
  const float ticksPerSecond = (count - previousCount) * 1000.0f / elapsedMs;

  Serial.print(F("count="));
  Serial.print(count);
  Serial.print(F(", rate="));
  Serial.print(ticksPerSecond, 1);
  Serial.println(F(" ticks/s"));

  previousCount = count;
  previousMs = now;
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_IN1_PIN, OUTPUT);
  pinMode(MOTOR_IN2_PIN, OUTPUT);
  pinMode(MOTOR_ENA_PIN, OUTPUT);
  setMotor(0);

  pinMode(ENCODER_A_PIN, INPUT_PULLUP);
  pinMode(ENCODER_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PIN), onEncoderAChange, CHANGE);

  Serial.println(F("TT motor encoder test ready."));
  printHelp();
}

void loop() {
  readSerialCommands();
  reportEncoderRate();
}
