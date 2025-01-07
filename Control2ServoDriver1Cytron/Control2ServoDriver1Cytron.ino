#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Create two PCA9685 instances
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Default I2C address for PCA9685
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x42);  // Second PCA9685 with alternate address

// Servo settings
#define SERVO_FREQ 50  // 50 Hz update rate for servos

// Cytron Motor Control Pins
#define PWM1_PIN 18
#define PWM2_PIN 33
#define DIR1_PIN 19
#define DIR2_PIN 32

void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize PCA9685 instances
  pca1.begin();
  pca1.setPWMFreq(SERVO_FREQ);  // 50 Hz for servos
  pca2.begin();
  pca2.setPWMFreq(SERVO_FREQ);  // 50 Hz for servos

  // Initialize Cytron motor control pins
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  // Initialize motors to stop
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);

  Serial.println("PCA9685 Servo Controller and Cytron Motor Controller Ready");
}

void loop() {
  // Check for incoming serial data
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    Serial.print("Received input: ");
    Serial.println(input);

    parseAndSetServos(input);
  }
}

void parseAndSetServos(String input) {
  // Example input: "L1:45,R1:90,L2:180,R2:181,LDC:100,RDC:-100"
  input.trim();

  if (input.length() == 0) {
    Serial.println("No input received.");
    return;
  }

  String pairs[32];
  int pairCount = 0;

  // Split input by commas
  int startIdx = 0;
  for (int i = 0; i < input.length(); i++) {
    if (input[i] == ',') {
      pairs[pairCount++] = input.substring(startIdx, i);
      startIdx = i + 1;
    }
  }
  pairs[pairCount++] = input.substring(startIdx);

  // Parse each key:value pair
  for (int i = 0; i < pairCount; i++) {
    int colonIdx = pairs[i].indexOf(':');
    if (colonIdx == -1) {
      Serial.print("Invalid pair: ");
      Serial.println(pairs[i]);
      continue;  // Skip invalid pairs
    }

    String key = pairs[i].substring(0, colonIdx);
    int value = pairs[i].substring(colonIdx + 1).toInt();

    Serial.print("Processing: ");
    Serial.print(key);
    Serial.print(" -> ");
    Serial.println(value);

    if (key == "LDC" || key == "RDC") {
      setMotorSpeed(key, value);
    } else {
      setServoPosition(key, value);
    }
  }
}

void setServoPosition(String key, int degrees) {
  // Map degrees to PCA9685 ticks (calculated based on pulse lengths for 0 to 180 degrees)
  int pulseLength = map(degrees, 0, 180, 102, 512);  // Adjust range as needed for your servos

  if (key.startsWith("L")) {
    int servoNum = key.substring(1).toInt() - 1;  // Extract servo number for left PCA
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        // Free the servo by turning off the PWM signal
        Serial.print("Freeing PCA1 servo ");
        Serial.println(servoNum);
        pca1.setPin(servoNum, 0, false);
      } else {
        Serial.print("Setting PCA1 servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.println(degrees);
        pca1.setPWM(servoNum, 0, pulseLength);
      }
    } else {
      Serial.print("Invalid left servo number: ");
      Serial.println(servoNum);
    }
  } else if (key.startsWith("R")) {
    int servoNum = key.substring(1).toInt() - 1;  // Extract servo number for right PCA
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        // Free the servo by turning off the PWM signal
        Serial.print("Freeing PCA2 servo ");
        Serial.println(servoNum);
        pca2.setPin(servoNum, 0, false);
      } else {
        Serial.print("Setting PCA2 servo ");
        Serial.print(servoNum);
        Serial.print(" to ");
        Serial.println(degrees);
        pca2.setPWM(servoNum, 0, pulseLength);
      }
    } else {
      Serial.print("Invalid right servo number: ");
      Serial.println(servoNum);
    }
  } else {
    Serial.print("Invalid key: ");
    Serial.println(key);
  }
}

void setMotorSpeed(String motor, int speed) {
  // Speed range: -255 to 255
  speed = constrain(speed, -255, 255);

  if (motor == "LDC") {
    digitalWrite(DIR1_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM1_PIN, abs(speed));
    Serial.print("Left Motor Speed: ");
    Serial.println(speed);
  } else if (motor == "RDC") {
    digitalWrite(DIR2_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM2_PIN, abs(speed));
    Serial.print("Right Motor Speed: ");
    Serial.println(speed);
  }
}
