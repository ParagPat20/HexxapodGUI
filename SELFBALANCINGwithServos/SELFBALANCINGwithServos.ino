/*
References:
1. https://learn.adafruit.com/adafruit-mpu6050-6-dof-accelerometer-and-gyroscope
2. https://github.com/adafruit/Adafruit_MPU6050
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// Create instances
Adafruit_MPU6050 mpu;
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x42);

// Madgwick filter parameter
const float beta = 0.1f;  // 2 * proportional gain

// Servo settings
#define SERVO_FREQ 50
#define SERVOMIN  125
#define SERVOMAX  575
#define USMIN  600
#define USMAX  2400

// Cytron Motor Control Pins
#define PWM1_PIN 18
#define PWM2_PIN 33
#define DIR1_PIN 19
#define DIR2_PIN 32

// MPU6050 calibration data structure
struct CalibrationData {
  float accel_bias[3];
  float gyro_bias[3];
  bool is_calibrated;
};

CalibrationData calData;

// Sensor fusion variables
float roll, pitch, yaw;
float deltat = 0.0f;
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // quaternion
float eInt[3] = {0.0f, 0.0f, 0.0f};       // integral error
unsigned long lastUpdate = 0;
float Now = 0;

// PID variables with better defaults
float Kp = 45.0;   // Proportional gain
float Ki = 2.5;    // Integral gain
float Kd = 2.8;    // Derivative gain
float targetAngle = 0.0;
float lastError = 0;
float errorSum = 0;
float deadband = 3.0;
float motorScale = 3.0;
bool isBalancing = false;

// Add motor direction variables
bool invertLeftMotor = false;
bool invertRightMotor = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize PCA9685 instances
  pca1.begin();
  pca1.setPWMFreq(SERVO_FREQ);
  delay(10);
  pca2.begin();
  pca2.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize MPU6050
  if (!initMPU()) {
    Serial.println("MPU6050 initialization failed!");
    while (1) {
      delay(10);
    }
  }

  // Load calibration data
  loadCalibration();

  // Initialize motor pins
  pinMode(PWM1_PIN, OUTPUT);
  pinMode(PWM2_PIN, OUTPUT);
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);

  // Stop motors initially
  analogWrite(PWM1_PIN, 0);
  analogWrite(PWM2_PIN, 0);
  digitalWrite(DIR1_PIN, LOW);
  digitalWrite(DIR2_PIN, LOW);

  Serial.println("System Ready!");
}

bool initMPU() {
  Wire.begin();
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    return false;
  }
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  return true;
}

void loadCalibration() {
  // Read calibration data from EEPROM
  EEPROM.get(0, calData);
  
  if (calData.is_calibrated) {
    // Note: MPU6050 doesn't support direct bias setting through library
    // We'll apply the calibration in the reading functions
    
    // Load motor direction configuration
    EEPROM.get(sizeof(CalibrationData), invertLeftMotor);
    EEPROM.get(sizeof(CalibrationData) + sizeof(bool), invertRightMotor);
    
    Serial.println("Loaded calibration data");
    Serial.print("Left motor inverted: "); Serial.println(invertLeftMotor);
    Serial.print("Right motor inverted: "); Serial.println(invertRightMotor);
  } else {
    Serial.println("No calibration data found");
  }
}

void calibrateMPU() {
  Serial.println("Calibrating MPU6050...");
  Serial.println("Keep the device still!");
  delay(2000);

  // Collect gyro and accel data for 1 second
  float gyro_temp[3] = {0, 0, 0};
  float accel_temp[3] = {0, 0, 0};
  int samples = 0;

  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    sensors_event_t a, g, temp;
    if (mpu.getEvent(&a, &g, &temp)) {
      gyro_temp[0] += g.gyro.x;
      gyro_temp[1] += g.gyro.y;
      gyro_temp[2] += g.gyro.z;
      accel_temp[0] += a.acceleration.x;
      accel_temp[1] += a.acceleration.y;
      accel_temp[2] += a.acceleration.z;
      samples++;
    }
  }

  // Calculate average bias
  for (int i = 0; i < 3; i++) {
    calData.gyro_bias[i] = gyro_temp[i] / samples;
    calData.accel_bias[i] = accel_temp[i] / samples;
  }
  
  // Remove gravity from Z axis
  calData.accel_bias[2] -= 9.81;

  calData.is_calibrated = true;
  
  // Save calibration to EEPROM
  EEPROM.put(0, calData);
  
  Serial.println("Sensor calibration complete!");
  
  // Now calibrate motor directions
  Serial.println("Starting motor direction calibration...");
  Serial.println("Robot will tilt forward slightly.");
  calibrateMotorDirections();
}

void calibrateMotorDirections() {
  // First, ensure we're reading angles correctly
  updateAngles();
  float initialPitch = pitch;
  
  // Test left motor
  Serial.println("Testing left motor...");
  setMotorSpeed("LDC", 50);  // Apply small forward power
  delay(500);  // Wait for movement
  updateAngles();
  
  // If pitch increased (tilted backward) when applying forward power, motor is reversed
  invertLeftMotor = (pitch > initialPitch);
  setMotorSpeed("LDC", 0);
  
  // Reset position
  delay(1000);
  updateAngles();
  initialPitch = pitch;
  
  // Test right motor
  Serial.println("Testing right motor...");
  setMotorSpeed("RDC", 50);  // Apply small forward power
  delay(500);  // Wait for movement
  updateAngles();
  
  // If pitch increased when applying forward power, motor is reversed
  invertRightMotor = (pitch > initialPitch);
  setMotorSpeed("RDC", 0);
  
  // Save motor direction configuration to EEPROM
  EEPROM.put(sizeof(CalibrationData), invertLeftMotor);
  EEPROM.put(sizeof(CalibrationData) + sizeof(bool), invertRightMotor);
  
  Serial.println("Motor direction calibration complete!");
  Serial.print("Left motor inverted: "); Serial.println(invertLeftMotor);
  Serial.print("Right motor inverted: "); Serial.println(invertRightMotor);
}

void updateAngles() {
  sensors_event_t a, g, temp;
  if (mpu.getEvent(&a, &g, &temp)) {
    // Get delta time
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;

    // Apply calibration if available
    float ax = a.acceleration.x - (calData.is_calibrated ? calData.accel_bias[0] : 0);
    float ay = a.acceleration.y - (calData.is_calibrated ? calData.accel_bias[1] : 0);
    float az = a.acceleration.z - (calData.is_calibrated ? calData.accel_bias[2] : 0);
    float gx = g.gyro.x - (calData.is_calibrated ? calData.gyro_bias[0] : 0);
    float gy = g.gyro.y - (calData.is_calibrated ? calData.gyro_bias[1] : 0);
    float gz = g.gyro.z - (calData.is_calibrated ? calData.gyro_bias[2] : 0);

    // Apply Madgwick filter (without magnetometer)
    float recipNorm;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q1, _2q2, _2q3, _2q4;
    float _4q1, _4q2, _4q3;
    float _8q2, _8q3;
    float q1q1, q2q2, q3q3, q4q4;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
    qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
    qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
    qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

    // Compute feedback only if accelerometer measurement valid
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
      // Normalise accelerometer measurement
      recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
      ax *= recipNorm;
      ay *= recipNorm;
      az *= recipNorm;

      // Auxiliary variables to avoid repeated arithmetic
      _2q1 = 2.0f * q[0];
      _2q2 = 2.0f * q[1];
      _2q3 = 2.0f * q[2];
      _2q4 = 2.0f * q[3];
      _4q1 = 4.0f * q[0];
      _4q2 = 4.0f * q[1];
      _4q3 = 4.0f * q[2];
      _8q2 = 8.0f * q[1];
      _8q3 = 8.0f * q[2];
      q1q1 = q[0] * q[0];
      q2q2 = q[1] * q[1];
      q3q3 = q[2] * q[2];
      q4q4 = q[3] * q[3];

      // Gradient decent algorithm corrective step
      s1 = _4q1 * q3q3 + _2q3 * ax + _4q1 * q2q2 - _2q2 * ay;
      s2 = _4q2 * q4q4 - _2q4 * ax + 4.0f * q1q1 * q[1] - _2q1 * ay - _4q2 + _8q2 * q2q2 + _8q2 * q3q3 + _4q2 * az;
      s3 = 4.0f * q1q1 * q[2] + _2q1 * ax + _4q3 * q4q4 - _2q4 * ay - _4q3 + _8q3 * q2q2 + _8q3 * q3q3 + _4q3 * az;
      s4 = 4.0f * q2q2 * q[3] - _2q2 * ax + 4.0f * q3q3 * q[3] - _2q3 * ay;

      // Normalise step magnitude
      recipNorm = 1.0f / sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);
      s1 *= recipNorm;
      s2 *= recipNorm;
      s3 *= recipNorm;
      s4 *= recipNorm;

      // Apply feedback step
      qDot1 -= beta * s1;
      qDot2 -= beta * s2;
      qDot3 -= beta * s3;
      qDot4 -= beta * s4;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q[0] += qDot1 * deltat;
    q[1] += qDot2 * deltat;
    q[2] += qDot3 * deltat;
    q[3] += qDot4 * deltat;

    // Normalise quaternion
    recipNorm = 1.0f / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    q[0] *= recipNorm;
    q[1] *= recipNorm;
    q[2] *= recipNorm;
    q[3] *= recipNorm;

    // Convert quaternion to Euler angles
    roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), 1.0f - 2.0f * (q[1] * q[1] + q[2] * q[2]));
    pitch = asin(2.0f * (q[0] * q[2] - q[3] * q[1]));
    yaw = atan2(2.0f * (q[0] * q[3] + q[1] * q[2]), 1.0f - 2.0f * (q[2] * q[2] + q[3] * q[3]));

    // Convert to degrees
    roll *= RAD_TO_DEG;
    pitch *= RAD_TO_DEG;
    yaw *= RAD_TO_DEG;
  }
}

void updateBalancing() {
  // Calculate PID
  float error = targetAngle - pitch;
  
  // Only update if error is outside deadband
  if (abs(error) > deadband) {
    errorSum = constrain(errorSum + error * deltat, -100, 100);  // Integral with anti-windup
    float dError = (error - lastError) / deltat;
    
    // Calculate motor power using PID
    float motorPower = (Kp * error) + (Ki * errorSum) + (Kd * dError);
    
    // Scale and constrain motor power
    int motorSpeed = constrain(motorPower * motorScale, -255, 255);
    
    // Set motor speeds with direction
    setMotorSpeed("LDC", motorSpeed);
    setMotorSpeed("RDC", motorSpeed);
    
    lastError = error;
  } else {
    // Within deadband, stop motors
    setMotorSpeed("LDC", 0);
    setMotorSpeed("RDC", 0);
    errorSum = 0;  // Reset integral term
  }
}

void loop() {
  // Update sensor fusion
  updateAngles();
  
  // Update balancing if enabled
  if (isBalancing) {
    updateBalancing();
  }
  
  // Handle serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseAndExecuteCommand(input);
  }
  
  // Print debug info every 100ms
  static unsigned long printTimer = 0;
  if ((millis() - printTimer) > 100) {
    printTimer = millis();
    printDebugInfo();
  }
}

void printDebugInfo() {
  Serial.print("Roll: "); Serial.print(roll);
  Serial.print(" Pitch: "); Serial.print(pitch);
  Serial.print(" Yaw: "); Serial.print(yaw);
  if (isBalancing) {
    Serial.print(" Error: "); Serial.print(lastError);
    Serial.print(" Sum: "); Serial.print(errorSum);
  }
  Serial.println();
}

void parseAndExecuteCommand(String input) {
  input.trim();
  
  if (input.startsWith("BALANCE:")) {
    String value = input.substring(8);
    if (value == "ON") {
      isBalancing = true;
      errorSum = 0;  // Reset integral term
      Serial.println("Balancing enabled");
    } else if (value == "OFF") {
      isBalancing = false;
      setMotorSpeed("LDC", 0);
      setMotorSpeed("RDC", 0);
      Serial.println("Balancing disabled");
    }
  }
  else if (input.startsWith("CALIBRATE")) {
    calibrateMPU();
  }
  else if (input.startsWith("PARAM:")) {
    updateParameter(input.substring(6));
  }
  else {
    parseAndSetServos(input);
  }
}

void updateParameter(String paramData) {
  int colonIndex = paramData.indexOf(':');
  if (colonIndex > 0) {
    String param = paramData.substring(0, colonIndex);
    float value = paramData.substring(colonIndex + 1).toFloat();
    
    if (param == "Kp") Kp = value;
    else if (param == "Ki") Ki = value;
    else if (param == "Kd") Kd = value;
    else if (param == "target_angle") targetAngle = value;
    else if (param == "deadband") deadband = value;
    else if (param == "motor_scale") motorScale = value;
    
    Serial.printf("Updated %s to %.2f\n", param.c_str(), value);
  }
}

void parseAndSetServos(String input) {
  if (input.length() == 0) return;

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
    if (colonIdx == -1) continue;

    String key = pairs[i].substring(0, colonIdx);
    int value = pairs[i].substring(colonIdx + 1).toInt();

    if (key == "LDC" || key == "RDC") {
      if (!isBalancing) {
        setMotorSpeed(key, value);
      }
    } else {
      setServoPosition(key, value);
    }
  }
}

void setServoPosition(String key, int degrees) {
  int pulse = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  if (key.startsWith("L")) {
    int servoNum = key.substring(1).toInt() - 1;
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        pca1.setPin(servoNum, 0, false);
      } else {
        pca1.setPWM(servoNum, 0, pulse);
      }
    }
  } else if (key.startsWith("R")) {
    int servoNum = key.substring(1).toInt() - 1;
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        pca2.setPin(servoNum, 0, false);
      } else {
        pca2.setPWM(servoNum, 0, pulse);
      }
    }
  }
}

void setMotorSpeed(String motor, int speed) {
  speed = constrain(speed, -255, 255);
  
  if (motor == "LDC") {
    // Apply inversion if needed
    if (invertLeftMotor) speed = -speed;
    digitalWrite(DIR1_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM1_PIN, abs(speed));
  } else if (motor == "RDC") {
    // Apply inversion if needed
    if (invertRightMotor) speed = -speed;
    digitalWrite(DIR2_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM2_PIN, abs(speed));
  }
} 