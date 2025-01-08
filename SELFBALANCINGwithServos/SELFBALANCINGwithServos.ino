/*
References:
1. https://github.com/bolderflight/MPU9250 - MPU9250 library with calibration
2. https://github.com/kriswiner/MPU9250 - Advanced sensor fusion
3. https://github.com/hideakitai/MPU9250 - DMP support
4. https://github.com/rpicopter/ArduinoMotionSensor - Motion processing
*/

#include <Wire.h>
#include <MPU9250.h>
#include <Adafruit_PWMServoDriver.h>
#include <EEPROM.h>

// Create instances
MPU9250 mpu;
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

// MPU9250 calibration data structure
struct CalibrationData {
  float accel_bias[3];
  float gyro_bias[3];
  float mag_bias[3];
  float mag_scale[3];
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

  // Initialize MPU9250
  if (!initMPU()) {
    Serial.println("MPU9250 initialization failed!");
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
  mpu.setup(0x68);  // Initialize with I2C address 0x68
  
  // Set up configuration
  mpu.setAccBias(0, 0, 0);
  mpu.setGyroBias(0, 0, 0);
  mpu.setMagBias(0, 0, 0);
  
  return true;
}

void loadCalibration() {
  // Read calibration data from EEPROM
  EEPROM.get(0, calData);
  
  if (calData.is_calibrated) {
    // Apply calibration data
    mpu.setAccBias(calData.accel_bias[0], calData.accel_bias[1], calData.accel_bias[2]);
    mpu.setGyroBias(calData.gyro_bias[0], calData.gyro_bias[1], calData.gyro_bias[2]);
    mpu.setMagBias(calData.mag_bias[0], calData.mag_bias[1], calData.mag_bias[2]);
    
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
  Serial.println("Calibrating MPU9250...");
  Serial.println("Keep the device still!");
  delay(2000);

  // Collect gyro and accel data for 1 second
  float gyro_temp[3] = {0, 0, 0};
  float accel_temp[3] = {0, 0, 0};
  int samples = 0;

  unsigned long startTime = millis();
  while ((millis() - startTime) < 1000) {
    if (mpu.update()) {
      gyro_temp[0] += mpu.getGyroX();
      gyro_temp[1] += mpu.getGyroY();
      gyro_temp[2] += mpu.getGyroZ();
      accel_temp[0] += mpu.getAccX();
      accel_temp[1] += mpu.getAccY();
      accel_temp[2] += mpu.getAccZ();
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

  // Calibrate magnetometer
  Serial.println("Rotate device in figure-8 pattern for 30 seconds");
  delay(2000);

  float mag_max[3] = {-32767, -32767, -32767};
  float mag_min[3] = {32767, 32767, 32767};
  
  startTime = millis();
  while ((millis() - startTime) < 30000) {
    if (mpu.update()) {
      float magX = mpu.getMagX();
      float magY = mpu.getMagY();
      float magZ = mpu.getMagZ();
      
      mag_max[0] = max(mag_max[0], magX);
      mag_min[0] = min(mag_min[0], magX);
      mag_max[1] = max(mag_max[1], magY);
      mag_min[1] = min(mag_min[1], magY);
      mag_max[2] = max(mag_max[2], magZ);
      mag_min[2] = min(mag_min[2], magZ);
    }
  }

  // Calculate mag bias and scale
  for (int i = 0; i < 3; i++) {
    calData.mag_bias[i] = (mag_max[i] + mag_min[i]) / 2;
    calData.mag_scale[i] = (mag_max[i] - mag_min[i]) / 2;
  }

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

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void updateAngles() {
  if (mpu.update()) {
    // Get delta time
    Now = micros();
    deltat = ((Now - lastUpdate) / 1000000.0f);
    lastUpdate = Now;

    // Get sensor data
    float ax = mpu.getAccX();
    float ay = mpu.getAccY();
    float az = mpu.getAccZ();
    float gx = mpu.getGyroX();
    float gy = mpu.getGyroY();
    float gz = mpu.getGyroZ();
    float mx = mpu.getMagX();
    float my = mpu.getMagY();
    float mz = mpu.getMagZ();

    // Apply Madgwick filter
    MadgwickQuaternionUpdate(ax, ay, az, gx, gy, gz, mx, my, mz);

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