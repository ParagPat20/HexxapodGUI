#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// Create instances
Adafruit_PWMServoDriver pca1 = Adafruit_PWMServoDriver(0x40);  // Default I2C address
Adafruit_PWMServoDriver pca2 = Adafruit_PWMServoDriver(0x42);  // Second PCA9685
Adafruit_MPU6050 mpu;

// Servo settings
#define SERVO_FREQ 50      // 50 Hz update rate for servos
#define SERVOMIN  125      // Minimum pulse length count (out of 4096)
#define SERVOMAX  575      // Maximum pulse length count (out of 4096)
#define USMIN  600         // microsecond range for servos
#define USMAX  2400

// Cytron Motor Control Pins
#define PWM1_PIN 18
#define PWM2_PIN 33
#define DIR1_PIN 19
#define DIR2_PIN 32

// PID Constants for balancing
float Kp = 30.0;   // Proportional gain
float Ki = 1.5;    // Integral gain
float Kd = 1.2;    // Derivative gain
float targetAngle = 0.0;  // Target angle for balancing (0 = upright)
float deadband = 2.0;     // Deadband for angle control
float motorScale = 2.5;   // Motor power scaling
float compFilterAlpha = 0.96; // Complementary filter coefficient

// MPU6050 variables
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float angleX, angleY;
float gyroXangle, gyroYangle;
float compAngleX, compAngleY;
float timer;
uint32_t timer1;

// Variables for PID control
float error = 0;
float lastError = 0;
float integral = 0;
float derivative = 0;
unsigned long lastTime = 0;
float deltaTime = 0;
float motorPower = 0;

// Motor control variables
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
bool isBalancing = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000); // Fast I2C clock

  // Initialize PCA9685 instances
  pca1.begin();
  pca1.setPWMFreq(SERVO_FREQ);
  delay(10);
  pca2.begin();
  pca2.setPWMFreq(SERVO_FREQ);
  delay(10);

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Setup MPU6050 with better configuration
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  
  // Initialize angles
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  angleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  angleY = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + 
                    a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

  gyroXangle = angleX;
  gyroYangle = angleY;
  compAngleX = angleX;
  compAngleY = angleY;

  timer = micros();

  // Initialize Cytron motor control pins
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

void loop() {
  // Update MPU readings with complementary filter
  updateMPU();
  
  // Calculate PID if balancing is enabled
  if (isBalancing) {
    updateBalancing();
  }

  // Check for incoming serial commands
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    parseAndExecuteCommand(input);
  }

  // Print debug info every 100ms
  static uint32_t printTimer;
  if ((millis() - printTimer) > 100) {
    printTimer = millis();
    printDebugInfo();
  }
}

void updateMPU() {
  // Read raw sensor data
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  // Calculate delta time
  float dt = (float)(micros() - timer) / 1000000;
  timer = micros();

  // Calculate angles using accelerometer
  float accelAngleX = atan2(a.acceleration.y, a.acceleration.z) * RAD_TO_DEG;
  float accelAngleY = atan(-a.acceleration.x / sqrt(a.acceleration.y * a.acceleration.y + 
                          a.acceleration.z * a.acceleration.z)) * RAD_TO_DEG;

  // Integrate gyroscope data
  gyroXangle += g.gyro.x * dt;
  gyroYangle += g.gyro.y * dt;

  // Complementary filter
  compAngleX = compFilterAlpha * (compAngleX + g.gyro.x * dt) + (1 - compFilterAlpha) * accelAngleX;
  compAngleY = compFilterAlpha * (compAngleY + g.gyro.y * dt) + (1 - compFilterAlpha) * accelAngleY;

  // Save raw values for debugging
  accX = a.acceleration.x;
  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

void updateBalancing() {
  unsigned long currentTime = millis();
  deltaTime = (currentTime - lastTime) / 1000.0;  // Convert to seconds
  
  // Skip if time delta is too large
  if (deltaTime > 0.1) {
    lastTime = currentTime;
    return;
  }

  // Use filtered angle for better stability
  float currentAngle = compAngleY;  // Using Y angle for forward/backward balance

  // PID calculations with improved derivative
  error = targetAngle - currentAngle;
  
  // Only update if error is outside deadband
  if (abs(error) > deadband) {
    integral = constrain(integral + (error * deltaTime), -100, 100);  // Anti-windup
    derivative = (error - lastError) / deltaTime;
    
    // Calculate motor power using PID
    motorPower = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Scale and constrain motor power
    int motorSpeed = constrain(motorPower * motorScale, -255, 255);
    
    // Set motor speeds
    setMotorSpeed("LDC", motorSpeed);
    setMotorSpeed("RDC", motorSpeed);
  } else {
    // Within deadband, stop motors
    setMotorSpeed("LDC", 0);
    setMotorSpeed("RDC", 0);
    integral = 0;  // Reset integral term
  }
  
  // Save for next iteration
  lastError = error;
  lastTime = currentTime;
}

void printDebugInfo() {
  Serial.print("Angle X: "); Serial.print(angleX);
  Serial.print(" Y: "); Serial.print(angleY);
  Serial.print(" Error: "); Serial.print(error);
  Serial.print(" Power: "); Serial.print(motorPower);
  Serial.print(" Speed L: "); Serial.print(leftMotorSpeed);
  Serial.print(" R: "); Serial.println(rightMotorSpeed);
}

void parseAndExecuteCommand(String input) {
  input.trim();
  
  if (input.startsWith("BALANCE:")) {
    // Handle balance command
    String value = input.substring(8);
    if (value == "ON") {
      isBalancing = true;
      Serial.println("Balancing enabled");
    } else if (value == "OFF") {
      isBalancing = false;
      setMotorSpeed("LDC", 0);
      setMotorSpeed("RDC", 0);
      Serial.println("Balancing disabled");
    }
    return;
  }
  
  if (input.startsWith("PARAM:")) {
    // Handle parameter updates
    String paramData = input.substring(6);
    int colonIndex = paramData.indexOf(':');
    if (colonIndex > 0) {
      String param = paramData.substring(0, colonIndex);
      float value = paramData.substring(colonIndex + 1).toFloat();
      
      // Update parameters
      if (param == "Kp") {
        Kp = value;
        Serial.printf("Updated Kp to %.2f\n", value);
      }
      else if (param == "Ki") {
        Ki = value;
        Serial.printf("Updated Ki to %.2f\n", value);
      }
      else if (param == "Kd") {
        Kd = value;
        Serial.printf("Updated Kd to %.2f\n", value);
      }
      else if (param == "target_angle") {
        targetAngle = value;
        Serial.printf("Updated target angle to %.2f\n", value);
      }
      else if (param == "deadband") {
        deadband = value;
        Serial.printf("Updated deadband to %.2f\n", value);
      }
      else if (param == "motor_scale") {
        motorScale = value;
        Serial.printf("Updated motor scale to %.2f\n", value);
      }
      else if (param == "comp_filter_alpha") {
        compFilterAlpha = value;
        Serial.printf("Updated comp filter alpha to %.2f\n", value);
      }
    }
    return;
  }
  
  // Handle regular servo and motor commands
  parseAndSetServos(input);
}

void parseAndSetServos(String input) {
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
      continue;
    }

    String key = pairs[i].substring(0, colonIdx);
    int value = pairs[i].substring(colonIdx + 1).toInt();

    if (key == "LDC" || key == "RDC") {
      if (!isBalancing) {  // Only allow direct motor control when not balancing
        setMotorSpeed(key, value);
      }
    } else {
      setServoPosition(key, value);
    }
  }
}

void setServoPosition(String key, int degrees) {
  // Map degrees to pulse length
  int pulse = map(degrees, 0, 180, SERVOMIN, SERVOMAX);

  if (key.startsWith("L")) {
    int servoNum = key.substring(1).toInt() - 1;
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        pca1.setPin(servoNum, 0, false);
      } else {
        pca1.setPWM(servoNum, 0, pulse);
      }
      Serial.printf("Left servo %d set to %d degrees\n", servoNum + 1, degrees);
    }
  } else if (key.startsWith("R")) {
    int servoNum = key.substring(1).toInt() - 1;
    if (servoNum >= 0 && servoNum < 16) {
      if (degrees == 181) {
        pca2.setPin(servoNum, 0, false);
      } else {
        pca2.setPWM(servoNum, 0, pulse);
      }
      Serial.printf("Right servo %d set to %d degrees\n", servoNum + 1, degrees);
    }
  }
}

void setMotorSpeed(String motor, int speed) {
  speed = constrain(speed, -255, 255);
  
  if (motor == "LDC") {
    digitalWrite(DIR1_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM1_PIN, abs(speed));
    leftMotorSpeed = speed;
  } else if (motor == "RDC") {
    digitalWrite(DIR2_PIN, speed >= 0 ? HIGH : LOW);
    analogWrite(PWM2_PIN, abs(speed));
    rightMotorSpeed = speed;
  }
  
  Serial.printf("%s motor speed set to %d\n", motor.c_str(), speed);
} 