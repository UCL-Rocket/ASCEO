/******************************************************************************
Gyroscope-Controlled Motor
Reads Z-axis rotation from BMI270 gyroscope and controls motor in opposite direction
with encoder feedback for precise 1:1 rotation control.

Connections:
TB6612FNG:
- AIN1 -> D6
- AIN2 -> D5
- PWMA -> D8
- STBY -> D7
- VM -> Motor power (6V)
- VCC -> Arduino 3V3
- GND -> Arduino GND

Encoder (341.2 PPR):
- Channel A -> D3 (with interrupt)
- Channel B -> D4 (shared with AIN2)

******************************************************************************/

#include <SparkFun_TB6612.h>
#include "Arduino_BMI270_BMM150.h"

// Motor control pins
#define AIN1 6
#define AIN2 5
#define PWMA 8
#define STBY 7

// Encoder pins
#define ENCODER_A 3
#define ENCODER_B 4

// Encoder constants (341.2 PPR with 11x Hall resolution)
const float PPR = 341.2;  
const float DEGREES_PER_PULSE = 360.0 / PPR;

// Encoder variables
volatile long encoderCount = 0;
bool lastEncA = 0;
bool lastEncB = 0;

// Gyroscope variables
float gyroZ = 0;           // Z-axis rotation in degrees/second
float targetRotation = 0;   // Target rotation in degrees
float currentRotation = 0;  // Current rotation based on encoder

// Motor control variables
const int offsetA = 1;    
int motorSpeed = 0;
const int MIN_SPEED = 70;  
const int MAX_SPEED = 255;
const float Kp = 2.0;      // Proportional gain for PID control

// Timing variables
unsigned long lastGyroUpdate = 0;
unsigned long lastEncoderUpdate = 0;
const unsigned long GYRO_UPDATE_MS = 10;  // 100Hz update rate

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

void setup() {
  Serial.begin(115200);
  while (!Serial);  
  Serial.println("Starting...");
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialise IMU!");
    while (1);
  }
  
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");
  
  // Setup encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Attach interrupt for encoder channel A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  
  // Read initial encoder states
  lastEncA = digitalRead(ENCODER_A);
  lastEncB = digitalRead(ENCODER_B);
  
  Serial.println("System Ready");
  Serial.println("Z Gyro (deg/s)\tTarget Rot(deg)\tActual Rot(deg)\tMotor Speed");
  Serial.println("--------------------------------------------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Update gyroscope reading every GYRO_UPDATE_MS
  if (currentTime - lastGyroUpdate >= GYRO_UPDATE_MS) {
    updateGyroscope();
    lastGyroUpdate = currentTime;
  }

  // Update current rotation from encoder
  updateCurrentRotation();
  
  controlMotor();
  
  // debugging
  static unsigned long lastPrint = 0;
  if (currentTime - lastPrint >= 100) {
    printStatus();
    lastPrint = currentTime;
  }
}

void updateGyroscope() {
  float x, y, z;
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    gyroZ = z;  // We only care about Z-axis rotation
    
    // Update target rotation based on gyroscope reading
    float deltaTime = GYRO_UPDATE_MS / 1000.0;  // Convert ms to s
    targetRotation -= gyroZ * deltaTime;  // Opposite direction 
    
    // Keep target rotation within reasonable bounds
    if (targetRotation > 360.0) targetRotation -= 360.0;
    if (targetRotation < -360.0) targetRotation += 360.0;
  }
}

void updateCurrentRotation() {
  currentRotation = encoderCount * DEGREES_PER_PULSE;
}

void controlMotor() {
  float error = targetRotation + currentRotation; //Since they're opposite direcitons
  
  // Prop control
  motorSpeed = Kp * error;
  
  if (abs(motorSpeed) < MIN_SPEED) {
    motorSpeed = (motorSpeed >= 0) ? MIN_SPEED : -MIN_SPEED;
  }
  
  if (motorSpeed > MAX_SPEED) motorSpeed = MAX_SPEED;
  if (motorSpeed < -MAX_SPEED) motorSpeed = -MAX_SPEED;
  
  // Apply motor control
  if (abs(error) > 0.5) {  // Deadband of 0.5 degrees
    motor1.drive(motorSpeed);
  } else {
    motor1.brake();
    motorSpeed = 0;
  }
}

void printStatus() {
  Serial.print(gyroZ, 1);
  Serial.print("\t\t");
  Serial.print(targetRotation, 1);
  Serial.print("\t\t");
  Serial.print(currentRotation, 1);
  Serial.print("\t\t");
  Serial.println(motorSpeed);
}

// Encoder interrupt service routine
void updateEncoder() {
  bool encA = digitalRead(ENCODER_A);
  bool encB = digitalRead(ENCODER_B);
  
  // Determine direction based on quadrature encoding
  if (encA != lastEncA) {
    if (encA != encB) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  } else if (encB != lastEncB) {
    if (encA == encB) {
      encoderCount++;
    } else {
      encoderCount--;
    }
  }
  
  lastEncA = encA;
  lastEncB = encB;
  
  // Update timestamp for non-interrupt context
  lastEncoderUpdate = millis();
}

// Reset
void resetSystem() {
  encoderCount = 0;
  targetRotation = 0;
  currentRotation = 0;
  gyroZ = 0;
  motor1.brake();
}