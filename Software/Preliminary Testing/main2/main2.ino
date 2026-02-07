/******************************************************************************
Gyroscope-Controlled Motor with PID Control
Simplified Serial Plotter Output: Target vs Current Rotation
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

// Control variables
float gyroZ = 0;           // Z-axis rotation rate in degrees/second
float targetAngle = 0;     // Target rotation angle (degrees)
float currentAngle = 0;    // Current rotation from encoder (degrees)
float pidError = 0;        // Error between target and current
float lastError = 0;       // Previous error for derivative
float integral = 0;        // Integral accumulator
float derivative = 0;      // Derivative term

// PID Gains
float Kp = 2.0;           // Proportional gain
float Ki = 0.05;          // Integral gain  
float Kd = 0.1;           // Derivative gain

// Motor control
const int offsetA = 1;    
int motorSpeed = 0;
const int MIN_SPEED = 70;  
const int MAX_SPEED = 255;
const float INTEGRAL_LIMIT = 100.0;  // Anti-windup

// Timing
unsigned long lastLoopTime = 0;
float deltaTime = 0.01;    // 100Hz = 0.01s

// Plotting
unsigned long lastPlotTime = 0;
const unsigned long PLOT_INTERVAL_MS = 20;  // 50Hz plotting

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

void setup() {
  Serial.begin(115200);
  while (!Serial);  
  
  Serial.println("Gyro-Controlled Motor - Target vs Current Angle");
  Serial.println("===============================================");
  
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
  
  // Initialize timing
  lastLoopTime = millis();
  
  // Print plotter labels (Arduino Serial Plotter format)
  Serial.println("TargetAngle,CurrentAngle");
  Serial.println("========================");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Calculate actual delta time (in seconds)
  deltaTime = (currentTime - lastLoopTime) / 1000.0;
  
  // Update at approximately 100Hz
  if (deltaTime >= 0.01) {  // 10ms = 100Hz
    lastLoopTime = currentTime;
    
    // 1. Read sensors
    readGyroscope();
    updateCurrentAngle();
    
    // 2. Update target based on gyro (integrate rate to get angle)
    targetAngle -= gyroZ * deltaTime;  // Move opposite to gyro
    
    // Keep target angle within ±360 degrees for readability
    if (targetAngle > 360.0) targetAngle -= 360.0;
    if (targetAngle < -360.0) targetAngle += 360.0;
    
    // 3. Run PID control
    runPID();
    
    // 4. Apply motor control
    applyMotorControl();
    
    // 5. Send data to Serial Plotter
    if (currentTime - lastPlotTime >= PLOT_INTERVAL_MS) {
      sendToPlotter();
      lastPlotTime = currentTime;
    }
  }
}

void readGyroscope() {
  float x, y, z;
  
  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);
    gyroZ = z;  // Z-axis rotation rate in deg/s
  }
}

void updateCurrentAngle() {
  // Safely read encoder count
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();
  
  currentAngle = currentCount * DEGREES_PER_PULSE;
  
  // Keep current angle within ±360 degrees for readability
  if (currentAngle > 360.0) {
    currentAngle -= 360.0;
    // Adjust encoder count to match
    noInterrupts();
    encoderCount = (long)(currentAngle / DEGREES_PER_PULSE);
    interrupts();
  }
  if (currentAngle < -360.0) {
    currentAngle += 360.0;
    noInterrupts();
    encoderCount = (long)(currentAngle / DEGREES_PER_PULSE);
    interrupts();
  }
}

void runPID() {
  // Calculate error (target - current)
  pidError = targetAngle + currentAngle;
  
  // Proportional term
  float P = Kp * pidError;
  
  // Integral term with anti-windup
  integral += pidError * deltaTime;
  
  // Limit integral term to prevent windup
  if (integral > INTEGRAL_LIMIT) integral = INTEGRAL_LIMIT;
  if (integral < -INTEGRAL_LIMIT) integral = -INTEGRAL_LIMIT;
  
  float I = Ki * integral;
  
  // Derivative term (rate of change of error)
  derivative = (pidError - lastError) / deltaTime;
  float D = Kd * derivative;
  
  // Calculate PID output
  float pidOutput = P + I + D;
  
  // Convert to motor speed
  motorSpeed = constrain((int)pidOutput, -MAX_SPEED, MAX_SPEED);
  
  // Store error for next derivative calculation
  lastError = pidError;
}

void applyMotorControl() {
  // Deadband and minimum speed handling
  if (abs(motorSpeed) < MIN_SPEED && motorSpeed != 0) {
    motorSpeed = (motorSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
  }
  
  // Apply motor control
  if (abs(pidError) > 0.5) {  // Deadband of 0.5 degrees
    motor1.drive(motorSpeed);
  } else {
    motor1.brake();
    motorSpeed = 0;
    // Reset integral when at target (prevents windup)
    integral = 0;
  }
}

void sendToPlotter() {
  // Send only target and current angle for clean plotting
  Serial.print(targetAngle);
  Serial.print(",");
  Serial.println(currentAngle);
  
  // Alternative format with labels (comment out above and uncomment below for labeled output):
  /*
  Serial.print("Target:");
  Serial.print(targetAngle);
  Serial.print(",Current:");
  Serial.println(currentAngle);
  */
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
}

// Optional debug output (every 2 seconds)
void debugOutput() {
  static unsigned long lastDebugTime = 0;
  unsigned long currentTime = millis();
  
  if (currentTime - lastDebugTime >= 2000) {
    Serial.print("[DEBUG] Target: ");
    Serial.print(targetAngle, 1);
    Serial.print("°, Current: ");
    Serial.print(currentAngle, 1);
    Serial.print("°, Error: ");
    Serial.print(pidError, 1);
    Serial.print("°, Motor: ");
    Serial.println(motorSpeed);
    
    lastDebugTime = currentTime;
  }
}