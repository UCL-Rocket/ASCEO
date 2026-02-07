/******************************************************************************
Gyroscope-Controlled Motor with Simulink Interface - CONTINUOUS STREAMING
Sends gyro and encoder data continuously, receives motor commands asynchronously
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

struct TelemetryPacket {
  float gyroZ_angle;        // Integrated gyro Z angle
  float encoder_angle;      // Encoder-based rotation
};


// Encoder constants
const float PPR = 341.2;  
const float DEGREES_PER_PULSE = 360.0 / PPR;

// Encoder variables
volatile long encoderCount = 0;
bool lastEncA = 0;
bool lastEncB = 0;

// Sensor variables
float gyroZ = 0;           // Z-axis rotation in degrees/second
float currentRotation = 0;  // Current rotation from encoder
float gyroZ_rate = 0.0f;        // deg/s
float gyroZ_angle = 0.0f;       // deg (integrated)
unsigned long lastGyroTime = 0;

// Motor control
const int offsetA = 1;
int motorSpeed = 0;        // Will be set by Simulink
const int MIN_SPEED = 70;
const int MAX_SPEED = 255;

// Streaming variables
unsigned long lastSendTime = 0;
const unsigned long SEND_INTERVAL_MS = 10;  // 100Hz streaming
bool newGyroData = false;
bool newEncoderData = false;

// Circular buffer for serial reception (non-blocking)
const int RX_BUFFER_SIZE = 32;
char rxBuffer[RX_BUFFER_SIZE];
int rxIndex = 0;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);


void setup() {
  Serial.begin(115200);
  while (!Serial);  
  Serial.println("Simulink Continuous Streaming Interface Ready");
  
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  // Setup encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Attach interrupt for encoder
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  
  // Read initial encoder states
  lastEncA = digitalRead(ENCODER_A);
  lastEncB = digitalRead(ENCODER_B);
  
  // Clear RX buffer
  memset(rxBuffer, 0, RX_BUFFER_SIZE);
  
  Serial.println("Streaming data to Simulink...");
  Serial.println("Format: G:gyro,E:encoder");
}

void loop() {
  unsigned long currentTime = millis();
  checkForCommands();
  updateGyroscope();
  updateCurrentRotation();
  
  // 3. Stream data at fixed interval (10ms = 100Hz)
  if (currentTime - lastSendTime >= SEND_INTERVAL_MS) {
    streamDataToSimulink();
    lastSendTime = currentTime;
  }
  
  // 4. Apply motor command (updated whenever new command arrives)
  applyMotorCommand();
}

// Read gyro continuously - sets flag when new data available
void updateGyroscope() {
  float x, y, z;

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x, y, z);

    unsigned long now = millis();
    if (lastGyroTime > 0) {
      float dt = (now - lastGyroTime) * 0.001f; 
      gyroZ_rate = z;                            // deg/s
      gyroZ_angle += gyroZ_rate * dt;            // integrate
    }
    lastGyroTime = now;
  }
}


// Update encoder angle from interrupt-modified count
void updateCurrentRotation() {
  static long lastEncoderCount = 0;
  
  // Check if encoder changed (protected read in main loop)
  noInterrupts();
  long currentCount = encoderCount;
  interrupts();
  
  if (currentCount != lastEncoderCount) {
    currentRotation = currentCount * DEGREES_PER_PULSE;
    newEncoderData = true;
    lastEncoderCount = currentCount;
  }
}

void streamDataToSimulink() {
  TelemetryPacket packet;

  packet.gyroZ_angle   = gyroZ_angle;
  packet.encoder_angle = currentRotation;

  Serial.write((uint8_t*)&packet, sizeof(packet));
}


// NON-BLOCKING command reception
void checkForCommands() {
  while (Serial.available() > 0) {
    char c = Serial.read();
    
    // Check for command start ('M' for Motor command)
    if (c == 'M') {
      // Read the next 2 bytes for int16 motor command
      byte bytes[2];
      int bytesRead = 0;
      
      unsigned long startTime = millis();
      while (bytesRead < 2 && (millis() - startTime) < 10) {
        if (Serial.available() > 0) {
          bytes[bytesRead] = Serial.read();
          bytesRead++;
        }
      }
      
      if (bytesRead == 2) {
        // Convert bytes to int16 motor command
        int16_t newMotorSpeed;
        memcpy(&newMotorSpeed, bytes, 2);
        motorSpeed = newMotorSpeed;
        
        // confirmation
        // Serial.print("ACK:M:");
        // Serial.println(motorSpeed);
      }
    }
    // text-based command 
    else if (c == 'm' || c == 'S' || c == 'R') {
      processTextCommand(c);
    }
  }
}

// text commands ("M100\n" format)
void processTextCommand(char commandType) {
  String command = String(commandType);
  
  // Read until newline (with timeout)
  unsigned long startTime = millis();
  while (Serial.available() > 0 && (millis() - startTime) < 10) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') break;
    command += c;
  }
  
  if (commandType == 'm') {  
    motorSpeed = command.substring(1).toInt();
    // Serial.print("ACK:");
    // Serial.println(command);
  }
}

void applyMotorCommand() {
  // Apply speed limits
  int appliedSpeed = motorSpeed;
  
  if (abs(appliedSpeed) < MIN_SPEED && appliedSpeed != 0) {
    appliedSpeed = (appliedSpeed > 0) ? MIN_SPEED : -MIN_SPEED;
  }
  
  if (appliedSpeed > MAX_SPEED) appliedSpeed = MAX_SPEED;
  if (appliedSpeed < -MAX_SPEED) appliedSpeed = -MAX_SPEED;


  static int lastAppliedSpeed = 0;
  if (appliedSpeed != lastAppliedSpeed) {
    if (appliedSpeed != 0) {
      motor1.drive(appliedSpeed);
    } else {
      motor1.brake();
    }
    lastAppliedSpeed = appliedSpeed;
  }
}

// Encoder ISR 
void updateEncoder() {
  bool encA = digitalRead(ENCODER_A);
  bool encB = digitalRead(ENCODER_B);
  
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