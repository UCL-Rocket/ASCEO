/******************************************************************************
TestRun.ino - Single Motor with Encoder
TB6612FNG H-Bridge Motor Driver Example code

Connections:
TB6612FNG:
- AIN1 -> D6
- AIN2 -> D5
- PWMA -> D8
- STBY -> D7
- VM -> Motor power (6V)
- VCC -> Arduino 3V3
- GND -> Arduino GND

Encoder:
- Channel A -> D3 (with interrupt)
- Channel B -> D4

******************************************************************************/

#include <SparkFun_TB6612.h>

// Motor control pins
#define AIN1 6
#define AIN2 5
#define PWMA 8
#define STBY 7

// Encoder pins
#define ENCODER_A 3
#define ENCODER_B 4

// Encoder variables
volatile long encoderCount = 0;
bool lastEncA = 0;
bool lastEncB = 0;

// Motor configuration
const int offsetA = 1; // Change to -1 if motor direction is reversed

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);

void setup()
{
  Serial.begin(115200);
  Serial.println("Motor Test with Encoder");
  
  // Setup encoder pins
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  
  // Attach interrupt for encoder channel A
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), updateEncoder, CHANGE);
  
  // Read initial encoder states
  lastEncA = digitalRead(ENCODER_A);
  lastEncB = digitalRead(ENCODER_B);
}

void loop()
{
  Serial.print("Encoder Count: ");
  Serial.println(encoderCount);
  
  // Test 1: Forward
  Serial.println("Forward at 150 for 2 seconds");
  motor1.drive(150);
  delay(2000);
  
  Serial.print("Encoder Count after forward: ");
  Serial.println(encoderCount);
  
  // Test 2: Brake
  Serial.println("Braking");
  motor1.brake();
  delay(1000);
  
  // Test 3: Reverse
  Serial.println("Reverse at 150 for 2 seconds");
  motor1.drive(-150);
  delay(2000);
  
  Serial.print("Encoder Count after reverse: ");
  Serial.println(encoderCount);
  
  // Test 4: Stop
  Serial.println("Stopping");
  motor1.brake();
  delay(2000);
  
  // Test 5: Variable speed test
  for(int speed = 0; speed <= 255; speed += 25) {
    Serial.print("Speed: ");
    Serial.println(speed);
    motor1.drive(speed);
    delay(500);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
  }
  
  motor1.brake();
  delay(2000);
  
  // Test 6: Negative speed (reverse)
  for(int speed = 0; speed >= -255; speed -= 25) {
    Serial.print("Speed: ");
    Serial.println(speed);
    motor1.drive(speed);
    delay(500);
    Serial.print("Encoder Count: ");
    Serial.println(encoderCount);
  }
  
  motor1.brake();
  delay(3000);
  
  // Reset encoder for next cycle
  encoderCount = 0;
  Serial.println("Reset encoder count");
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