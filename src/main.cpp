#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Pin definitions
#define IN1 2   // Left Motor Direction
#define IN2 12  // Left Motor Speed (PWM Channel 0)
#define IN3 32  // Right Motor Direction
#define IN4 33  // Right Motor Speed (PWM Channel 1)

const int leftChannel = 0;  
const int rightChannel = 1; 

// Hardware Balance Tuning
float LEFT_MOTOR_BIAS = 0.95; 

// Camera Processing
int width;
int height;
int targetRow = 75;        // Lowered slightly (closer to wheels) to track tight curves better
int threshold = 120;       

// Base Speed
int BASE_SPEED = 120;

// PID Parameters
float Kp = 0.9;
float Ki = 0.0;
float Kd = 0.7;

float error = 0;
float prevError = 0;
float integral = 0;

bool isRunning = true; 

// Function declarations
void turnMotors(float steering_offset);
void stopMotors();
void checkSerialCommands();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Continuous PID Line Follower (Robust Sweep) ---");
  Serial.println(">>> PRESS SPACEBAR IN SERIAL MONITOR TO START/STOP <<<");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); 
  camera.pixformat.grayscale();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  
  ledcSetup(leftChannel, 5000, 8);
  ledcSetup(rightChannel, 5000, 8);
  ledcAttachPin(IN2, leftChannel);
  ledcAttachPin(IN4, rightChannel);

  while (!camera.begin().isOk()) {
    Serial.println("Camera Initialization Failed!");
    delay(2000);
  }
  Serial.println("Camera Successfully Initialized!");
  
  width = camera.resolution.getWidth();   // 160 px
  height = camera.resolution.getHeight(); // 120 px

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
}

void loop() {
  checkSerialCommands();

  // Manual pause via Spacebar in Serial Monitor
  if (!isRunning) {
    stopMotors();
    delay(50); 
    return;
  }

  // Frame Capture
  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame.");
    return; 
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int startIndex = targetRow * width;
  
  long sumX = 0;
  int totalBlackPixels = 0;

  // Scan across target row to find center of mass of the line
  for (int x = 0; x < width; x++) {
    if (pixelBuffer[startIndex + x] < threshold) {
      sumX += x;
      totalBlackPixels++;
    }
  }

  float steering_offset = 0;

  if (totalBlackPixels > 0) {
    // Calculate actual center position of line (0 to 159)
    float lineCenter = (float)sumX / totalBlackPixels;
    
    // Error relative to center of image (80)
    error = lineCenter - (width / 2.0); 

    // Deadband filter: Ignore tiny jitter near true center
    if (abs(error) < 4) {
      error = 0;
    }

    // Standard PID calculation
    integral += error;
    integral = constrain(integral, -100, 100); // Anti-windup clamp
    
    float derivative = error - prevError;
    steering_offset = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // Save last valid non-zero error so we know which way we were turning before losing the line
    if (error != 0) {
      prevError = error;
    }

  } else {
    // ALL WHITE (LINE LOST): Force hard sweep in direction of last known error
    if (prevError >= 0) {
      steering_offset = 140;  // Sharp right recovery turn
    } else {
      steering_offset = -140; // Sharp left recovery turn
    }
  }

  // Continuously drive without stopping
  turnMotors(steering_offset);
                
  camera.free();
}

void checkSerialCommands() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == ' ') {
      isRunning = !isRunning;
      if (isRunning) {
        Serial.println("\n>>> STARTING MOTOR ENGINE <<<");
      } else {
        Serial.println("\n>>> ENGINE PAUSED <<<");
        stopMotors();
      }
    }
  }
}

void turnMotors(float steering_offset) {
  int left_speed = BASE_SPEED + steering_offset;
  int right_speed = BASE_SPEED - steering_offset;
  
  left_speed = (int)(left_speed * LEFT_MOTOR_BIAS);
  
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  if (left_speed >= 0) {
    digitalWrite(IN1, HIGH);
    ledcWrite(leftChannel, left_speed);
  } else {
    digitalWrite(IN1, LOW);
    ledcWrite(leftChannel, abs(left_speed));
  }
  
  if (right_speed >= 0) {
    digitalWrite(IN3, HIGH);
    ledcWrite(rightChannel, right_speed);
  } else {
    digitalWrite(IN3, LOW);
    ledcWrite(rightChannel, abs(right_speed));
  }
}

void stopMotors() {
  ledcWrite(leftChannel, 0);
  ledcWrite(rightChannel, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}