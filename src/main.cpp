#include <Wire.h>
#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

#define IN1 32
#define IN2 33
#define IN3 2
#define IN4 12

// Vision Variables
int width;
int height;
int dynamicThreshold;

int targetRow[] = {10, 90, 110};

// TODO
//PID variables (filled with dummy variables)
float kp_pos = 0.5;
float kp_ang = 1.0;
float ki = 0.1;
float kd = 0.3;

float errorIntegral = 0;
float errorDerivative = 0;
float currentError = 0;
float lastError = 0;

// Motor variables
int BASE_SPEED = 150;


// Function Definitions
int calculateThreshold(uint8_t* buffer, int totalPixels);
int calculatePositionOffset(uint8_t* pixelBuffer, int dynamicThreshold, int targetRow);
float calculateAngleOffset(uint8_t* pixelBuffer, int dynamicThreshold);

void turnMotors(float error);
void stopMotors();
void testMotors();
void rotateLeft();
void rotateRight();


void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Line-Tracking (With PID) Test ---");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); //160x120 resolution
  camera.pixformat.grayscale();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  while (!camera.begin().isOk()) {
    Serial.println("Camera Initialization Failed: ");
    delay(2000);
  }
  Serial.println("Camera Initialization Successful: ");

  width = camera.resolution.getWidth();
  height = camera.resolution.getHeight();

  
  Serial.printf("Initialized with Width: %d, Height: %d\n", width, height);
}


void loop() {
  
  // Checking if camera is working
  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame, starting without camera: ");
    delay(2000);
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int totalPixels = width * height;

  // Calculating threshold value to dynamically classify black and white
  // dynamicThreshold = calculateThreshold(pixelBuffer, totalPixels);
  dynamicThreshold = 120;
  
  int offsetPosition = calculatePositionOffset(pixelBuffer, dynamicThreshold, targetRow[1]);
  float offsetAngle = calculateAngleOffset(pixelBuffer, dynamicThreshold);

  // // PID
  currentError = offsetPosition*kp_pos + offsetAngle*kp_ang;

  errorIntegral += currentError;
  errorDerivative = currentError - lastError;

  float e = currentError + (errorIntegral*ki) + (errorDerivative*kd);
  // turnMotors(e);

  lastError = currentError;

  // // Logging results for debugging - remove this at the end
  Serial.printf("PosErr: %d | AngErr: %.1f | TurnOut: %.1f\n", offsetPosition, offsetAngle, e);  
  Serial.printf("Error: %f\n", e);
  Serial.printf("Threshold: %f\n", dynamicThreshold);
  delay(200);
  camera.free();


}

void testMotors() {

  // Serial.println("Moving motors\n");
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, LOW);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, LOW);

  // delay(500);

  // Serial.println("Stopping motors\n");
  // digitalWrite(IN1, HIGH);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, HIGH);
  // digitalWrite(IN4, HIGH);

  // delay(500);

  // Serial.println("Moving motors Backward\n");
  // digitalWrite(IN1, LOW);
  // digitalWrite(IN2, HIGH);
  // digitalWrite(IN3, LOW);
  // digitalWrite(IN4, HIGH);

  // delay(500);
}

void rotateLeft() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void rotateRight() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Check if line is off-centered from middle of the frame
int calculatePositionOffset(uint8_t* pixelBuffer, int dynamicThreshold, int targetRow) {
  
  long sumPositions = 0;
  int whitePixelCount = 0;
  int startIndex = targetRow * width;
  
  // Count how many pixels are white and get their position
  for (int x = 0; x < width; x++) {
    int pixelIndex = (startIndex) + x;
    uint8_t pixelValue = pixelBuffer[pixelIndex];
    
    if (pixelValue > dynamicThreshold) {
      sumPositions += x;
      whitePixelCount++;
    }
  }
  
  // Get average position of white pixels
  if (whitePixelCount > 0) {
    int line_centroid_x = sumPositions / whitePixelCount;
    int camera_midpoint_x = width / 2; 
    
    int alignment_offset = line_centroid_x - camera_midpoint_x;
    return alignment_offset; 
  }
  
  return 0;
}


// Angle offset from center used to control PID algorithm
float calculateAngleOffset(uint8_t* pixelBuffer, int dynamicThreshold) {
  
  int top_XOffset = calculatePositionOffset(pixelBuffer, dynamicThreshold, targetRow[0]);
  int bottom_XOffset = calculatePositionOffset(pixelBuffer, dynamicThreshold, targetRow[2]);
  
  int  dy = targetRow[2] - targetRow[0];
  int dx = top_XOffset - bottom_XOffset;
  
  float angle = atan2(dx, dy);
  
  return angle;
}


// Instead of statically defining the threshold once, define it as the average of all pixels on frame
int calculateThreshold(uint8_t* pixelBuffer, int totalPixels) {
  
  long thresholdSum = 0;
  
  for (int i = 0; i < totalPixels; i++) {
    thresholdSum += pixelBuffer[i];
  }
  
  int frameAverage = thresholdSum / totalPixels;
  int calculatedThreshold = frameAverage;
  
  return calculatedThreshold;
}


// Motor control algorithm based off of PID
void turnMotors(float error) {

  int left_speed = BASE_SPEED + error;
  int right_speed = BASE_SPEED - error;
  
  left_speed = constrain(left_speed, -255, 255);
  right_speed = constrain(right_speed, -255, 255);
  
  // Left motor forward
  if (left_speed >= 0) {
    Serial.printf("Left motor forward\n");
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    delay(100);
    
    // Left motor backward
  } else {
    Serial.printf("Left motor backward\n");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    delay(100);
    
  }
  
  // Right motor forward
  if (right_speed >= 0) {
    Serial.printf("right motor forward\n");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    delay(100);
    
    // Right motor backward
  } else {
    Serial.printf("right motor backward\n");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    delay(100);
  
  }

  delay(500);
}


// Breaks
void stopMotors() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}