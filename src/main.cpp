#include <Wire.h>
#include <Arduino.h>
#include <VL53L0X.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Vision Variables
int width;
int height;
int dynamicThreshold;

int targetRow[] = {10, 90, 110};

// TODO
//PID variables
float kp_pos = 1.2;
float kp_ang = 5.0;
float ki = 0.1;
float kd = 0.3;

float errorIntegral = 0;
float errorDerivative = 0;
float currentError = 0;
float lastError = 0;


// Function Definitions
int calculate_Threshold(uint8_t* buffer, int totalPixels);
int calculate_positionOffset(uint8_t* pixelBuffer, int dynamicThreshold, int targetRow);
float calculate_AngleOffset(uint8_t* pixelBuffer, int dynamicThreshold);

void turn_motors(float error);
float run_obstacle_detection();
void run_obstacle_maneuver();


void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Line-Tracking (With PID) Test ---");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); //160x120 resolution
  camera.pixformat.grayscale();
  
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
  while (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame: ");
    delay(2000);
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int totalPixels = width * height;

  // Calculating threshold value to dynamically classify black and white
  dynamicThreshold = calculate_Threshold(pixelBuffer, totalPixels);
  
  int offsetPosition = calculate_positionOffset(pixelBuffer, dynamicThreshold, targetRow[1]);
  float offsetAngle = calculate_AngleOffset(pixelBuffer, dynamicThreshold);

  // PID
  currentError = offsetPosition*kp_pos + offsetAngle*kp_ang;

  errorIntegral += currentError;
  errorDerivative = currentError - lastError;

  float e = currentError + (errorIntegral*ki) + (errorDerivative*kd);
  turn_motors(e);

  lastError = currentError;

  // Logging results for debugging - remove this at the end
  Serial.printf("PosErr: %d | AngErr: %.1f | TurnOut: %.1f\n", offsetPosition, offsetAngle, e);  
  camera.free();    
}


// Check if line is off-centered from middle of the frame
int calculate_positionOffset(uint8_t* pixelBuffer, int dynamicThreshold, int targetRow) {
  
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
float calculate_AngleOffset(uint8_t* pixelBuffer, int dynamicThreshold) {
  
  int top_XOffset = calculate_positionOffset(pixelBuffer, dynamicThreshold, targetRow[0]);
  int bottom_XOffset = calculate_positionOffset(pixelBuffer, dynamicThreshold, targetRow[2]);
  
  int  dy = targetRow[2] - targetRow[0];
  int dx = top_XOffset - bottom_XOffset;

  float angle = atan2(dy, dx);
  
  return angle;
}


// Instead of statically defining the threshold once, define it as the average of all pixels on frame
int calculate_Threshold(uint8_t* pixelBuffer, int totalPixels) {

  long thresholdSum = 0;
  
  for (int i = 0; i < totalPixels; i++) {
    thresholdSum += pixelBuffer[i];
  }
  
  int frameAverage = thresholdSum / totalPixels;
  int calculatedThreshold = frameAverage;
  
  return calculatedThreshold;
}


//TODO
// Motor control algorithm based off of PID
void turn_motors(float error) {
  return;
}


//TODO
// Detect objects in frint of robot
float run_obstacle_detection() {
  return 0.0;
}

//TODO
// It is a hassle to make complex code for this,
// Make a predefined block of code that makes the robot go clckwise around a circle
// avoiding  the obstacle (initial idea)
void run_obstacle_maneuver() {
  return;
}