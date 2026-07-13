#include <Wire.h>
#include <Arduino.h>
#include <VL53L1X.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Pin Definitions
#define SDA 14
#define SCL 15

#define IN1 18
#define IN2 19
#define IN3 32
#define IN4 33


// Class Definitions
VL53L1X tofSensor;


// Vision Variables
int width;
int height;
int dynamicThreshold;

int targetRow[] = {10, 90, 110};

// TODO
//PID variables (filled with dummy variables)
float kp_pos = 1.2;
float kp_ang = 5.0;
float ki = 0.1;
float kd = 0.3;

float errorIntegral = 0;
float errorDerivative = 0;
float currentError = 0;
float lastError = 0;

// Motor variables
int BASE_SPEED = 150;


// Function Definitions
void initialize();
int calculateThreshold(uint8_t* buffer, int totalPixels);
int calculatePositionOffset(uint8_t* pixelBuffer, int dynamicThreshold, int targetRow);
float calculateAngleOffset(uint8_t* pixelBuffer, int dynamicThreshold);

void turnMotors(float error);
void stopMotors();
void getEncoderData();
void readLeftEncoder();
void readRightEncoder();

float runObstacleDetection();
void executeObstacleManeuver();


void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Line-Tracking (With PID) Test ---");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); //160x120 resolution
  camera.pixformat.grayscale();
  
  Wire.begin(SDA, SCL);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  initialize();

  
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
  dynamicThreshold = calculateThreshold(pixelBuffer, totalPixels);
  
  int offsetPosition = calculatePositionOffset(pixelBuffer, dynamicThreshold, targetRow[1]);
  float offsetAngle = calculateAngleOffset(pixelBuffer, dynamicThreshold);

  
  float distance_to_object = runObstacleDetection();
  // if (distance_to_object <= 10) executeObstacleManeuver();

  // // PID
  currentError = offsetPosition*kp_pos + offsetAngle*kp_ang;

  errorIntegral += currentError;
  errorDerivative = currentError - lastError;

  float e = currentError + (errorIntegral*ki) + (errorDerivative*kd);
  turnMotors(e);

  lastError = currentError;

  // // Logging results for debugging - remove this at the end
  // Serial.printf("PosErr: %d | AngErr: %.1f | TurnOut: %.1f\n", offsetPosition, offsetAngle, e);  
  Serial.printf("dist: %f\n", distance_to_object);  
  Serial.printf("Error: %f\n", e);
  camera.free();
  delay(500);
  
}


void initialize() {
while (!camera.begin().isOk()) {
    Serial.println("Camera Initialization Failed: ");
    delay(2000);
  }
  Serial.println("Camera Initialization Successful: ");

  width = camera.resolution.getWidth();
  height = camera.resolution.getHeight();

  tofSensor.setTimeout(500);
  if (!tofSensor.init()) {
    Serial.println("Failed to detect or initialize VL53L0X ToF sensor!");
    while (1);
  }
  Serial.println("VL53L0X ToF Sensor Initialized Successfully.");

  tofSensor.startContinuous(50);
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
    // analogWrite(ENA, left_speed);
    
    // Left motor backward
  } else {
    Serial.printf("Left motor backward\n");
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    // analogWrite(ENA, abs(left_speed));
  }
  
  // Right motor forward
  if (right_speed >= 0) {
    Serial.printf("right motor forward\n");
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    // analogWrite(ENB, right_speed);
    
    // Right motor backward
  } else {
    Serial.printf("right motor backward\n");
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    // analogWrite(ENB, abs(right_speed));
  }

}


// Breaks
void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  
  // analogWrite(ENA, 0);
  // analogWrite(ENB, 0);
}


void getEncoderData() {

}


// void readLeftEncoder() {
//   int bState = digitalRead(LEFT_ENCODER_B);
//   if (bState == LOW) {
//     leftTicks++;
//   } else {
//     leftTicks--;
//   }
// }

// void readRightEncoder() {
//   int bState = digitalRead(RIGHT_ENCODER_B);
//   if (bState == LOW) {
//     rightTicks++;
//   } else {
//     rightTicks--;
//   }
// }


// Detect objects in front of robot
float runObstacleDetection() {

  uint16_t distance = tofSensor.read();

  if (tofSensor.timeoutOccurred()) {
    Serial.println(" ToF Sensor Timeout!");
    return -1.0;
  }
  return (float)distance;
}

//TODO
// It is a hassle to make complex code for this,
// Make a predefined block of code that makes the robot go clckwise around a circle
// avoiding  the obstacle (initial idea)
void executeObstacleManeuver() {
  return;
}