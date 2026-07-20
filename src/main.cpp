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
float LEFT_MOTOR_BIAS = 0.90; 

// Camera Processing
int width;
int height;
int targetRows[3] = {70, 90, 110}; // Top, Middle, Bottom scan rows      
int threshold = 190;       

// Base Speed Settings
int BASE_SPEED = 110;    
int MIN_SPEED  = 40;     

// PID Parameters
float Kp_pos = 1.20;     
float Kp_ang = 0.45;     
float Ki     = 0.00;
float Kd     = 0.50;     

float error = 0;
float prevError = 0;
float integral = 0;

// Logging control
int frameCounter = 0;
const int LOG_EVERY_N_FRAMES = 5; // Print ASCII map every 5th frame

// Function declarations
void turnMotors(int base_speed, float steering_offset);
void stopMotors();
float calculate_positionOffset(uint8_t* pixelBuffer, int threshold, int row);
float calculate_AngleOffset(uint8_t* pixelBuffer, int threshold);
void printAsciiLine(uint8_t* pixelBuffer, float midX, float angle_offset, int current_base_speed, float steering_offset);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Continuous Line Follower with Visual Logging ---");

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

  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame.");
    return; 
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  
  float midX = calculate_positionOffset(pixelBuffer, threshold, targetRows[1]);

  float steering_offset = 0;
  int current_base_speed = BASE_SPEED;
  float angle_offset = 0;

  if (midX >= 0) {
    // Position Error (-80 to +80)
    error = midX - (width / 2.0);

    if (abs(error) < 3) {
      error = 0;
    }

    // Angle Offset in degrees (0 when straight)
    angle_offset = calculate_AngleOffset(pixelBuffer, threshold);

    // Dynamic Cornering Speed Adjustment
    float severity = (abs(error) / (width / 2.0)) + (abs(angle_offset) / 45.0);
    severity = constrain(severity, 0.0, 1.0);
    
    current_base_speed = BASE_SPEED - (severity * (BASE_SPEED - MIN_SPEED));

    integral += error;
    integral = constrain(integral, -100, 100);
    
    float derivative = error - prevError;

    // Combined PID steering response
    steering_offset = (Kp_pos * error) + (Kp_ang * angle_offset) + (Ki * integral) + (Kd * derivative);
    
    prevError = error;

  } else {
    // LINE LOST: Pivot search
    current_base_speed = 0; 
    if (prevError > 0) {
      steering_offset = 120;  // Pivot Right
    } else {
      steering_offset = -120; // Pivot Left
    }
  }

  // Log visual telemetry periodically
  frameCounter++;
  if (frameCounter >= LOG_EVERY_N_FRAMES) {
    printAsciiLine(pixelBuffer, midX, angle_offset, current_base_speed, steering_offset);
    frameCounter = 0;
  }

  // Continuous motor actuation
  turnMotors(current_base_speed, steering_offset);
  
  camera.free();
}

// Generates an ASCII visual bar of what row 90 sees across a downscaled 40-character screen
void printAsciiLine(uint8_t* pixelBuffer, float midX, float angle_offset, int current_base_speed, float steering_offset) {
  const int displayWidth = 40; // Downscale 160px width to 40 characters for fast printing
  char asciiBar[displayWidth + 1];
  
  int middleRow = targetRows[1];
  int rowStart = middleRow * width;

  // Render ground/line pixels across the row
  for (int i = 0; i < displayWidth; i++) {
    int pixelX = i * (width / displayWidth);
    if (pixelBuffer[rowStart + pixelX] > threshold) {
      asciiBar[i] = '#'; // Line pixel detected
    } else {
      asciiBar[i] = '.'; // Dark background
    }
  }

  // Mark center of camera view
  int centerCharPos = (width / 2) / (width / displayWidth);
  if (asciiBar[centerCharPos] == '.') {
    asciiBar[centerCharPos] = '+'; 
  }

  // Mark calculated line center
  if (midX >= 0) {
    int lineCharPos = midX / (width / displayWidth);
    lineCharPos = constrain(lineCharPos, 0, displayWidth - 1);
    asciiBar[lineCharPos] = '|'; 
  }

  asciiBar[displayWidth] = '\0'; // Null-terminate string

  if (midX >= 0) {
    Serial.printf("[%s] Err:%5.1f | Ang:%5.1f deg | Spd:%3d | Steer:%6.1f\n", 
                  asciiBar, error, angle_offset, current_base_speed, steering_offset);
  } else {
    Serial.printf("[%s] *** LINE LOST *** | Spd:%3d | Steer:%6.1f\n", 
                  asciiBar, current_base_speed, steering_offset);
  }
}

float calculate_positionOffset(uint8_t* pixelBuffer, int threshold, int row) {
  long sumX = 0;
  int totalWhitePixels = 0;

  for (int r = row - 2; r <= row + 2; r++) {
    int startIndex = r * width;
    for (int x = 0; x < width; x++) {
      if (pixelBuffer[startIndex + x] > threshold) {
        sumX += x;
        totalWhitePixels++;
      }
    }
  }

  if (totalWhitePixels == 0) {
    return -1.0; 
  }

  return (float)sumX / totalWhitePixels;
}

float calculate_AngleOffset(uint8_t* pixelBuffer, int threshold) {
  float top_XOffset = calculate_positionOffset(pixelBuffer, threshold, targetRows[0]);
  float bottom_XOffset = calculate_positionOffset(pixelBuffer, threshold, targetRows[2]);

  if (top_XOffset < 0 || bottom_XOffset < 0) {
    return 0.0;
  }

  float dx = top_XOffset - bottom_XOffset;
  float dy = targetRows[2] - targetRows[0]; 

  float angle_rad = atan2(dx, dy);
  float angle_deg = angle_rad * (180.0 / PI);

  return angle_deg;
}

void turnMotors(int base_speed, float steering_offset) {
  int left_speed = (int)((base_speed + steering_offset) * LEFT_MOTOR_BIAS);
  int right_speed = base_speed - steering_offset;
  
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