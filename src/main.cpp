#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Safe Pin Map (Adjusted per your current setup)
#define IN1 33  // Left Motor Direction
#define IN2 32  // Left Motor Speed (PWM Channel 0)
#define IN3 12  // Right Motor Direction
#define IN4 2   // Right Motor Speed (PWM Channel 1)

// ESP32 PWM Channel Configurations (For older 2.x Arduino Cores)
const int leftChannel = 0;  
const int rightChannel = 1; 

// Vision Settings
int width;
int height;
int targetRow = 60;        // Scanning row across the middle of the frame
int threshold = 120;       // Hardcoded light threshold
int trigger_count = 3;     // Minimum white pixels to trigger a virtual sensor
bool wasAllActive = false;

// Motor variables
int BASE_SPEED = 90;       // Lowered slightly from 100 to reduce forward momentum momentum overshoots
float lastError = 0;
int stopCount = 0;


void turnMotors(float steering_offset);
void stopMotors();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Paced 7-Sensor Emulation ---");
  Serial.println(">>> KEY COMMANDS: Send 's' to START, 'q' to PAUSE/STOP <<<");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); // 160x120 resolution
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
  Serial.println("Camera Initialization Successful!");
  
  width = camera.resolution.getWidth();
  height = camera.resolution.getHeight();

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
}

void loop() {

  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame.");
    return; 
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int startIndex = targetRow * width;

  int count_L3 = 0; 
  int count_L2 = 0; 
  int count_L1 = 0; 
  int count_C  = 0; 
  int count_R1 = 0; 
  int count_R2 = 0; 
  int count_R3 = 0; 

  for (int x = 0; x < width; x++) {
    uint8_t pixelValue = pixelBuffer[startIndex + x];
    
    if (pixelValue < threshold) {
      if (x < 23)            count_L3++;
      else if (x < 46)       count_L2++;
      else if (x < 69)       count_L1++;
      else if (x < 92)       count_C++;  
      else if (x < 115)      count_R1++;
      else if (x < 138)      count_R2++;
      else                   count_R3++;
    }
  }

  bool L3 = (count_L3 > trigger_count);
  bool L2 = (count_L2 > trigger_count);
  bool L1 = (count_L1 > trigger_count);
  bool C  = (count_C  > trigger_count);
  bool R1 = (count_R1 > trigger_count);
  bool R2 = (count_R2 > trigger_count);
  bool R3 = (count_R3 > trigger_count);

  float steering_offset = 0;

  // --- INTERSECTION & DEVIATION HANDLING TREE ---
  if (L3 && L2 && L1 && C && R1 && R2 && R3) {
    steering_offset = 0; 
    
    if (!wasAllActive) {
      stopCount++;
      wasAllActive = true; 
      Serial.printf("!!! New Marker Detected! Total Count: %d !!!\n", stopCount);
    }
  } 
  else {
    wasAllActive = false; 

    if (C && L1 && R1) {
      steering_offset = 0; 
    }
    else if (C && !L1 && !R1) {
      steering_offset = 0;    
    }
    else if (L1 && !R1) {
      steering_offset = -15;  
    }
    else if (R1 && !L1) {
      steering_offset = 15;   
    }
    else if (L2) {
      steering_offset = -40;  
    }
    else if (R2) {
      steering_offset = 40;   
    }
    else if (L3) {
      steering_offset = -65;  // Trimmed slightly down from 90 to prevent aggressive over-turning
    }
    else if (R3) {
      steering_offset = 65;   // Trimmed slightly down from 90 to prevent aggressive over-turning
    }
    else if (!L3 && !L2 && !L1 && !C && !R1 && !R2 && !R3) {
      steering_offset = 0; 
    }
  }

  // --- BRAKING RULES ---
  if (stopCount >= 6) { 
    stopMotors(); 
    while(1); 
  } else {
    turnMotors(steering_offset);
  }
  
  lastError = steering_offset;
  
  Serial.printf("[ %d | %d | %d | %d | %d | %d | %d ] -> Steer: %.0f\n", L3, L2, L1, C, R1, R2, R3, steering_offset);
                
  camera.free();

  // --- PACING DELAY ---
  // Forces the robot to actively execute the new motor speeds for 25 milliseconds
  // before snapping a new picture. This prevents sensor-reading lag loops.
  delay(200); 
}

void turnMotors(float steering_offset) {
  int left_speed = BASE_SPEED + steering_offset;
  int right_speed = BASE_SPEED - steering_offset;
  
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