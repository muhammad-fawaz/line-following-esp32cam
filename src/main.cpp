#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Safe Pin Map
#define IN1 2   // Left Motor Direction
#define IN2 12  // Left Motor Speed (PWM Channel 0)
#define IN3 32  // Right Motor Direction
#define IN4 33  // Right Motor Speed (PWM Channel 1)

const int leftChannel = 0;  
const int rightChannel = 1; 

// Vision Settings
int width;
int height;
int targetRow = 60;        
int threshold = 120;       
int trigger_count = 3;     
bool wasAllActive = false;

// STEP-AND-SCAN TUNING PARAMETERS
int BASE_SPEED = 110;       // Speed needs to be high enough to overcome friction instantly
int STEP_DURATION = 100;     // How many milliseconds to pulse the motors (Lower = smaller steps)
int SETTLE_DELAY = 20;      // How many milliseconds to sit still and wait for a clean screen reading

float lastError = 0;
int stopCount = 0;

void turnMotors(float steering_offset);
void stopMotors();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Step-and-Scan Follower ---");

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
  
  width = camera.resolution.getWidth();
  height = camera.resolution.getHeight();

  digitalWrite(IN1, HIGH);
  digitalWrite(IN3, HIGH);
}

void loop() {

  // 1. CHASSIS IS ISOLATED/STATIONARY -> Capture a crisp frame
  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame.");
    return; 
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int startIndex = targetRow * width;

  int count_L3 = 0; int count_L2 = 0; int count_L1 = 0; int count_C = 0; 
  int count_R1 = 0; int count_R2 = 0; int count_R3 = 0; 

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

  if (L3 && L2 && L1 && C && R1 && R2 && R3) {
    steering_offset = 0; 
    if (!wasAllActive) {
      stopCount++;
      wasAllActive = true; 
    }
  } 
  else {
    wasAllActive = false; 

    if (C && L1 && R1)       steering_offset = 0; 
    else if (C && !L1 && !R1) steering_offset = 0;    
    else if (L1 && !R1)       steering_offset = -20;  
    else if (R1 && !L1)       steering_offset = 20;   
    else if (L2)              steering_offset = -50;  
    else if (R2)              steering_offset = 50;   
    else if (L3)              steering_offset = -95;  // Strong turn if off track
    else if (R3)              steering_offset = 95;   
    else if (!L3 && !L2 && !L1 && !C && !R1 && !R2 && !R3) {
      steering_offset = (lastError > 0) ? 95 : -95; // Search back if line is completely gone
    }
  }

  if (L3 || L2 || L1 || R1 || R2 || R3) {
    lastError = steering_offset;
  }

  if (stopCount >= 6) { 
    stopMotors(); 
    while(1); 
  } 

  Serial.printf("[ %d | %d | %d | %d | %d | %d | %d ] -> Steer: %.0f\n", L3, L2, L1, C, R1, R2, R3, steering_offset);

  // 2. PULSE THE MOTORS
  turnMotors(steering_offset);
  delay(STEP_DURATION); // Let motors run for exactly this many milliseconds
  
  // 3. FORCE ACTIVE BRAKING IMMEDIATELY
  stopMotors();
  
  // 4. SETTLE TIME FOR THE CAMERA
  delay(SETTLE_DELAY); // Wait for physical shaking to stop before looping back
                
  camera.free();
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

// Rewritten to act as an active electronic brake
void stopMotors() {
  ledcWrite(leftChannel, 0);
  ledcWrite(rightChannel, 0);
  // Shorting the direction pins to ground forces the H-Bridge to actively stop the motor coils
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}