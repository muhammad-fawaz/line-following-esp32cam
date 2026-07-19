#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Pin definitions.
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

// Step-and-scan tuning parameters.
int BASE_SPEED = 110;
int STEP_DURATION = 100;
int SETTLE_DELAY = 10;

// State Variables
float prevError = 0;
int stopCount = 0;
bool isRunning = false; // Starts paused until you press Spacebar

// Function declarations
void turnMotors(float steering_offset);
void stopMotors();
void checkSerialCommands();

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Step-and-Scan Follower ---");
  Serial.println(">>> PRESS SPACEBAR IN SERIAL MONITOR TO START/STOP <<<");

  camera.pinout.wrover();
  camera.brownout.disable();
  camera.resolution.qqvga(); // 160x120 pixels
  camera.pixformat.grayscale();
  
  pinMode(IN1, OUTPUT);
  pinMode(IN3, OUTPUT);
  
  // Defining pins IN2 and IN4 as PWM pins so that we can control the speed of the motors.
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
  // Listen for Spacebar taps at the very beginning of every loop execution
  checkSerialCommands();

  // If the robot is paused, lock the motors up and skip everything else
  if (!isRunning) {
    stopMotors();
    delay(50); 
    return;
  }

  // Check if camera is capturing frame.
  if (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame.");
    return; 
  }
  
  // Get frame from camera and store it in an array.
  uint8_t* pixelBuffer = camera.frame->buf;
  int startIndex = targetRow * width;

  // Set all sensors reading black at the start.
  int count_L3 = 0; int count_L2 = 0; int count_L1 = 0; int count_C = 0; 
  int count_R1 = 0; int count_R2 = 0; int count_R3 = 0; 

  // Go through each pixel in the row and count how many black pixels are detected.
  for (int x = 0; x < width; x++) {
    uint8_t pixelValue = pixelBuffer[startIndex + x];

    // If black pixel is detected, increase count of the dedicated zone
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

  // If no. of black pixels for a certain sensor were greater than trigger value, mark that spot as black
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

    if (C && L1 && R1)        steering_offset = 0; 
    else if (C && !L1 && !R1) steering_offset = 0;    
    else if (L1 && !R1)       steering_offset = -20;  
    else if (R1 && !L1)       steering_offset = 20;   
    else if (L2)              steering_offset = -50;  
    else if (R2)              steering_offset = 50;   
    else if (L3)              steering_offset = -95;
    else if (R3)              steering_offset = 95;   

    else if (!L3 && !L2 && !L1 && !C && !R1 && !R2 && !R3) {  // All sensors detect white (line lost)
      if (abs(prevError) < 40) {  // Likely driving through a track gap
        steering_offset = 0;
      } else {  // Fell off an extreme curve corner
        if (prevError > 0) {
          steering_offset = 95; 
        } else {
          steering_offset = -95;   
        }
      }
    }
  }

  if ((L1 || C || R1) && !(L3 && R3)) {
    prevError = steering_offset;
  }

  if (stopCount >= 6) { 
    stopMotors(); 
    while(1); 
  } 

  // Logging results on Serial monitor.
  Serial.printf("[ %d | %d | %d | %d | %d | %d | %d ] -> Steer: %.0f\n", L3, L2, L1, C, R1, R2, R3, steering_offset);

  // Turn the motors according to the visual matrix mapping
  turnMotors(steering_offset);
  delay(STEP_DURATION);
  
  // Stop the motors briefly so that the chassis can settle for a clean visual frame
  stopMotors();
  delay(SETTLE_DELAY);
                
  camera.free();
}

// Scans the serial buffer for keyboard commands
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    
    // Check if user hit the Spacebar
    if (incomingChar == ' ') {
      isRunning = !isRunning; // Flip state variable
      
      if (isRunning) {
        Serial.println("\n>>> STARTING MOTOR ENGINE <<<");
      } else {
        Serial.println("\n>>> ENGINE PAUSED - BRINGING TO AN ACTIVE STOP <<<");
        stopMotors();
      }
    }
  }
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

// Active electronic brake
void stopMotors() {
  ledcWrite(leftChannel, 0);
  ledcWrite(rightChannel, 0);
  digitalWrite(IN1, LOW);
  digitalWrite(IN3, LOW);
}