#include <Arduino.h>
#include "eloquent_esp32cam.h"

using eloq::camera;

// Defining Variables
int width;
int height;
int offset;
int dynamicThreshold;

int targetRow = 90;

int calculate_Threshold(uint8_t* buffer, int totalPixels);
int calculate_Offset(uint8_t* pixelBuffer, int dynamicThreshold);

void setup() {

  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- Starting ESP32-CAM Line-Tracking Test ---");

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
  
  while (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame: ");
    delay(2000);
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int totalPixels = width * height;

  dynamicThreshold = calculate_Threshold(pixelBuffer, totalPixels);
  offset = calculate_Offset(pixelBuffer, dynamicThreshold);

  Serial.printf("Offset Value: %d\n", offset);
  
  camera.free();    
  delay(20); 
}


int calculate_Offset(uint8_t* pixelBuffer, int dynamicThreshold) {
  
  long sumPositions = 0;
  int whitePixelCount = 0;
  int startIndex = targetRow * width;

  for (int x = 0; x < width; x++) {
    int pixelIndex = (startIndex) + x;
    uint8_t pixelValue = pixelBuffer[pixelIndex];
  
    if (pixelValue > dynamicThreshold) {
      sumPositions += x;
      whitePixelCount++;
    }
  }

  if (whitePixelCount > 0) {
    int line_centroid_x = sumPositions / whitePixelCount;
    int camera_midpoint_x = width / 2; 
    
    int alignment_offset = line_centroid_x - camera_midpoint_x;
    return alignment_offset; 
  }

  return 0;

}


int calculate_Threshold(uint8_t* buffer, int totalPixels) {
  long thresholdSum = 0;
  
  for (int i = 0; i < totalPixels; i++) {
    thresholdSum += buffer[i];
  }
  
  int frameAverage = thresholdSum / totalPixels;
  int calculatedThreshold = frameAverage;
  
  return calculatedThreshold;
}