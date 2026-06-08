#include <Arduino.h>
#include "eloquent_esp32cam.h"

// Defining Variables
int dynamicThreshold;
uint16_t thresholdSum;
uint16_t width;
uint16_t height;

int targetRow = 90;

using eloq::camera;

int calculate_Threshold(uint8_t* buffer, int totalPixels);

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
  Serial.printf("Width: %d, Height: %d\n", width, height);

}


void loop() {

  while (!camera.capture().isOk()) {
    Serial.println("Failed to Capture Frame: ");
    delay(2000);
  }
  
  uint8_t* pixelBuffer = camera.frame->buf;
  int totalPixels = width * height;
  
  dynamicThreshold = calculate_Threshold(pixelBuffer, totalPixels);
  
  long sumPositions = 0;
  int whitePixelCount = 0;
  int startIndex = targetRow * width;
  
  // 2. Loop across the single horizontal row
  for (int x = 0; x < width; x++) {
    int pixelIndex = (startIndex) + x;
    uint8_t pixelValue = pixelBuffer[pixelIndex];
  
    if (pixelValue > dynamicThreshold) {
      sumPositions += x;
      whitePixelCount++;
    }
  }

  if (whitePixelCount > 0) {

    int line_centroid_x = sumPositions / whitePixelCount;    //problem here. always 79    
    int camera_midpoint_x = width / 2; 
    
    int alignment_offset = line_centroid_x - camera_midpoint_x;

    Serial.printf("Line Centroid X: %d  | Offset from Middle: %d \n", line_centroid_x, alignment_offset);
    delay(20);
    
  } else {
    Serial.println("Line Lost! [Error 999]");
  }

  camera.free();    
  delay(20); 
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