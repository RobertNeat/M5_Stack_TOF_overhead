//Reading the object height on the surface using overhead distance sensor
//Sensor callibrates automatically when firs turn on
//TOF sensor library: https://www.arduino.cc/reference/en/libraries/vl53l0x/
#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;
int16_t zeroDistance = 0; // Variable to store the zero distance calibration value
int16_t sensor_readout = 0;
int16_t distance = 0;

int16_t calibrateSensor();

void setup() {
  Serial.begin(115200);
  Wire.begin(26, 32); // SDA, SCL
  sensor.init();
  sensor.setTimeout(500);

  // Perform initial calibration to set the zero distance
  zeroDistance = calibrateSensor();
  
  sensor.startContinuous();
}

void loop() {
  int16_t sensor_readout = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) {
    Serial.print(" TIMEOUT");
  } else {
    int16_t objectHeight = zeroDistance - sensor_readout; // Calculate object height
    Serial.print("Object height: ");
    Serial.print(objectHeight / 10.0);
    Serial.print(" cm, Sensor distance from the table surface (aka. max object distance): ");
    Serial.print(zeroDistance / 10.0);
    Serial.println(" cm");
  }
  
  delay(1000); // Delay between measurements
}

int16_t calibrateSensor() {
  Serial.println("Calibrating sensor...");
  delay(1000); // Allow time for the sensor to stabilize

  int16_t baseline = sensor.readRangeSingleMillimeters();
  Serial.print("Baseline measurement: ");
  Serial.print(baseline / 10.0);
  Serial.println(" cm");
  
  return baseline; // Use the baseline as the zero distance
}
