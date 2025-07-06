#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Compass sensor object
Adafruit_HMC5883_Unified compass = Adafruit_HMC5883_Unified(12345);  // Replace 12345 with the correct I2C address of your compass

// Motor pins
const int motorPin1 = 9;
const int motorPin2 = 10;

// Set desired heading
const float desiredHeading = 180.0;  // Set your desired heading in degrees (0-360)

void setup() {
  // Initialize the compass
  compass.begin();

  // Set the measurement range of the compass (adjust if needed)
  compass.setMagGain(HMC5883_MAGGAIN_1_3);

  // Set motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
}

void loop() {
  // Read the current heading from the compass
  sensors_event_t event;
  compass.getEvent(&event);
  float currentHeading = ((float)event.orientation.x);
  
  // Calculate the difference between desired and current headings
  float headingDiff = desiredHeading - currentHeading;
  
  // Adjust the motor speed based on the heading difference
  int motorSpeed = map(headingDiff, -180, 180, -255, 255);
  
  // Control the motor based on the heading difference
  if (headingDiff > 0) {
    analogWrite(motorPin1, motorSpeed);
    analogWrite(motorPin2, 0);
  } else {
    analogWrite(motorPin1, 0);
    analogWrite(motorPin2, -motorSpeed);
  }
}