#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Set the LCD address and dimensions

// Initialize Magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Motor Pins
const int motorPin = 9;

// Potentiometer Pin
const int potPin = A0;

// Calibration values for the magnetic compass
const float declinationAngle = 0.22;  // Adjust this value according to your location

void setup() {
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.backlight();

  // Initialize Magnetometer
  mag.begin();

  // Set Magnetometer measurement range
  mag.setMagGain(HMC5883L_GAIN_1_3);

  // Set Motor pin as output
  pinMode(motorPin, OUTPUT);

  // Set up Serial Monitor
  Serial.begin(9600);
}

void loop() {
  // Read potentiometer value
  int potValue = analogRead(potPin);

  // Map the potentiometer value to the motor speed range
  int motorSpeed = map(potValue, 0, 1023, 0, 255);

  // Set motor speed
  analogWrite(motorPin, motorSpeed);

  // Read magnetometer data
  sensors_event_t event;
  mag.getEvent(&event);

  // Calculate heading
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  heading += declinationAngle;

  // Normalize heading value to 0-360 degrees
  if (heading < 0) {
    heading += 2 * PI;
  }
  if (heading > 2 * PI) {
    heading -= 2 * PI;
  }
  
  // Convert heading to degrees
  float headingDegrees = heading * 180 / M_PI;

  // Display heading on LCD
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Heading: ");
  lcd.print(headingDegrees);
  lcd.print(" deg");

  // Display motor speed on LCD
  lcd.setCursor(0, 1);
  lcd.print("Motor Speed: ");
  lcd.print(motorSpeed);

  // Print heading and motor speed to Serial Monitor
  Serial.print("Heading: ");
  Serial.print(headingDegrees);
  Serial.print(" deg\t");
  Serial.print("Motor Speed: ");
  Serial.println(motorSpeed);

  delay(100); // Adjust delay as needed
}