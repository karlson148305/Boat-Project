#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column and 2 rows

// Initialize Magnetometer
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);  // Change the address if necessary

// Define motor pins
const int motorPin = 9;

// Define potentiometer pin
const int potentiometerPin = A0;

// Variables
int motorSpeed = 0;
int compassHeading = 0;

void setup() {
  // Initialize LCD
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Boat System");

  // Initialize Magnetometer
  if (!mag.begin()) {
    lcd.clear();
    lcd.print("Magnetometer");
    lcd.setCursor(0, 1);
    lcd.print("not found!");
    while (1);
  }

  // Set motor pin as output
  pinMode(motorPin, OUTPUT);

  // Set potentiometer pin as input
  pinMode(potentiometerPin, INPUT);
}

void loop() {
  // Read motor speed from potentiometer
  motorSpeed = map(analogRead(potentiometerPin), 0, 1023, 0, 255);
  analogWrite(motorPin, motorSpeed);

  // Read compass heading
  sensors_event_t event;
  mag.getEvent(&event);
  compassHeading = (int)(atan2(-event.magnetic.y, event.magnetic.x) * 180.0 / PI) + 180;

  // Display motor speed and compass heading on LCD
  lcd.clear();
  lcd.print("Speed: ");
  lcd.print(motorSpeed);
  lcd.setCursor(0, 1);
  lcd.print("Heading: ");
  lcd.print(compassHeading);
  lcd.print(" deg");

  delay(100);
}