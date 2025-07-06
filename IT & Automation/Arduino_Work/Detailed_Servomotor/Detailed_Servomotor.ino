#include <Servo.h>

const int potentiometerPin = A0;  // Analog pin used for the potentiometer
const int servoPin = 9;          // Pin used for the servo motor

Servo servo;  // Creating the Servo object

int minAngle = 0;     // Minimum angle for the servo motor
int maxAngle = 180;   // Maximum angle for the servo motor
int stopAngle = 90;   // Angle at which the servo motor stops rotating

void setup() {
  servo.attach(servoPin);  // Attaching the servo motor to the specified pin
  servo.write(stopAngle);  // Setting the initial position of the servo motor to the stop angle
}

void loop() {
  int potValue = analogRead(potentiometerPin);  // Reading the potentiometer value
  int servoAngle;

  if (potValue < 512) {
    // Rotate to the left
    servoAngle = map(potValue, 0, 511, minAngle, stopAngle);
  } else {
    // Rotate to the right
    servoAngle = map(potValue, 512, 1023, stopAngle, maxAngle);
  }

  servo.write(servoAngle);  // Setting the angle of the servo motor

  delay(15);  // Delay to allow the servo motor to reach the desired position
}