#include <Servo.h>

const int potentiometerPin = A0;  // Analog pin used for the potentiometer
const int servoPin = 9;          // Pin used for the servo motor

Servo servo;  // Creating the Servo object

void setup() {
  servo.attach(servoPin);  // Attaching the servo motor to the specified pin
}

void loop() {
  int potValue = analogRead(potentiometerPin);  // Reading the potentiometer value
  int servoAngle = map(potValue, 0, 1023, 0, 180);  // Converting the potentiometer value to an angle (0-180)

  servo.write(servoAngle);  // Setting the angle of the servo motor

  delay(15);  // Delay to allow the servo motor to reach the desired position.
}