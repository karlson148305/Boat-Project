// Motor control pins
const int motorPin1 = 5;   // Connect to motor pin 1
const int motorPin2 = 6;   // Connect to motor pin 2

// Potentiometer pin
const int potPin = A0;     // Connect the potentiometer to analog pin A0

void setup() {
  // Set motor pins as output
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(Pot_Pin, INPUT);
}

void loop() {
  // Read the value from the potentiometer
  int potValue = analogRead(potPin);

  // Map the potentiometer value to a range of motor directions
  int motorDirection = map(potValue, 0, 1023, 0, 255);

  // Set the motor direction based on the potentiometer value
  if (motorDirection > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  }
  else if (motorDirection < 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  }
  else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}