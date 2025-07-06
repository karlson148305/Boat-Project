#include <Servo.h>

Servo ESC;
int valeur;

void setup() {
  // put your setup code here, to run once:
  ESC.attach(9, 1000, 2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  valeur= analogRead(A0);
  valeur= map(valeur, 0, 1023, 0, 180);
  ESC.write(valeur);
}