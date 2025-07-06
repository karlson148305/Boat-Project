
const int dir1a = 6;
const int dir1b = 5;
const int Enable1 = 3;

int PWM_Value = 255;

void setup() {
  // put your setup code here, to run once:
  pinMode(dir1a, OUTPUT);
  pinMode(dir1b, OUTPUT);
  pinMode(Enable1, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
digitalWrite(dir1a, HIGH);
digitalWrite(dir1b, LOW);
analogWrite(Enable1, PWM_Value);
delay(3000);

digitalWrite(dir1a, LOW);
digitalWrite(dir1b, HIGH);
analogWrite(Enable1, PWM_Value);
delay(3000);
}
