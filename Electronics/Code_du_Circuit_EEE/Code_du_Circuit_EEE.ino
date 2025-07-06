// C++ code
//
const int Button1 = 3;
const int Button2 = 5;
const int Direction1 = 10;
const int Direction2 = 11;

const int pot = A0;

void setup()
{
pinMode(Direction1, OUTPUT);
pinMode(Direction2, OUTPUT);
pinMode(Button1, INPUT);
pinMode(Button2, INPUT);
pinMode(pot, INPUT);

}

void loop()
{
int pot_value= analogRead(pot);
int Speed = map(pot_value, 0, 1023, 0 ,255);
analogWrite(9, Speed);
  
if (digitalRead(Button1) == HIGH){
analogWrite(Direction1, HIGH);
analogWrite(Direction2, LOW);
// delay(5000); // Wait for 1000 millisecond(s)
}
else if (digitalRead(Button2) == HIGH){
analogWrite(Direction1, LOW);
analogWrite(Direction2, HIGH);
//delay(5000); // Wait for 1000 millisecond(s)
}
}
