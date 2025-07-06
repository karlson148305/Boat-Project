#include <Servo.h>    //Import de la bibliotheque servo.h pour l'utilisation du servomoteur
#include <Wire.h>     //Import de la bibliotheque wire.h pour la communi
#include <LiquidCrystal_I2C.h>

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_MPU6050 mpu;

const int MPU_addr=0x68;
const int Potentiometer_Pin = A0;  // Analog pin used for the potentiometer
const int Servo_Pin = 9;          // Pin used for the servo motor

int16_t Acc_x,Acc_y,Acc_z,Readings_x,Readings_y,Readings_z,Angle_x,Angle_y,Angle_z,Tmp,GyX,GyY,GyZ ;

Servo servo;  // Creating the Servo object

void setup() {
  servo.attach(Servo_Pin);  // Attaching the servo motor to the specified pin
  
  lcd.init();
  lcd.backlight();
  lcd.clear();  

  Serial.begin(9600);
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);  

  lcd.setCursor(4,0);
  lcd.print("Groupe 1");
  lcd.setCursor(2,0);
  lcd.print("Presentation");
  delay(2000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting...");
  delay(500);

   lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting...");
  delay(500);

   lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting...");
  delay(500);

     lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait...");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait...");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait...");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait...");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Please wait...");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting");
  delay(1000);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting.");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting..");
  delay(500);

  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting...");
  delay(500);
  
  lcd.clear();
}

void loop()   

{
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  
  
  int Pot_Value = analogRead(Potentiometer_Pin);  // Reading the potentiometer value
  int Servo_Angle = map(Pot_Value, 0, 1023, 0, 180);  // Converting the potentiometer value to an angle (0-150)

  servo.write(Servo_Angle);  // Setting the angle of the servo motor

  delay(500);  // Delay to allow the servo motor to reach the desired position.

  Readings_x = Wire.read()<<8|Wire.read();
  Readings_y = Wire.read()<<8|Wire.read();
  Readings_z = Wire.read()<<8|Wire.read();

// Convert raw accelerometer values to m/s²
  float Acc_x = Readings_x / 16384.0;  // 16384 LSB/g (2g range)
  float Acc_y = Readings_y / 16384.0;
  float Acc_z = Readings_z / 16384.0;

  float Angle_z = atan(Acc_x / sqrt(pow(Acc_y, 2) + pow(Acc_z, 2))) * (180.0 / PI);

  Serial.print("Angle z = ");
  Serial.println(Angle_z);

  lcd.setCursor(0,0);
  lcd.print("Initial = ");
  lcd.println(Servo_Angle);
  lcd.print((char)223);

  lcd.setCursor(0,1);
  lcd.print("Gyro = ");
  lcd.println(Angle_z);
  lcd.print((char)223);
     
  
  if (Servo_Angle < Angle_z)
{
  int Rotation = Angle_z - Servo_Angle;
 
  // Move the servo to the difference angle
  int Target_Angle = servo.read() + Rotation;
  servo.write(Target_Angle);
}

  else if (Servo_Angle > Angle_z)
{
  int Rotation = Servo_Angle - Angle_z;

  // Move the servo to the difference angle
  int Target_Angle = servo.read() - Rotation;
  servo.write(Target_Angle);
}

}