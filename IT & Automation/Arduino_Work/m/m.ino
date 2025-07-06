#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <PID_v1.h>

LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_MPU6050 mpu;

const int MPU_addr = 0x68;
const int potentiometerPin = A0;  // Analog pin used for the potentiometer
const int servoPin = 9;          // Pin used for the servo motor
const int bouton_save = 5;

int16_t Ang_x, Ang_y, Ang_z, Angle_x, Angle_y, Angle_z, angle_x, angle_y, angle_z, Tmp, GyX, GyY, GyZ;
int Ang, Angle_initial, Angle_final, dep, new_servoAngle, new_dep;
int OldservoAngle = 0;
int potValue = 0;
int servoAngle = 0;

Servo servo;  // Creating the Servo object

double setpoint = 0; // Valeur de consigne (angle souhaité)
double input = 0; // Valeur de retour (angle mesuré)
double output = 0; // Valeur de sortie (angle du servo)
double Kp = 1.0; // Coefficient proportionnel
double Ki = 0.0; // Coefficient intégral
double Kd = 0.0; // Coefficient dérivatif
PID servoPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  servo.attach(servoPin);  // Attaching the servo motor to the specified pin
  
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
  lcd.print("Groupe 7");
  delay(2000);
  lcd.clear();
  lcd.setCursor(2,0);
  lcd.print("Starting.");
  delay(500);
  lcd.setCursor(2,0);
  lcd.print("Starting..");
  delay(500);
  lcd.setCursor(2,0);
  lcd.print("Starting...");
  delay(500);
  lcd.clear();
  
  servoPID.SetMode(AUTOMATIC); // Initialiser le PID en mode automatique
}

void loop() {
  OldservoAngle = servoAngle;
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);  
  
  potValue = analogRead(potentiometerPin);  // Reading the potentiometer value
  servoAngle = map(potValue, 0, 1023, 0, 180);  // Converting the potentiometer value to an angle (0-180)

  input = angle_z; // Mettre à jour la valeur de l'entrée PID avec l'angle mesuré
  servoPID.Compute(); // Calculer la valeur de sortie du PID
  servo.write(output); // Définir l'angle du servo motor avec la valeur de sortie du PID

  delay(15);  // Delay to allow the servo motor to reach the desired position.

  Angle_x = Wire.read() << 8 | Wire.read();
  Angle_y = Wire.read() << 8 | Wire.read();
  Angle_z = Wire.read() << 8 | Wire.read();

  angle_x = RAD_TO_DEG * (atan2(-Angle_y, -Angle_z) + PI);
  angle_y = RAD_TO_DEG * (atan2(-Angle_x, -Angle_z) + PI);
  angle_z = RAD_TO_DEG * (atan2(-Angle_y, -Angle_x) + PI);

  Serial.print("Angle x= ");
  Serial.println(angle_x);

  Serial.print("Angle y= ");
  Serial.println(angle_y);

  Serial.print("Angle z= ");
  Serial.println(angle_z);

  if (servoAngle - OldservoAngle > 3) {
    Angle_initial = angle_z;
    Serial.print("Angle_initial = ");
    Serial.println(Angle_initial);
  } else if (servoAngle - OldservoAngle < -3) {
    Angle_initial = angle_z;
    Serial.print("Angle_initial = ");
    Serial.println(Angle_initial);
  }

  Angle_final = angle_z - Angle_initial;

  Serial.print("Angle_final = ");
  Serial.println(Angle_final);
  Serial.println(" ");

  lcd.setCursor(0,0);
  lcd.print("Angle final: ");
  lcd.print(Angle_final);
  lcd.print("   ");

  lcd.setCursor(0,1);
  lcd.print("Servo angle: ");
  lcd.print(servoAngle);
  lcd.print("   ");

  Serial.print("Angle final: ");
  Serial.print(Angle_final);
  Serial.println(" ");

  Serial.print("Servo angle: ");
  Serial.print(servoAngle);
  Serial.println("   ");  
}