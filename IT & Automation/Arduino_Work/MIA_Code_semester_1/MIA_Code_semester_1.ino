#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Servo servo_2; // Déclaration de l'objet Servo pour contrôler le servomoteur
LiquidCrystal_I2C lcd(0x27, 16, 2); // Déclaration de l'objet LiquidCrystal_I2C pour contrôler l'écran LCD
int pinpotentio2 = A1; // Broche analogique utilisée pour le deuxième potentiomètre
int pinpotentio1 = A0; // Broche analogique utilisée pour le premier potentiomètre
int b; // Variable non utilisée

Adafruit_MPU6050 mpu; // Déclaration de l'objet MPU6050 pour communiquer avec le MPU6050

void setup() {
  servo_2.attach(9); // On attache le servomoteur à la broche 9
  lcd.init(); // Initialisation de l'écran LCD
  Serial.begin(9600); // Initialisation de la communication série à une vitesse de 115200 bauds
  while (!Serial)
    delay(10); // Pause l'exécution jusqu'à ce que la console série soit ouverte

  Serial.println("Adafruit MPU6050 test!");

  // Tentative d'initialisation du MPU6050
  /*if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }*/
  Serial.println("MPU6050 Found!");

  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ); // Configuration de la bande passante du MPU6050
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
    case MPU6050_BAND_260_HZ:
      Serial.println("260 Hz");
      break;
    case MPU6050_BAND_184_HZ:
      Serial.println("184 Hz");
      break;
    case MPU6050_BAND_94_HZ:
      Serial.println("94 Hz");
      break;
    case MPU6050_BAND_44_HZ:
      Serial.println("44 Hz");
      break;
    case MPU6050_BAND_21_HZ:
      Serial.println("21 Hz");
      break;
    case MPU6050_BAND_10_HZ:
      Serial.println("10 Hz");
      break;
    case MPU6050_BAND_5_HZ:
      Serial.println("5 Hz");
      break;
  }

  Serial.println("");
  delay(100);
}

void loop() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp); // Lecture des données d'accélération, de gyroscope et de température du MPU6050
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);

  int valeurPotentio1 = analogRead(pinpotentio1); // Lecture de la valeur du premier potentiomètre
  int valeurPotentio2 = analogRead(pinpotentio2); // Lecture de la valeur du deuxième potentiomètre

  int angle1 = map(valeurPotentio1, 0, 1023, 0, 180); // Conversion de la valeur du premier potentiomètre en angle
  int angle2 = map(valeurPotentio2, 0, 1023, 0, 180); // Conversion de la valeur du deuxième potentiomètre en angle

  servo_2.write(angle1); // Déplacement du servomoteur à l'angle du premier potentiomètre
  lcd.setCursor(0, 0);
  lcd.print(angle1); // Affichage de l'angle sur l'écran LCD

  // Ajout de la logique de l'autopilote
  float capSouhaite = map(angle1, 0, 180, -90, 90); // Conversion de l'angle du premier potentiomètre en consigne de cap
  float capActuel = g.gyro.z * (180.0 / PI); // Lecture du cap actuel à partir du gyroscope

  float correction = capSouhaite - capActuel; // Calcul de la correction à appliquer
  int correctionAngle = map(correction, -90, 90, -45, 45); // Conversion de la correction en angle de servo

  servo_2.write(angle1 + correctionAngle); // Déplacement du servomoteur à l'angle du premier potentiomètre avec correction

  if (angle2 < 90) {
    lcd.backlight(); // Allumer le rétroéclairage de l'écran LCD si l'angle du deuxième potentiomètre est inférieur à 90
  } else {
    lcd.noBacklight(); // Éteindre le rétroéclairage de l'écran LCD sinon
}
}