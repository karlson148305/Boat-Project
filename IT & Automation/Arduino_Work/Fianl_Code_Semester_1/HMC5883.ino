#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345); // Adresse I2C du capteur (12345 est un exemple)

void setup() {
  Serial.begin(9600);
  // Initialise le capteur magnétomètre
  if (!mag.begin()) {
    Serial.println("Erreur de connexion avec le magnétomètre !");
    while (1);
  }
}

void loop() {
  // Obtenir une nouvelle mesure du magnétomètre
  sensors_event_t event;
  mag.getEvent(&event);

  // Afficher les valeurs de champ magnétique sur les axes X, Y et Z
  Serial.print("X: ");
  Serial.print(event.magnetic.x);
  Serial.print("  Y: ");
  Serial.print(event.magnetic.y);
  Serial.print("  Z: ");
  Serial.print(event.magnetic.z);
  Serial.println(" uT");

  delay(1000); // Attendre 1 seconde avant de prendre une nouvelle mesure
}