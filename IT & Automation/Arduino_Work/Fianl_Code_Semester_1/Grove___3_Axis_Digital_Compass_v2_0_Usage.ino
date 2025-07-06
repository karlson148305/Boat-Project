#include <Wire.h>
#include <Grove_Compass_Library.h>

Grove_Compass compass;

void setup()
{
  Serial.begin(9600);
  compass.init();
  compass.setMode(Continuous, ODR_10Hz);
  compass.setCalibration(0, 0, 0);
}

void loop()
{
  compass.update();

  float heading = compass.heading();
  float heading_degrees = heading * 180.0 / PI; // Convert heading to degrees

  // Determine the direction based on the heading value
  String direction;
  if (heading_degrees >= 337.5 || heading_degrees < 22.5)
  {
    direction = "North";
  }
  else if (heading_degrees >= 22.5 && heading_degrees < 67.5)
  {
    direction = "Northeast";
  }
  else if (heading_degrees >= 67.5 && heading_degrees < 112.5)
  {
    direction = "East";
  }
  else if (heading_degrees >= 112.5 && heading_degrees < 157.5)
  {
    direction = "Southeast";
  }
  else if (heading_degrees >= 157.5 && heading_degrees < 202.5)
  {
    direction = "South";
  }
  else if (heading_degrees >= 202.5 && heading_degrees < 247.5)
  {
    direction = "Southwest";
  }
  else if (heading_degrees >= 247.5 && heading_degrees < 292.5)
  {
    direction = "West";
  }
  else if (heading_degrees >= 292.5 && heading_degrees < 337.5)
  {
    direction = "Northwest";
  }

  // Print the direction and heading in the serial monitor
  Serial.print("Current direction : ");
  Serial.print(direction);
  Serial.print(" (");
  Serial.print(heading_degrees);
  Serial.println(" °.)");

  delay(500);
}