#include <Wire.h>
#include <Grove_Compass_Library.h>

Grove_Compass compass;

int calibrationData[3][2] = {{32767, -32768},
                             {32767, -32768},
                             {32767, -32768}};

bool changed = false;
bool done = false;
unsigned long t = 0;
unsigned long c = 0;

void setup()
{
  Serial.begin(9600);
  compass.init();

  Serial.println("This will provide calibration settings for your Grove - 3-Axis Digital Compass v2.0 module. When prompted, move the compass in all directions until the calibration is complete.");
  Serial.println("Calibration will begin in 5 seconds.");
  delay(5000);

  c = millis();
}

void loop()
{
  int x, y, z;

  delay(100);

  compass.read();
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  changed = false;

  if (x < calibrationData[0][0])
  {
    calibrationData[0][0] = x;
    changed = true;
  }
  if (x > calibrationData[0][1])
  {
    calibrationData[0][1] = x;
    changed = true;
  }
  if (y < calibrationData[1][0])
  {
    calibrationData[1][0] = y;
    changed = true;
  }
  if (y > calibrationData[1][1])
  {
    calibrationData[1][1] = y;
    changed = true;
  }
  if (z < calibrationData[2][0])
  {
    calibrationData[2][0] = z;
    changed = true;
  }
  if (z > calibrationData[2][1])
  {
    calibrationData[2][1] = z;
    changed = true;
  }

  if (changed && !done)
  {
    Serial.println("CALIBRATING... Keep moving your sensor around.");
    c = millis();
  }

  t = millis();

  if ((t - c > 10000) && !done)
  {
    done = true;
    Serial.println("DONE. Copy the line below and paste it into your project's sketch.");
    Serial.println();
    Serial.print("compass.setCalibration(");
    Serial.print(calibrationData[0][0]);
    Serial.print(", ");
    Serial.print(calibrationData[0][1]);
    Serial.print(", ");
    Serial.print(calibrationData[1][0]);
    Serial.print(", ");
    Serial.print(calibrationData[1][1]);
    Serial.print(", ");
    Serial.print(calibrationData[2][0]);
    Serial.print(", ");
    Serial.print(calibrationData[2][1]);
    Serial.println(");");
  }

  // Your main code logic here
}