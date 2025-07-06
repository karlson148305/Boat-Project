#include <LiquidCrystal.h>      //for the LCD
#include <math.h>               //for the  max and abs() functin
#include <Wire.h>               //I2C Arduino Library
#include <Adafruit_Sensor.h>    //for the magnetometer (1)
#include <Adafruit_HMC5883_U.h> //for the magnetometer (2)

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); 

const int dir2a = 6;
const int dir2b = 5;
const int enable2 = 3;

int sensorPin = A0;    // select the input pin for the potentiometer

int pwmValue ;


int desiredDirection = 0;
float heading = 0;
float headingDegrees = 0;
float gap;

void setup() 
{
  /* 1 - Initialise the sensor */
  //...
   
  /* 2 - Initialise the LCD */  
  //...
}


void loop()
{

  //
  // 1 - sensor reading
  //


  // 1.1 -- reading from the magnetometer
 

  // 1.2 -- reading from the potentiometer
 

  //
  // 2 - LCD writing
  //

  // 2.1 -- erase the previous writings

  // 2.2 -- write the current heading



  //
  // 3 - motor speed & direction
  //

  

  // 3.1 -- check if we need to move the boat or not (if the diffence is under a certain value, we probably don't need to move anymove)
  
  if ( 0 < 1 ) { // you should change the direction

    // 3.1.1 compute the pwmValue (you can use the map function https://www.arduino.cc/reference/en/language/functions/math/map/)

    //3.1.2 indicate the rotatory direction


  }
  else {

    //3.1.3 if false, set the pwmValue to zero

  }


  //
  // 4 - always send the desired speed to the dc motor
  //

  //
  // 5 - print your variables using these two functions : Serial.print("david") and  Serial.println("fasani");
  //

  //Serial.print("\theadingDegrees\t" ); Serial.print( headingDegrees);
  //Serial.print("\tcompensationDeg\t"); Serial.print( gap);
  //Serial.print("\tdesiredDirection\t" ); Serial.print( desiredDirection);
  //Serial.print("\tpwmValue\t"); Serial.println( pwmValue);


  //
  // 6 - wait for a delay
  //
  delay(5); // 5 millis could be a good value...

}
