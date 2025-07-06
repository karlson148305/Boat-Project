#include <LiquidCrystal.h>      //For the LCD
#include <LiquidCrystal_I2C.h>  //For the LCD_I2C
#include <math.h>               //For the  max and abs() functin
#include <Wire.h>               //I2C Arduino Library
#include <Adafruit_Sensor.h>    //For the magnetometer (1)
#include <Adafruit_HMC5883_U.h> //For the magnetometer (2)
#include <Grove_Compass_Library.h> //For the magnetometer (3)

Grove_Compass compass;

//* Assign a unique ID to this sensor at the same time.
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to.
LiquidCrystal lcd(7, 8, 9, 10, 11, 12); 
// Set the LCD address to 0x27 for a 16 chars and 2 line display.
LiquidCrystal_I2C lcd(0x27, 16, 2);

const int dir1a = 5; //Connect the In1a of the Motor_Driver to the pin 5 of the arduino board.
const int dir1b = 6; //Connect the In1b of the Motor_Driver to the pin 6 of the arduino board.
const int Enable1 = 3; //Connect the ena1 of the Motor_Driver to the pin 3 of the arduino board.

int Pot_Pin = A0;    // Set the input pin for the potentiometer.

int PWM_Value = 255; //Set the PWM_Value to max or half, as you want.

int Desired_Direction_By_Pot = 50;
float Heading = 0; //Variable for the magnetometer readings in radians.
float Heading_Degrees = 0; //Variable for the magnetometer readings in degrees.
float gap;

void setup() 
{
  Serial.begin(9600); // Set the baud rate of the Serial monitor.

  pinMode(Pot_Pin, INPUT); // Configure the Potentiometer Pin as Input.

  pinMode(dir1a, OUTPUT); // Configure the direction control(In1a) of one motor of the L298N as Output.
  pinMode(dir1b, OUTPUT); // Configure the direction control(In1b) of one motor of the L298N as Output.
  pinMode(Enable1, OUTPUT); // Configure the ena1 of one motor of the L298N as Output.
  
  // Initialize Magnetometer
  compass.init();
  compass.setMode(Continuous, ODR_10Hz);
  compass.setCalibration(0, 0, 0);

  // Initialize LCD I2C
  lcd.begin(16, 2);
  lcd.backlight();
}


void Magnetometer_Sensor()
{
  compass.update();

  float heading = compass.heading();
  float heading_degrees = heading * 180.0 / PI; // Convert heading to degrees

  // Determine the direction based on the heading value
  String direction;
  if (heading_degrees >= 0 || heading_degrees < 22.5)
  {
    direction = "N";
  }
  else if (heading_degrees >= 22.5 && heading_degrees < 45)
  {
    direction = "N.N_E";
  }
  else if (heading_degrees >= 45 && heading_degrees < 67.5)
  {
    direction = "N_E";
  }
  else if (heading_degrees >= 67.5 && heading_degrees < 90)
  {
    direction = "E.N_E";


  }
  else if (heading_degrees >= 90 && heading_degrees < 112.5)
  {
    direction = "E";
  }
  else if (heading_degrees >= 112.5 && heading_degrees < 135)
  {
    direction = "E.S_E";
  }
  else if (heading_degrees >= 135 && heading_degrees < 157.5)
  {
    direction = "S_E";
  }
  else if (heading_degrees >= 157.5 && heading_degrees < 180)
  {
    direction = "S.S_E";
  }


  else if (heading_degrees >= 180 && heading_degrees < 202.5)
  {
    direction = "S";
  }
  else if (heading_degrees >= 202.5 && heading_degrees < 225)
  {
    direction = "S.S_W";
  }
  else if (heading_degrees >= 225 && heading_degrees < 247.5)
  {
    direction = "S_W";
  }
  else if (heading_degrees >= 247.5 && heading_degrees < 270)
  {
    direction = "W.S_W";
  }


  else if (heading_degrees >= 270 && heading_degrees < 292.5)
  {
    direction = "W";
  }
  else if (heading_degrees >= 292.5 && heading_degrees < 315)
  {
    direction = "W.N_W";
  }
  else if (heading_degrees >= 315 && heading_degrees < 337.5)
  {
    direction = "N_W";
  }
  else if (heading_degrees >= 337.5 && heading_degrees < 360)
  {
    direction = "N.N_W";
  }

  // Print the direction and heading in the serial monitor
  Serial.print("Current direction : ");
  Serial.print(direction);
  Serial.print(" (");
  Serial.print(heading_degrees);
  Serial.println(" °.)");

  delay(500);
}


void Potentiometer_Sensor()
{
  // Read the value from the potentiometer
  int Pot_Value = analogRead(Pot_Pin);

  // Map the potentiometer value to a range of motor directions
  int Desired_Direction_By_Pot = map(Pot_Value, 0, 1023, 0, 255);

  // Set the motor direction based on the potentiometer value
  if (Desired_Direction_By_Pot > 50) 
  {
    digitalWrite(dir1a, HIGH);
    digitalWrite(dir1b, LOW);
    lcd.setCursor(0, 1);
    lcd.print("D_D: ");
    lcd.print(To_the_Left);
  }
  else if (Desired_Direction_By_Pot < 50) 
  {
    digitalWrite(dir1a, LOW);
    digitalWrite(dir1b, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("D_D: ");
    lcd.print(To_the_Right);
  }
  else 
  {
    digitalWrite(dir1a, LOW);
    digitalWrite(dir1b, LOW);
    lcd.setCursor(0, 1);
    lcd.print("D_D: ");
    lcd.print(Not_Chosen.);
  }
}


void loop()
{
  // 1 - Sensor reading
  Magnetometer_Sensor() ; // This void is for the Magnetometer reading and based on these informations, decisions are taken.

  Potentiometer_Sensor() ; // This void is for the Potentiometer reading and based on these informations, decisions are taken.
 

  // 2 - LCD writing
  // Display heading on LCD
  lcd.setCursor(0, 0); // Set cursor to row 0 i.e at the first bloc and column 0 i.e also at the first bloc.
  lcd.print("Direction: "); // Print the information to precise what is displayed.
  lcd.print(direction); //Display the direction i.e N, N_W or S.
 

  // 3 - motor speed & direction
  // 3.1 -- check if we need to move the boat or not (if the diffence is under a certain value, we probably don't need to move anymove)
  
  if (heading_degrees >= 180 && heading_degrees < 202.5)
   {
    // 3.1.1 compute the PWM_Value (you can use the map function https://www.arduino.cc/reference/en/language/functions/math/map/)
     digitalWrite(dir1a, LOW);
     digitalWrite(dir1b, LOW);
     analogWrite(Enable1, PWM_Value);
   }

  else if (heading_degrees < 180)
   {
    digitalWrite(dir1a, LOW);
    digitalWrite(dir1b, HIGH);
    analogWrite(Enable1, PWM_Value);
   }

else if (heading_degrees > 202.5)
   {
    digitalWrite(dir1a, HIGH);
    digitalWrite(dir1b, LOW);
    analogWrite(Enable1, PWM_Value);
   }

  //
  // 4 - always send the desired speed to the dc motor
  //

  //
  // 5 - print your variables using these two functions : Serial.print("david") and  Serial.println("fasani");
  //

  Serial.print("\tHeading_Degrees\t" ); 
  Serial.print(Heading_Degrees);
  Serial.print("\tCompensation_Deg\t"); 
  Serial.print(gap);
  Serial.print("\tDesired_Direction\t" ); 
  Serial.print(Desired_Direction);
  Serial.print("\tPWM_Value\t"); 
  Serial.println(PWM_Value);

  delay(5); // 5 millis could be a good value...

}
