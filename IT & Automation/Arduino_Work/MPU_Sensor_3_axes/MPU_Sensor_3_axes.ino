
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;


void setup() {
  Serial.begin(9600);

  Wire.begin();

 // Set MPU6050 gyroscope range to +/- 250 degrees/sec
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);

  // Set MPU6050 accelerometer range to +/- 2g
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
}

void loop() {
  //Read accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer values to m/s²
  float accelX = ax / 16384.0;  // 16384 LSB/g (2g range)
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Calculate pitch and roll angles
  float accelAngleX = atan(-accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;
  float accelAngleY = atan(accelX / sqrt(pow(accelY, 2) + pow(accelZ, 2))) * 180.0 / PI;
  

  // Print angles
  Serial.print("Accelerometer angles (deg): ");
  Serial.print(accelAngleX);
  Serial.print(", ");
  Serial.println(accelAngleY);

  delay(100);
}