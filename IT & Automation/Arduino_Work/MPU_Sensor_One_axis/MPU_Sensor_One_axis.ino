#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  mpu.initialize();

 // Set MPU6050 gyroscope range to +/- 250 degrees/sec
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);

  // Set MPU6050 accelerometer range to +/- 2g
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  
}

void loop() {
  // Read accelerometer and gyroscope data
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Convert raw accelerometer values to m/s²
  float accelX = ax / 16384.0;  // 16384 LSB/g (2g range)
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  // Convert raw gyroscope values to °/s
  float gyroX = gx / 131.0;  // 131 LSB/°/s (250°/s range)

  // Calculate pitch angle using accelerometer and gyroscope
  float accelAngleX = atan(-accelY / sqrt(pow(accelX, 2) + pow(accelZ, 2))) * 180.0 / PI;
  float gyroAngleX = gyroX * 0.01 + (1 - 0.01) * accelAngleX;

  // Print pitch angle
  Serial.print("Pitch angle (deg): ");
  Serial.println(gyroAngleX);

  delay(100);
}