// For reference, the code originates from:
// Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial by Dejan, https://howtomechatronics.com
// It has good code explanations and other useful resources

#include "IMU.h"
#include <Wire.h>

// Constructor
IMU::IMU() {
  // Empty
}

// IMU initialisation
void IMU::init() {
  
  Serial.begin(19200);
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(this->MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  /*
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10);                  //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10);                   // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  */
  
  // Call this function if you need to get the IMU error values for your module
  calcIMUError();
  delay(20);
  
}

// Get IMU readings
void IMU::update() {
  
  // === Read acceleromter data === //
  Wire.beginTransmission(this->MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(this->MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  this->AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  this->AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  this->AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  
  // Calculating Roll and Pitch from the accelerometer data
  this->accAngleX = (atan(this->AccY / sqrt(pow(this->AccX, 2) + pow(this->AccZ, 2))) * 180 / PI) - this->AccErrorX;
  this->accAngleY = (atan(-1 * this->AccX / sqrt(pow(this->AccY, 2) + pow(this->AccZ, 2))) * 180 / PI) - this->AccErrorY;
  
  // === Read gyroscope data === //
  this->previousTime = this->currentTime;        // Previous time is stored before the actual time read
  this->currentTime = millis();            // Current time actual time read
  this->elapsedTime = (this->currentTime - this->previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(this->MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(this->MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  this->GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  this->GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  this->GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  
  // Correct the outputs with the calculated error values
  this->GyroX = this->GyroX - this->GyroErrorX;
  this->GyroY = this->GyroY - this->GyroErrorY;
  this->GyroZ = this->GyroZ - this->GyroErrorZ;
  
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  this->gyroAngleX = this->gyroAngleX + this->GyroX * this->elapsedTime; // deg/s * s = deg
  this->gyroAngleY = this->gyroAngleY + this->GyroY * this->elapsedTime;
  this->yaw =  this->yaw + this->GyroZ * this->elapsedTime;
  
  // Complementary filter - combine accelerometer and gyro angle values
  this->roll = 0.96 * this->gyroAngleX + 0.04 * this->accAngleX;
  this->pitch = 0.96 * this->gyroAngleY + 0.04 * this->accAngleY;

  // Update angle calculations from gyroscope to the output of the complentary filter
  this->gyroAngleX = this->roll;
  this->gyroAngleY = this->pitch;
  
  // Print the values on the serial monitor
//  Serial.print(this->roll);
//  Serial.print("/");
//  Serial.print(this->pitch);
//  Serial.print("/");
//  Serial.println(this->yaw);
  
}

// Calculate IMU calibration error
void IMU::calcIMUError() {
  
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (this->c < 200) {
    Wire.beginTransmission(this->MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(this->MPU, 6, true);
    this->AccX = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    this->AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    this->AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    // Sum all readings
    this->dispAccErrorX = this->dispAccErrorX + ((atan((this->AccY) / sqrt(pow((this->AccX), 2) + pow((this->AccZ), 2))) * 180 / PI));
    this->dispAccErrorY = this->dispAccErrorY + ((atan(-1 * (this->AccX) / sqrt(pow((this->AccY), 2) + pow((this->AccZ), 2))) * 180 / PI));
    this->c++;
  }
  
  //Divide the sum by 200 to get the error value
  this->dispAccErrorX = this->dispAccErrorX / 200;
  this->dispAccErrorY = this->dispAccErrorY / 200;
  this->c = 0;
  
  // Read gyro values 200 times
  while (this->c < 200) {
    Wire.beginTransmission(this->MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(this->MPU, 6, true);
    this->GyroX = Wire.read() << 8 | Wire.read();
    this->GyroY = Wire.read() << 8 | Wire.read();
    this->GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    this->dispGyroErrorX = this->dispGyroErrorX + (this->GyroX / 131.0);
    this->dispGyroErrorY = this->dispGyroErrorY + (this->GyroY / 131.0);
    this->dispGyroErrorZ = this->dispGyroErrorZ + (this->GyroZ / 131.0);
    this->c++;
  }
  
  //Divide the sum by 200 to get the error value
  this->dispGyroErrorX = this->dispGyroErrorX / 200;
  this->dispGyroErrorY = this->dispGyroErrorY / 200;
  this->dispGyroErrorZ = this->dispGyroErrorZ / 200;
  
  // Print the error values on the Serial Monitor
  Serial.print("dispAccErrorX: ");
  Serial.println(this->dispAccErrorX);
  Serial.print("dispAccErrorY: ");
  Serial.println(this->dispAccErrorY);
  Serial.print("dispGyroErrorX: ");
  Serial.println(this->dispGyroErrorX);
  Serial.print("dispGyroErrorY: ");
  Serial.println(this->dispGyroErrorY);
  Serial.print("dispGyroErrorZ: ");
  Serial.println(this->dispGyroErrorZ);
  
}

// Get roll in deg
float IMU::getRoll() {
//  Serial.println((String)"Roll: " + this->roll);
  return this->roll;
}

// Get pitch in deg
float IMU::getPitch() {
  return this->pitch;
}

// Get yaw in deg
float IMU::getYaw() {
  return this->yaw;
}
