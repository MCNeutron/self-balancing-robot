#include <Arduino.h>

class IMU {
  
  private:
    const int MPU = 0x68; // MPU6050 I2C address
    float AccX, AccY, AccZ; // Accelerometer readings
    float GyroX, GyroY, GyroZ; // Gyroscope readings
    float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; // Angle calculations using accelerometer and gyro readings
    float roll, pitch, yaw; // Final roll, pitch and yaw values
    float dispAccErrorX, dispAccErrorY, dispGyroErrorX, dispGyroErrorY, dispGyroErrorZ; // Calibration error for accelerometer and gyroscope
    float elapsedTime, currentTime, previousTime; // For calculating angles using gyro readings
    int c = 0; // For calibration error code
    
    float AccErrorX = -0.71; // AccErrorX ~(-1.68) See the calculate_IMU_error()custom function for more details
    float AccErrorY = -0.95; // AccErrorY ~(-0.73)
    float GyroErrorX = -0.73; // GyroErrorX ~(-0.77)
    float GyroErrorY = 0.18; // GyroErrorY ~(0.19)
    float GyroErrorZ = 0.31; // GyroErrorZ ~ (0.26)

  public:
    IMU(); // Constructor
    void init(); // IMU initialisation code
    void update(); // Obtain new IMU readings
    void calcIMUError(); // Calculate IMU error calibration values
    float getRoll(); // Return the roll in deg
    float getPitch(); // Return in pitch in deg
    float getYaw(); // Return the yaw in deg
  
};
