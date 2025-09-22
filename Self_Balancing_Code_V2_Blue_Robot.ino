#include "IMU.h"
#include "Motor.h"
#include "Robot.h"

// Create objects
IMU imu;
Motor motor;
Robot robot;

void setup() {
  // put your setup code here, to run once:
  imu.init();
  motor.init();

}

void loop() {
  // put your main code here, to run repeatedly:
  imu.update();
  double u = robot.PID(imu.getRoll(), imu.getPitch(), imu.getYaw());
//  Serial.print((String)"u: " + u + " || ");
  u = robot.safetyChecks(u);
//  Serial.println((String)"u after safety: " + u);
  motor.drive(-u, -u); // Control action sign should be flipped for robot to adjust in correct direction when tilted, tested experimentally
  delay(10);

//  delay(100);
//  motor.drive(0, 0);
//  delay(1000);
//  motor.drive(-30, 30);
//  delay(500);
}
