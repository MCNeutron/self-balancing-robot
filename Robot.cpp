#include "Robot.h"

// Constructor
Robot::Robot() {
  // Empty
}

// PID controller, returns motor PWM for balancing robot
double Robot::PID(float currentRoll, float currentPitch, float currentYaw) {

  // Update roll, pitch, yaw values
  updateAngles(currentRoll, currentPitch, currentYaw);

  // Calculate time step
  unsigned long currentTime = millis(); // Make sure the call to obtain IMU readings is not too far from this call to obtain current time
  unsigned long timeStep = currentTime - this->prevTime;

  // Calculate error and cumulative error
  float error = this->roll - this->targetRoll;
  this->errorSum = this->errorSum + (error * timeStep);

  // Calculate control action, u
  double u = this->Kp * error + this->Ki * this->errorSum + this->Kd * ((error - this->prevError) / timeStep);
//  Serial.print((String)"error: " + error);
//  Serial.print((String)" || error-prevError: " + (error-this->prevError));
//  Serial.println((String)" || timestep: " + timeStep);

  // Update variables for next control loop
  this->prevTime = currentTime;
  this->prevError = error;

  // Return the control action, u
  return u;
  
}

// Updates attitude angle variables of the robot
void Robot::updateAngles(float currentRoll, float currentPitch, float currentYaw) {

  // Update the current angle values
  this->roll = currentRoll;
  this->pitch = currentPitch;
  this->yaw = currentYaw;
  
}

// Safety checks to prevent potential damage before sending final control action to motors
double Robot::safetyChecks(double u) {

  if (u > 0) { u = u + 0; } //30
  else if (u < 0) { u = u - 0; }

  // Constrain motor input PWM to prevent excessive motor commands
  u = constrain(u, -150, 150);

  // Deadzone - Ignore small control inputs that cause whining in the motors but do not move the robot anyways
  //if (abs(u) <= 30) { u = 0; }

  return u;

//  // Turn motors off when robot pitch exceeds 45 degrees. This is probably too far from saving the robot
//  if (abs(this->pitch) >= 30) { this->errorState = 1; } // Change error state to indicate excessive pitch
//
//  // Keep error state 1 active (which keeps motors off) until robot returned to upright position
//  if (this->errorState == 1 && abs(this->pitch) < 5) { this->errorState = 0; }
//
//  // Update control action if needed
//  if (this->errorState == 1) { return 0; } // 0 control action i.e. stop motors
//  else { // Original control action i.e. no change, but with a rate limiter
//
//    // The control action, u, will be applied on top of the stop PWM
//    u = constrain(u, -5, 5);
//  
//    // Deadzone - Ignore small control inputs to avoid motor jittering
//    if (abs(u) <= 4) {
//      u = 0;
//    }
//
////    // Rate limiter on control action, to prevent changes that are too quick, which may damage motor
////    double uChange = constrain(u - this->prevU, -5, 5);
////    double outU = this->prevU + uChange;
////    this->prevU = outU;
////    
////    return outU;
//
//    return u;
//    
//  }
  
}
