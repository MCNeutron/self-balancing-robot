#include "Motor.h"

// Constructor
Motor::Motor() {
  // Empty
}

// Motor initialisation code
void Motor::init() {

  pinMode(this->AIN1, OUTPUT);
  pinMode(this->AIN2, OUTPUT);
  pinMode(this->BIN1, OUTPUT);
  pinMode(this->BIN2, OUTPUT);
  
}

// Set motor PWM commands
void Motor::drive(int pwmA, int pwmB) {

  // -ve PWM = forward spin, +ve PWM = backward spin
  // pwmA command sent to motor A, pwmB command sent to motor B
  if (pwmA >= 0) {
    digitalWrite(AIN2, HIGH);
    analogWrite(AIN1, 255-pwmA);
  }
  else {
    digitalWrite(AIN1, HIGH);
    analogWrite(AIN2, 255+pwmA);
  }

  if (pwmB >= 0) {
    digitalWrite(BIN2, HIGH);
    analogWrite(BIN1, 255-pwmB);
  }
  else {
    digitalWrite(BIN1, HIGH);
    analogWrite(BIN2, 255+pwmB);
  }
  
}
