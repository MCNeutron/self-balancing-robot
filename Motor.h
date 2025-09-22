#include <Arduino.h>

class Motor {

  private:
    // Motor pins
    const int AIN1 = 6;
    const int AIN2 = 5;
    const int BIN1 = 10;
    const int BIN2 = 9;

    // Motor state variables
    int leftPWM = 0;
    int rightPWM = 0;

  public:
    Motor(); // Constructor
    void init(); // Motor initialisation code
    void drive(int, int); // Set motor PWM commands
  
};
