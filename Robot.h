#include <Arduino.h>

class Robot {

  private:
    int roll = 0;
    int targetRoll = 0;
    int pitch = 0;
    int yaw = 0;

    // PID
    // Decent gains: Kp = 33, Ki = 0, Kd = 0
    float Kp = 33;
    float Ki = 0.3;//0.0000001;
    float Kd = 0;//0.1
    unsigned long prevTime = 0;
    float prevError = 0;
    float errorSum = 0;

    // State indicators
    int errorState = 0;
    double prevU = 0;

  public:
    Robot(); // Constructor
    double PID(float, float, float); // PID controller for robot balancing
    void updateAngles(float, float, float); // Updates the attitude angle variables of the robot
    double safetyChecks(double); // Safety checks to prevent potential damage before sending final control action to motors
  
};
