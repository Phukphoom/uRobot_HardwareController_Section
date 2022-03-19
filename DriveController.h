#include "DCMotor.h"

#ifndef DriveController_h
#define DriveController_h

class DriveController {
  public:
    DriveController(float robotRadius, float wheelPerimeter, uint8_t motorBootPower, DCMotor* motorFront, DCMotor* motorLeft, DCMotor* motorRight, DCMotor* motorBack);
    ~DriveController();

    void moveForward(unsigned int speed);
    void moveBackward(unsigned int speed);
    void moveLeft(unsigned int speed);
    void moveRight(unsigned int speed);

    void moveForward(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveBackward(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveLeft(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveRight(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    void rotateLeft(unsigned int speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void rotateRight(unsigned int speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    void stop(DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    bool isIdle();

  private:
    float robotRadius, wheelPerimeter;
    uint8_t motorBootPower;

    DCMotor* motorFront;
    DCMotor* motorLeft;
    DCMotor* motorRight;
    DCMotor* motorBack;

    void initInfo(float robotRadius, float wheelPerimeter, uint8_t motorBootPower);
};

#endif
