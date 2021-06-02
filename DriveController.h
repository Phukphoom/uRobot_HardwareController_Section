#include "DCMotor.h"

#ifndef DriveController_h
#define DriveController_h

class DriveController {
   public:
    DriveController(float robotRadius, float wheelPerimeter, DCMotor* motorFront, DCMotor* motorLeft, DCMotor* motorRight, DCMotor* motorBack);
    ~DriveController();

    void moveForward(uint8_t speed);
    void moveBackward(uint8_t speed);
    void moveLeft(uint8_t speed);
    void moveRight(uint8_t speed);

    void moveForward(uint8_t speed, float distance,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveBackward(uint8_t speed, float distance,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveLeft(uint8_t speed, float distance,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void moveRight(uint8_t speed, float distance,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    void rotateLeft(uint8_t speed, float degree,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);
    void rotateRight(uint8_t speed, float degree,DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    void stop(DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE);

    bool isIdle();

   private:
    float robotRadius, wheelPerimeter;
    
    DCMotor* motorFront;
    DCMotor* motorLeft;
    DCMotor* motorRight;
    DCMotor* motorBack;

    void initInfo(float robotRadius, float wheelPerimeter);
};

#endif
