#include "Arduino.h"

#ifndef DCMotor_h
#define DCMotor_h

class DCMotor {
   public:
    enum class RunMode {
        RUN_CONTINUE = 0,
        RUN_TO_POSITION = 1,
    };
    enum class Direction {
        FORWARD = 0,
        REVERSE = 1
    };
    enum class ZeroPowerBehavior {
        BRAKE = 0,
        RELEASE = 1 
        
    };

    DCMotor(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinEncoder);
    ~DCMotor();

    bool setRunMode(RunMode runMode);
    RunMode getRunMode();
    bool setDirection(Direction direction);
    Direction getDirection();
    void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior);
    ZeroPowerBehavior getZeroPowerBehavior();

    void setPower(uint8_t power);
    uint8_t getPower();
    void setActuallyPower(uint8_t power);
    uint8_t getActuallyPower();


    bool setTargetPosition(long targetPosition);
    void resetTargetPosition();
    long getTargetPosition();

    void incEncoderPosition();
    void resetEncoderPosition();
    long getEncoderPosition();

    void run();
    void stop();

    bool isRunning();

   private:
    uint8_t pinINA, pinINB, pinPWM, pinEncoder;
    uint8_t power = 0;
    uint8_t actuallyPower = 0;

    long targetPosition = 0;
    long encoderPosition = 0;
    long remEncoderPosition = 0;

    bool onRunning = false;

    RunMode runMode = RunMode::RUN_CONTINUE;
    Direction direction = Direction::FORWARD;
    ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior::BRAKE;

    void initPin(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinEncoder);
};

#endif
