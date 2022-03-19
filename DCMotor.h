#include "Arduino.h"

#ifndef DCMotor_h
#define DCMotor_h

class DCMotor {
   public:
    // unsigned long debuggingValue = 0;

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

    void setSpeed(unsigned int speed);
    unsigned int getSpeed();

    bool setTargetPosition(unsigned long targetPosition);
    void resetTargetPosition();
    long getTargetPosition();

    void incEncoderPosition();
    void resetEncoderPosition();
    unsigned long getEncoderPosition();

    void start(uint8_t bootPower);
    void stop();
    void autoAdjustPower();

    bool isRunning();
    unsigned long getActuallySpeed();
    uint8_t getPower();

    unsigned long encoderPosition = 0;  // temporary public

    unsigned long previousAdjustPowerTime_AutoAdjust = 0;  // temporary public
    unsigned long previousEncoderPosition_AutoAdjust = 0;  // temporary public

   private:
    uint8_t pinINA, pinINB, pinPWM, pinEncoder;

    RunMode runMode = RunMode::RUN_CONTINUE;
    Direction direction = Direction::FORWARD;
    ZeroPowerBehavior zeroPowerBehavior = ZeroPowerBehavior::BRAKE;
    unsigned int speed = 0;
    unsigned long targetPosition = 0;

    uint8_t power = 0;

    bool onRunning = false;

    void initPin(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinEncoder);
};

#endif
