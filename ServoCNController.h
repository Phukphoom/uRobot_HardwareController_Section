#include "Arduino.h"
#include "Servo.h"

#ifndef ServoCNController_h
#define ServoCNController_h

class ServoCNController {
   public:
    enum class Direction {
        FORWARD = 0,
        REVERSE = 1
    };

    ServoCNController(Servo* servo, uint8_t pinForwardStop, uint8_t pinReverseStop);
    ~ServoCNController();

    void runToLimit(Direction direction, uint8_t speed);

   private:
    uint8_t pinForwardStop, pinReverseStop;

    Servo* servo;

    void initPin(uint8_t pinForwardStop, uint8_t pinReverseStop);
};

#endif
