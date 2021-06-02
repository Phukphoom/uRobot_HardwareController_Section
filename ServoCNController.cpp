#include "ServoCNController.h"

// Private
/* Initialization */
void ServoCNController::initPin(uint8_t pinForwardStop, uint8_t pinReverseStop) {
    this->pinForwardStop = pinForwardStop;
    this->pinReverseStop = pinReverseStop;

    pinMode(pinForwardStop, INPUT);
    pinMode(pinReverseStop, INPUT);
}

// Public
/* Constructor */
ServoCNController::ServoCNController(Servo *servo, uint8_t pinForwardStop, uint8_t pinReverseStop) : servo(servo) {
    this->initPin(pinForwardStop, pinReverseStop);
}

/* Destructor */
ServoCNController::~ServoCNController() {
}

/* Method */
void ServoCNController::runToLimit(Direction direction, uint8_t speed) {
    if (direction == ServoCNController::Direction::FORWARD) {
        uint8_t angle_signal = map(speed, 0, 255, 95, 180);

        /* Block Arduino process! */
        while (digitalRead(this->pinForwardStop) != HIGH) {
            this->servo->write(angle_signal);
        }
    } else if (direction == ServoCNController::Direction::REVERSE) {
        uint8_t angle_signal = map(speed, 0, 255, 95, 0);
        
        /* Block Arduino process! */
        while (digitalRead(this->pinReverseStop) != HIGH) {
            this->servo->write(angle_signal);
        }
    }

    /* Stop Servo */
    this->servo->write(95);
}