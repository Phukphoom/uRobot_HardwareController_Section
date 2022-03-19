#include "Arduino.h"
#include "Servo.h"

#ifndef ServoNMController_h
#define ServoNMController_h

class ServoNMController {
  public:
    ServoNMController(Servo *servo);
    ~ServoNMController();

    void goToDeg(float degree, uint16_t changingDelay = 10);

  private:
    Servo *servo;

    uint8_t lastDeg = 0;
};

#endif
