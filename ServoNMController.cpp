#include "ServoNMController.h"

// Public
/* Constructor */
ServoNMController::ServoNMController(Servo *servo):servo(servo) {
}

/* Destructor */
ServoNMController::~ServoNMController() {
}

/* Method */
void ServoNMController::goToDeg(float degree,uint16_t changingDelay = 10) {
  /* Block Arduino process! */
  while(this->lastDeg < degree){
    this->lastDeg = this->lastDeg+1;
    this->servo->write(this->lastDeg);
    delay(changingDelay);
  }

  /* Block Arduino process! */
  while(this->lastDeg > degree){
    this->lastDeg = this->lastDeg-1;
    this->servo->write(this->lastDeg);
    delay(changingDelay);
  }
}
