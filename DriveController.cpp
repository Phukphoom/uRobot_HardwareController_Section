#include "DriveController.h"

// Private
/* Initialization */
void DriveController::initInfo(float robotRadius, float wheelPerimeter, uint8_t motorBootPower) {
  this->robotRadius = robotRadius;
  this->wheelPerimeter = wheelPerimeter;
  this->motorBootPower = motorBootPower;
}

// Public
/* Constructor */
DriveController::DriveController(float robotRadius, float wheelPerimeter, uint8_t motorBootPower, DCMotor *motorFront, DCMotor *motorLeft, DCMotor *motorRight, DCMotor *motorBack)
  : motorFront(motorFront), motorLeft(motorLeft), motorRight(motorRight), motorBack(motorBack) {
  this->initInfo(robotRadius, wheelPerimeter, motorBootPower);
}

/* Destructor */
DriveController::~DriveController() {
}

/* Method */
void DriveController::moveForward(unsigned int speed) {
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

  this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
  this->motorRight->setDirection(DCMotor::Direction::REVERSE);

  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}
void DriveController::moveBackward(unsigned int speed) {
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

  this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
  this->motorRight->setDirection(DCMotor::Direction::FORWARD);

  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}
void DriveController::moveLeft(unsigned int speed) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

  this->motorFront->setDirection(DCMotor::Direction::REVERSE);
  this->motorBack->setDirection(DCMotor::Direction::FORWARD);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
}
void DriveController::moveRight(unsigned int speed) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

  this->motorFront->setDirection(DCMotor::Direction::FORWARD);
  this->motorBack->setDirection(DCMotor::Direction::REVERSE);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
}

void DriveController::moveForward(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorLeft->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));
  this->motorRight->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));

  this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
  this->motorRight->setDirection(DCMotor::Direction::REVERSE);

  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}
void DriveController::moveBackward(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorLeft->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));
  this->motorRight->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));

  this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
  this->motorRight->setDirection(DCMotor::Direction::FORWARD);

  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}
void DriveController::moveLeft(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorFront->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));
  this->motorBack->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));

  this->motorFront->setDirection(DCMotor::Direction::REVERSE);
  this->motorBack->setDirection(DCMotor::Direction::FORWARD);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
}
void DriveController::moveRight(unsigned int speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorFront->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));
  this->motorBack->setTargetPosition((unsigned long)((360 / this->wheelPerimeter) * distance));

  this->motorFront->setDirection(DCMotor::Direction::FORWARD);
  this->motorBack->setDirection(DCMotor::Direction::REVERSE);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
}

void DriveController::rotateLeft(unsigned int speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorFront->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorBack->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorLeft->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorRight->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));

  this->motorFront->setDirection(DCMotor::Direction::REVERSE);
  this->motorBack->setDirection(DCMotor::Direction::REVERSE);
  this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
  this->motorRight->setDirection(DCMotor::Direction::REVERSE);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);
  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}
void DriveController::rotateRight(unsigned int speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
  this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

  this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorFront->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorBack->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorLeft->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
  this->motorRight->setTargetPosition((unsigned long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));

  this->motorFront->setDirection(DCMotor::Direction::FORWARD);
  this->motorBack->setDirection(DCMotor::Direction::FORWARD);
  this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
  this->motorRight->setDirection(DCMotor::Direction::FORWARD);

  this->motorFront->setSpeed(speed);
  this->motorBack->setSpeed(speed);
  this->motorLeft->setSpeed(speed);
  this->motorRight->setSpeed(speed);

  this->motorFront->start(this->motorBootPower);
  this->motorBack->start(this->motorBootPower);
  this->motorLeft->start(this->motorBootPower);
  this->motorRight->start(this->motorBootPower);
}

void DriveController::stop(DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
  this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);
  this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

  this->motorFront->stop();
  this->motorLeft->stop();
  this->motorRight->stop();
  this->motorBack->stop();
}
bool DriveController::isIdle() {
  return !this->motorFront->isRunning() && !this->motorLeft->isRunning() && !this->motorRight->isRunning() && !this->motorBack->isRunning();
}
