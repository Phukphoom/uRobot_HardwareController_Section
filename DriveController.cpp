#include "DriveController.h"

// Private
/* Initialization */
void DriveController::initInfo(float robotRadius, float wheelPerimeter) {
    this->robotRadius = robotRadius;
    this->wheelPerimeter = wheelPerimeter;
}

// Public
/* Constructor */
DriveController::DriveController(float robotRadius, float wheelPerimeter, DCMotor *motorFront, DCMotor *motorLeft, DCMotor *motorRight, DCMotor *motorBack)
    : motorFront(motorFront), motorLeft(motorLeft), motorRight(motorRight), motorBack(motorBack) {
    this->initInfo(robotRadius, wheelPerimeter);
}

/* Destructor */
DriveController::~DriveController() {
}

/* Method */
void DriveController::moveForward(uint8_t speed) {
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

    this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
    this->motorRight->setDirection(DCMotor::Direction::REVERSE);

    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorLeft->run();
    this->motorRight->run();
}
void DriveController::moveBackward(uint8_t speed) {
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

    this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
    this->motorRight->setDirection(DCMotor::Direction::FORWARD);

    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorLeft->run();
    this->motorRight->run();
}
void DriveController::moveLeft(uint8_t speed) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

    this->motorFront->setDirection(DCMotor::Direction::REVERSE);
    this->motorBack->setDirection(DCMotor::Direction::FORWARD);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
}
void DriveController::moveRight(uint8_t speed) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_CONTINUE);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_CONTINUE);

    this->motorFront->setDirection(DCMotor::Direction::FORWARD);
    this->motorBack->setDirection(DCMotor::Direction::REVERSE);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
}

void DriveController::moveForward(uint8_t speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorLeft->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));
    this->motorRight->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));

    this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
    this->motorRight->setDirection(DCMotor::Direction::REVERSE);

    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorLeft->run();
    this->motorRight->run();
}
void DriveController::moveBackward(uint8_t speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorLeft->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));
    this->motorRight->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));

    this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
    this->motorRight->setDirection(DCMotor::Direction::FORWARD);

    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorLeft->run();
    this->motorRight->run();
}
void DriveController::moveLeft(uint8_t speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorFront->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));
    this->motorBack->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));

    this->motorFront->setDirection(DCMotor::Direction::REVERSE);
    this->motorBack->setDirection(DCMotor::Direction::FORWARD);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
}
void DriveController::moveRight(uint8_t speed, float distance, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorFront->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));
    this->motorBack->setTargetPosition((long)((360 / this->wheelPerimeter) * distance));

    this->motorFront->setDirection(DCMotor::Direction::FORWARD);
    this->motorBack->setDirection(DCMotor::Direction::REVERSE);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
}

void DriveController::rotateLeft(uint8_t speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorFront->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorBack->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorLeft->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorRight->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));

    this->motorFront->setDirection(DCMotor::Direction::REVERSE);
    this->motorBack->setDirection(DCMotor::Direction::REVERSE);
    this->motorLeft->setDirection(DCMotor::Direction::REVERSE);
    this->motorRight->setDirection(DCMotor::Direction::REVERSE);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));
    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
    this->motorLeft->run();
    this->motorRight->run();
}
void DriveController::rotateRight(uint8_t speed, float degree, DCMotor::ZeroPowerBehavior zeroPowerBehavior = DCMotor::ZeroPowerBehavior::BRAKE) {
    this->motorFront->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorBack->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorLeft->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);
    this->motorRight->setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    this->motorFront->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorLeft->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorRight->setZeroPowerBehavior(zeroPowerBehavior);
    this->motorBack->setZeroPowerBehavior(zeroPowerBehavior);

    this->motorFront->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorBack->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorLeft->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));
    this->motorRight->setTargetPosition((long)((((2 * PI * this->robotRadius) / 360) * degree) * (360 / this->wheelPerimeter)));

    this->motorFront->setDirection(DCMotor::Direction::FORWARD);
    this->motorBack->setDirection(DCMotor::Direction::FORWARD);
    this->motorLeft->setDirection(DCMotor::Direction::FORWARD);
    this->motorRight->setDirection(DCMotor::Direction::FORWARD);

    this->motorFront->setPower(min(speed, 255));
    this->motorBack->setPower(min(speed, 255));
    this->motorLeft->setPower(min(speed, 255));
    this->motorRight->setPower(min(speed, 255));

    this->motorFront->run();
    this->motorBack->run();
    this->motorLeft->run();
    this->motorRight->run();
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