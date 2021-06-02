#include "DCMotor.h"

// Private
/* Initialization */
void DCMotor::initPin(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinEncoder) {
    this->pinINA = pinINA;
    this->pinINB = pinINB;
    this->pinPWM = pinPWM;
    this->pinEncoder = pinEncoder;

    pinMode(this->pinINA, OUTPUT);
    pinMode(this->pinINB, OUTPUT);
    pinMode(this->pinINB, OUTPUT);
    pinMode(this->pinEncoder, INPUT_PULLUP);
}

// Public
/* Constructor */
DCMotor::DCMotor(uint8_t pinINA, uint8_t pinINB, uint8_t pinPWM, uint8_t pinEncoder) {
    this->initPin(pinINA, pinINB, pinPWM, pinEncoder);
}

/* Destructor */
DCMotor::~DCMotor() {
}

/* Method */
bool DCMotor::setRunMode(RunMode runMode) {
    if (!this->isRunning()) {
        this->runMode = runMode;
        return true;
    }
    return false;
}
DCMotor::RunMode DCMotor::getRunMode() {
    return this->runMode;
}
bool DCMotor::setDirection(Direction direction) {
    if (!this->isRunning()) {
        this->direction = direction;
        return true;
    }
    return false;
}
DCMotor::Direction DCMotor::getDirection() {
    return this->direction;
}
void DCMotor::setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
    this->zeroPowerBehavior = zeroPowerBehavior;
}
DCMotor::ZeroPowerBehavior DCMotor::getZeroPowerBehavior() {
    return this->zeroPowerBehavior;
}

void DCMotor::setPower(uint8_t power) {
    this->power = power;
    this->setActuallyPower(this->power);
}
uint8_t DCMotor::getPower() {
    return this->power;
}
void DCMotor::setActuallyPower(uint8_t power) {
    this->actuallyPower = power;
    
    /* Generate new PWM pulse */
    if (this->isRunning()) {
        analogWrite(this->pinPWM, this->actuallyPower);
    }
}
uint8_t DCMotor::getActuallyPower() {
    return this->actuallyPower;
}

bool DCMotor::setTargetPosition(long targetPosition) {
    if (this->runMode == DCMotor::RunMode::RUN_TO_POSITION) {
        this->targetPosition = targetPosition;
        this->resetEncoderPosition();
        return true;
    }
    return false;
}
void DCMotor::resetTargetPosition() {
    this->targetPosition = 0;
}
long DCMotor::getTargetPosition() {
    return this->targetPosition;
}

void DCMotor::incEncoderPosition() {
    this->encoderPosition++;
    if (this->runMode == DCMotor::RunMode::RUN_TO_POSITION) {
        if (this->encoderPosition >= this->targetPosition) {
            this->stop();
        }
    }
}
void DCMotor::resetEncoderPosition() {
    this->encoderPosition = 0;
}
long DCMotor::getEncoderPosition() {
    return this->encoderPosition;
}

void DCMotor::run() {
    if (this->runMode == DCMotor::RunMode::RUN_CONTINUE) {
        if (this->direction == Direction::FORWARD) {
            digitalWrite(this->pinINA, HIGH);
            digitalWrite(this->pinINB, LOW);
        } else if (this->direction == Direction::REVERSE) {
            digitalWrite(this->pinINA, LOW);
            digitalWrite(this->pinINB, HIGH);
        }
        analogWrite(this->pinPWM, this->actuallyPower);
        this->onRunning = true;
    } else if (this->runMode == DCMotor::RunMode::RUN_TO_POSITION) {
        if (this->encoderPosition < this->targetPosition) {
            if (this->direction == Direction::FORWARD) {
                digitalWrite(this->pinINA, HIGH);
                digitalWrite(this->pinINB, LOW);
            } else if (this->direction == Direction::REVERSE) {
                digitalWrite(this->pinINA, LOW);
                digitalWrite(this->pinINB, HIGH);
            }
            analogWrite(this->pinPWM, this->actuallyPower);
            this->onRunning = true;
        }
    }
}
void DCMotor::stop() {
    digitalWrite(this->pinINA, LOW);
    digitalWrite(this->pinINB, LOW);

    if (this->zeroPowerBehavior == ZeroPowerBehavior::BRAKE) {
        analogWrite(this->pinPWM, 255);
    } else if (this->zeroPowerBehavior == ZeroPowerBehavior::RELEASE) {
        analogWrite(this->pinPWM, 0);
    }
    this->onRunning = false;

    this->setPower(0);
    this->resetEncoderPosition();
    this->resetTargetPosition();
}

bool DCMotor::isRunning() {
    return this->onRunning;
}
