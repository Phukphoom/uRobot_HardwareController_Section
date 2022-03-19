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

void DCMotor::setSpeed(unsigned int speed) {
    this->speed = speed;
}
unsigned int DCMotor::getSpeed() {
    return this->speed;
}

bool DCMotor::setTargetPosition(unsigned long targetPosition) {
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
            return;
        }
    }
}
void DCMotor::resetEncoderPosition() {
    this->encoderPosition = 0;
}
unsigned long DCMotor::getEncoderPosition() {
    return this->encoderPosition;
}

void DCMotor::start(uint8_t bootPower) {
    if (this->runMode == DCMotor::RunMode::RUN_CONTINUE) {
        if (this->direction == Direction::FORWARD) {
            digitalWrite(this->pinINA, HIGH);
            digitalWrite(this->pinINB, LOW);
        } else if (this->direction == Direction::REVERSE) {
            digitalWrite(this->pinINA, LOW);
            digitalWrite(this->pinINB, HIGH);
        }

        this->power = bootPower;
        analogWrite(this->pinPWM, this->power);
        this->onRunning = true;
        this->previousAdjustPowerTime_AutoAdjust = millis();
        this->previousEncoderPosition_AutoAdjust = this->encoderPosition;
    } else if (this->runMode == DCMotor::RunMode::RUN_TO_POSITION) {
        if (this->encoderPosition < this->targetPosition) {
            if (this->direction == Direction::FORWARD) {
                digitalWrite(this->pinINA, HIGH);
                digitalWrite(this->pinINB, LOW);
            } else if (this->direction == Direction::REVERSE) {
                digitalWrite(this->pinINA, LOW);
                digitalWrite(this->pinINB, HIGH);
            }

            this->power = bootPower;
            analogWrite(this->pinPWM, this->power);
            this->onRunning = true;
            this->previousAdjustPowerTime_AutoAdjust = millis();
            this->previousEncoderPosition_AutoAdjust = this->encoderPosition;
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

    this->speed = 0;
    this->power = 0;

    this->onRunning = false;

    this->resetEncoderPosition();
    this->resetTargetPosition();

    this->previousAdjustPowerTime_AutoAdjust = 0;
    this->previousEncoderPosition_AutoAdjust = 0;
}
void DCMotor::autoAdjustPower() {
    if (this->isRunning()) {
        unsigned long actuallySpeed = this->getActuallySpeed();
        /* dont adjust if actuallySpeed is overflow or eq 0 */
        if (actuallySpeed > 0 && actuallySpeed < 10000) {
            //      this->debuggingValue = actuallySpeed;
            if (abs(actuallySpeed - this->speed) >= 5) {
                if (actuallySpeed < this->speed) {
                    this->power = min(255, this->power + 1);
                } else if (actuallySpeed > this->speed) {
                    this->power = max(0, this->power - 1);
                }
                analogWrite(this->pinPWM, this->power);
            }
            this->previousAdjustPowerTime_AutoAdjust = millis();
            this->previousEncoderPosition_AutoAdjust = this->encoderPosition;
        }
    }
}

bool DCMotor::isRunning() {
    return this->onRunning;
}

unsigned long DCMotor::getActuallySpeed() {
    return ((this->encoderPosition - this->previousEncoderPosition_AutoAdjust) * 1000) / (millis() - this->previousAdjustPowerTime_AutoAdjust);
}

uint8_t DCMotor::getPower() {
    return this->power;
}
