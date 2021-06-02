#include "DCMotor.h"
#include "DriveController.h"
#include "Servo.h"
#include "ServoCNController.h"
#include "ServoNMController.h"

/* ---------------------------- INFO DEFINE -------------------------------------------------------- */
#define ROBOT_RADIUS 14.5
#define WHEEL_PERIMETER 31.88

/* ---------------------------- PIN DEFINE --------------------------------------------------------- */
#define IR_LEFT_PIN 34
#define IR_RIGHT_PIN 35

#define SERVO_GATE_PIN_PWM 8

#define SERVO_CRANE_PIN_PWM 44

#define SERVO_HLIFT_PIN_PWM 45
#define SERVO_HLIFT_PIN_FORWARD_STOP 16
#define SERVO_HLIFT_PIN_REVERSE_STOP 17

#define MOTOR_KEEPER_PIN_INA 32
#define MOTOR_KEEPER_PIN_INB 33
#define MOTOR_KEEPER_PIN_PWM 5
#define MOTOR_KEEPER_PIN_ENCODER 21

#define MOTOR_FRONT_PIN_INA 22
#define MOTOR_FRONT_PIN_INB 23
#define MOTOR_FRONT_PIN_PWM 9
#define MOTOR_FRONT_PIN_ENCODER 2

#define MOTOR_BACK_PIN_INA 24
#define MOTOR_BACK_PIN_INB 25
#define MOTOR_BACK_PIN_PWM 10
#define MOTOR_BACK_PIN_ENCODER 3

#define MOTOR_LEFT_PIN_INA 26
#define MOTOR_LEFT_PIN_INB 27
#define MOTOR_LEFT_PIN_PWM 6
#define MOTOR_LEFT_PIN_ENCODER 18

#define MOTOR_RIGHT_PIN_INA 28
#define MOTOR_RIGHT_PIN_INB 29
#define MOTOR_RIGHT_PIN_PWM 7
#define MOTOR_RIGHT_PIN_ENCODER 19

/* ---------------------------- Hardware Initial --------------------------------------------------- */
Servo servoGate;
ServoNMController servoGateController(&servoGate);
Servo servoCrane;
ServoNMController servoCraneController(&servoCrane);
Servo servoHlift;
ServoCNController servoHliftController(&servoHlift, SERVO_HLIFT_PIN_FORWARD_STOP, SERVO_HLIFT_PIN_REVERSE_STOP);

DCMotor motorKeeper(MOTOR_KEEPER_PIN_INA, MOTOR_KEEPER_PIN_INB, MOTOR_KEEPER_PIN_PWM, MOTOR_KEEPER_PIN_ENCODER);

DCMotor motorFront(MOTOR_FRONT_PIN_INA, MOTOR_FRONT_PIN_INB, MOTOR_FRONT_PIN_PWM, MOTOR_FRONT_PIN_ENCODER);
DCMotor motorLeft(MOTOR_LEFT_PIN_INA, MOTOR_LEFT_PIN_INB, MOTOR_LEFT_PIN_PWM, MOTOR_LEFT_PIN_ENCODER);
DCMotor motorRight(MOTOR_RIGHT_PIN_INA, MOTOR_RIGHT_PIN_INB, MOTOR_RIGHT_PIN_PWM, MOTOR_RIGHT_PIN_ENCODER);
DCMotor motorBack(MOTOR_BACK_PIN_INA, MOTOR_BACK_PIN_INB, MOTOR_BACK_PIN_PWM, MOTOR_BACK_PIN_ENCODER);
DriveController driveController(ROBOT_RADIUS, WHEEL_PERIMETER, &motorFront, &motorLeft, &motorRight, &motorBack);

/* ---------------------------- Main Setup --------------------------------------------------------- */
void setup() {
    Serial.begin(115200);

    pinMode(IR_LEFT_PIN, INPUT);
    pinMode(IR_RIGHT_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(MOTOR_KEEPER_PIN_ENCODER), updateKeeperEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_FRONT_PIN_ENCODER), updateFrontEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_LEFT_PIN_ENCODER), updateLeftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_RIGHT_PIN_ENCODER), updateRightEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(MOTOR_BACK_PIN_ENCODER), updateBackEncoder, RISING);

    servoGate.attach(SERVO_GATE_PIN_PWM);
    servoGateController.goToDeg(180);
    servoCrane.attach(SERVO_CRANE_PIN_PWM);
    servoCraneController.goToDeg(120);
    servoHlift.attach(SERVO_HLIFT_PIN_PWM);
    servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 100);

    Serial.println("Setup Completed!");
}

/* ---------------------------- Main Logic --------------------------------------------------------- */
void loop() {
    if (!driveController.isIdle()) {
        balanceMotorSpeed(&motorFront, &motorBack);
        balanceMotorSpeed(&motorLeft, &motorRight);

        // For Debuging
        /*
        Serial.print("dFB: ");
        Serial.println(motorFront.getEncoderPosition() - motorBack.getEncoderPosition());
        Serial.print("F-power: ");
        Serial.print(motorFront.getActuallyPower());
        Serial.print(" B-power: ");
        Serial.println(motorBack.getActuallyPower());
        Serial.println();

        Serial.print("dLR: ");
        Serial.println(motorLeft.getEncoderPosition() - motorRight.getEncoderPosition());
        Serial.print("L-power: ");
        Serial.print(motorLeft.getActuallyPower());
        Serial.print(" R-power: ");
        Serial.println(motorRight.getActuallyPower());
        Serial.println();
        */   
    }

    if (Serial.available()) {
        String receiveMsg = Serial.readStringUntil('\n');

        String type = getValue(receiveMsg, ':', 0);
        String command = getValue(receiveMsg, ':', 1);
        String param_1 = getValue(receiveMsg, ':', 2);
        String param_2 = getValue(receiveMsg, ':', 3);
        String param_3 = getValue(receiveMsg, ':', 4);

        /* TYPE : Command */
        if (type == "CMD") {
            /* DRIVE [Continue] - Command Control */
            if (command == "STP") {
                if (param_1 == "RLS") {
                    driveController.stop(DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "MFC") {
                driveController.moveForward(param_1.toInt());
                Serial.println("DONE");
            } else if (command == "MBC") {
                driveController.moveBackward(param_1.toInt());
                Serial.println("DONE");
            } else if (command == "MLC") {
                driveController.moveLeft(param_1.toInt());
                Serial.println("DONE");
            } else if (command == "MRC") {
                driveController.moveRight(param_1.toInt());
                Serial.println("DONE");
            }

            /* DRIVE [To Target] - Command Control */
            else if (command == "MFT") {
                if (param_3 == "RLS") {
                    driveController.moveForward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveForward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "MBT") {
                if (param_3 == "RLS") {
                    driveController.moveBackward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveBackward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "MLT") {
                if (param_3 == "RLS") {
                    driveController.moveLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "MRT") {
                if (param_3 == "RLS") {
                    driveController.moveRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "RLT") {
                if (param_3 == "RLS") {
                    driveController.rotateLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.rotateLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            } else if (command == "RRT") {
                if (param_3 == "RLS") {
                    driveController.rotateRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.rotateRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
                Serial.println("DONE");
            }

            /* GATE - Command Control */
            else if (command == "GTE") {
                if (param_1 == "OPN") {
                    servoGateController.goToDeg(120);

                    Serial.println("DONE");
                } else if (param_1 == "CLS") {
                    servoGateController.goToDeg(180);

                    Serial.println("DONE");
                }
            }

            /* KEEPER - Command Control */
            else if (command == "KPB") {
                /* Keep Ball */
                servoCraneController.goToDeg(120);
                servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                keeperUncatch();
                servoCraneController.goToDeg(175, 50);
                keeperCatch();

                servoCraneController.goToDeg(120);
                servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);

                Serial.println("DONE");
            } else if (command == "SRS") {
                /* Same Base Release */
                servoCraneController.goToDeg(120);
                servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                keeperUncatch();
                delay(500);
                keeperCatch();

                servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);

                Serial.println("DONE");
            } else if (command == "YRS") {
                /* Yellow Release */
                servoCraneController.goToDeg(70);
                servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                keeperUncatch();
                delay(500);
                keeperCatch();

                servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);
                servoCraneController.goToDeg(120);

                Serial.println("DONE");
            } else if (command == "BRS") {
                /* Basket Release */
                keeperUncatch();
                delay(500);
                keeperCatch();

                Serial.println("DONE");
            }

            /* SPACIAL_OPTION - Command Control */
            else if (command == "OPT") {
                /* Move Front to Brake Line */
                if (param_1 == "MFBL") {
                    driveController.moveForward(param_2.toInt());

                    bool irLeft = digitalRead(IR_LEFT_PIN);
                    bool irRight = digitalRead(IR_RIGHT_PIN);

                    while (!irLeft || !irRight) {
                        if (irLeft == HIGH && irRight == LOW) {
                            driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);
                            driveController.rotateLeft(param_2.toInt(), 1, DCMotor::ZeroPowerBehavior::RELEASE);
                            while (!driveController.isIdle()) {
                                delay(1);
                            }
                        } else if (irLeft == LOW && irRight == HIGH) {
                            driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);
                            driveController.rotateRight(param_2.toInt(), 1, DCMotor::ZeroPowerBehavior::RELEASE);
                            while (!driveController.isIdle()) {
                                delay(1);
                            }
                        } else {
                            driveController.moveForward(param_2.toInt());
                        }

                        irLeft = digitalRead(IR_LEFT_PIN);
                        irRight = digitalRead(IR_RIGHT_PIN);
                    }
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);

                    Serial.println("DONE");
                }
                /* Move Back to Brake Line */
                else if (param_1 == "MBBL") {
                    driveController.moveBackward(param_2.toInt());

                    bool irLeft = digitalRead(IR_LEFT_PIN);
                    bool irRight = digitalRead(IR_RIGHT_PIN);

                    while (!irLeft || !irRight) {
                        if (irLeft == HIGH && irRight == LOW) {
                            driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);
                            driveController.rotateRight(param_2.toInt(), 1, DCMotor::ZeroPowerBehavior::RELEASE);
                            while (!driveController.isIdle()) {
                                delay(1);
                            }
                        } else if (irLeft == LOW && irRight == HIGH) {
                            driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);
                            driveController.rotateLeft(param_2.toInt(), 1, DCMotor::ZeroPowerBehavior::RELEASE);
                            while (!driveController.isIdle()) {
                                delay(1);
                            }
                        } else {
                            driveController.moveBackward(param_2.toInt());
                        }

                        irLeft = digitalRead(IR_LEFT_PIN);
                        irRight = digitalRead(IR_RIGHT_PIN);
                    }
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);

                    Serial.println("DONE");
                }
                /* Move Left to Brake Line */
                else if (param_1 == "MLBL") {
                    driveController.moveLeft(param_2.toInt());

                    bool irLeft = digitalRead(IR_LEFT_PIN);

                    while (!irLeft) {
                        irLeft = digitalRead(IR_LEFT_PIN);
                    }
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);

                    Serial.println("DONE");
                }
                /* Move Right to Brake Line */
                else if (param_1 == "MRBL") {
                    driveController.moveRight(param_2.toInt());

                    bool irRight = digitalRead(IR_RIGHT_PIN);

                    while (!irRight) {
                        irRight = digitalRead(IR_RIGHT_PIN);
                    }
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);

                    Serial.println("DONE");
                }
            }
        }

        /* TYPE : Get */
        else if (type == "GET") {
            if (command == "DC_ISIDLE") {
                Serial.println(driveController.isIdle());
            }
        }
    }
}
/* ---------------------------- Robot Function ----------------------------------------------------- */
void keeperCatch() {
    motorKeeper.setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    motorKeeper.setTargetPosition(420);

    motorKeeper.setDirection(DCMotor::Direction::FORWARD);

    motorKeeper.setPower(255);

    motorKeeper.run();

    /* Block Arduino process! */
    while (motorKeeper.isRunning()) {
        delay(1);
    }
}
void keeperUncatch() {
    motorKeeper.setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    motorKeeper.setTargetPosition(420);

    motorKeeper.setDirection(DCMotor::Direction::REVERSE);

    motorKeeper.setPower(255);

    motorKeeper.run();

    /* Block Arduino process! */
    while (motorKeeper.isRunning()) {
        delay(1);
    }
}

/* ---------------------------- Adjust Motor Speed V.4 Function ------------------------------------ */
void balanceMotorSpeed(DCMotor *motor_1, DCMotor *motor_2) {
    long diff = motor_1->getEncoderPosition() - motor_2->getEncoderPosition();

    const float wantedPower = (motor_1->getPower() + motor_2->getPower())/2;

    /* Kp from Linear regresion (experiment)*/
    /*
      [speed]   ->      [Kp]
      30        ->      0.2
      50        ->      0.4
      80        ->      0.7
      110       ->      1.1
      130       ->      1.4
      150       ->      1.8
    */
    const float Kp = (0.01302 * wantedPower) - 0.2599;
    
    if (motor_1->isRunning() || motor_2->isRunning()) {
        motor_1->setActuallyPower(uint8_t(max(min(motor_1->getPower() - (Kp * diff), 255), 0)));
        motor_2->setActuallyPower(uint8_t(max(min(motor_2->getPower() + (Kp * diff), 255), 0)));
    }
}

/* ---------------------------- Utility Function --------------------------------------------------- */
String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i + 1 : i;
        }
    }

    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

/* ---------------------------- Interrupt-Callback Function ---------------------------------------- */
void updateKeeperEncoder() {
    motorKeeper.incEncoderPosition();
}
void updateFrontEncoder() {
    motorFront.incEncoderPosition();
}
void updateLeftEncoder() {
    motorLeft.incEncoderPosition();
}
void updateRightEncoder() {
    motorRight.incEncoderPosition();
}
void updateBackEncoder() {
    motorBack.incEncoderPosition();
}
