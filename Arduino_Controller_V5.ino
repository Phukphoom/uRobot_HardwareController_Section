#include "DCMotor.h"
#include "DriveController.h"
#include "Servo.h"
#include "ServoCNController.h"
#include "ServoNMController.h"

/* ---------------------------- INFO DEFINE -------------------------------------------------------- */
#define ROBOT_RADIUS 14.5
#define WHEEL_PERIMETER 31.88
#define MOTOR_BOOT_POWER 50

/* ---------------------------- PIN DEFINE --------------------------------------------------------- */
#define IR_LEFT_PIN 35
#define IR_RIGHT_PIN 34

#define SERVO_GATE_PIN_PWM 13

#define SERVO_CRANE_PIN_PWM 8

#define SERVO_HLIFT_PIN_PWM 4
#define SERVO_HLIFT_PIN_FORWARD_STOP 17
#define SERVO_HLIFT_PIN_REVERSE_STOP 16

#define MOTOR_KEEPER_PIN_INA 33
#define MOTOR_KEEPER_PIN_INB 32
#define MOTOR_KEEPER_PIN_PWM 5
#define MOTOR_KEEPER_PIN_ENCODER 20

#define MOTOR_FRONT_PIN_INA 22
#define MOTOR_FRONT_PIN_INB 23
#define MOTOR_FRONT_PIN_PWM 6
#define MOTOR_FRONT_PIN_ENCODER 18

#define MOTOR_BACK_PIN_INA 24
#define MOTOR_BACK_PIN_INB 25
#define MOTOR_BACK_PIN_PWM 7
#define MOTOR_BACK_PIN_ENCODER 3

#define MOTOR_LEFT_PIN_INA 26
#define MOTOR_LEFT_PIN_INB 27
#define MOTOR_LEFT_PIN_PWM 10
#define MOTOR_LEFT_PIN_ENCODER 2

#define MOTOR_RIGHT_PIN_INA 29
#define MOTOR_RIGHT_PIN_INB 28
#define MOTOR_RIGHT_PIN_PWM 9
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
DriveController driveController(ROBOT_RADIUS, WHEEL_PERIMETER, MOTOR_BOOT_POWER, &motorFront, &motorLeft, &motorRight, &motorBack);

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

    driveController.stop(DCMotor::ZeroPowerBehavior::RELEASE);

    Serial.println("Setup Completed!");
}

/* ---------------------------- Main Logic --------------------------------------------------------- */
void loop() {
    //  unsigned long prevTime = motorRight.previousAdjustPowerTime_AutoAdjust;

    /* Auto adjust motor speed - (Make motor speed stable) */
    motorFront.autoAdjustPower();
    motorLeft.autoAdjustPower();
    motorRight.autoAdjustPower();
    motorBack.autoAdjustPower();

    //  Serial.print("Speed:");
    //  Serial.print(motorRight.getSpeed());
    //  Serial.print("\tAcSpeed:");
    //  Serial.println(motorRight.debuggingValue);
    //  Serial.print("\tprevTime:");
    //  Serial.print(prevTime);
    //  Serial.print("\tTime:");
    //  Serial.print(motorRight.previousAdjustPowerTime_AutoAdjust);
    //  Serial.print("\tdT:");
    //  Serial.print(int(prevTime - motorRight.previousAdjustPowerTime_AutoAdjust));
    //  Serial.print("\tprevENC:");
    //  Serial.print(motorRight.previousEncoderPosition_AutoAdjust);
    //  Serial.print("\tENC:");
    //  Serial.print(motorRight.encoderPosition);
    //  Serial.print("\t | Power:");
    //  Serial.println(motorRight.getPower());

    /* Serial Communication with Ras-PI */
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
            } else if (command == "MFC") {
                driveController.moveForward(param_1.toInt());
            } else if (command == "MBC") {
                driveController.moveBackward(param_1.toInt());
            } else if (command == "MLC") {
                driveController.moveLeft(param_1.toInt());
            } else if (command == "MRC") {
                driveController.moveRight(param_1.toInt());
            }

            /* DRIVE [To Target] - Command Control */
            else if (command == "MFT") {
                if (param_3 == "RLS") {
                    driveController.moveForward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveForward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
            } else if (command == "MBT") {
                if (param_3 == "RLS") {
                    driveController.moveBackward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveBackward(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
            } else if (command == "MLT") {
                if (param_3 == "RLS") {
                    driveController.moveLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
            } else if (command == "MRT") {
                if (param_3 == "RLS") {
                    driveController.moveRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.moveRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
            } else if (command == "RLT") {
                if (param_3 == "RLS") {
                    driveController.rotateLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.rotateLeft(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
            } else if (command == "RRT") {
                if (param_3 == "RLS") {
                    driveController.rotateRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::RELEASE);
                } else {
                    driveController.rotateRight(param_1.toInt(), param_2.toFloat(), DCMotor::ZeroPowerBehavior::BRAKE);
                }
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

            /* CRANE - Command Control */
            else if (command == "CRN") {
                if (param_1 == "HIGH") {
                    servoCraneController.goToDeg(70);
                } else if (param_1 == "DEFAULT") {
                    servoCraneController.goToDeg(120);
                } else if (param_1 == "LOW") {
                    servoCraneController.goToDeg(175);
                }

                Serial.println("DONE");
            }

            /* HLIFT - Command Control */
            else if (command == "HLF") {
                if (param_1 == "OPN") {
                    servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);
                    Serial.println("DONE");
                } else if (param_1 == "CLS") {
                    servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);
                    Serial.println("DONE");
                }
            }

            /* KEEPER - Command Control */
            else if (command == "KPR") {
                if (param_1 == "OPN") {
                    keeperUncatch();
                    Serial.println("DONE");
                } else if (param_1 == "CLS") {
                    keeperCatch();
                    Serial.println("DONE");
                }
            }

            /* SPACIAL_OPTION - Command Control */
            else if (command == "OPT") {
                if (param_1 == "KPB") {
                    /* Keep Ball */
                    servoCraneController.goToDeg(120);
                    servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                    keeperUncatch();
                    servoCraneController.goToDeg(175);
                    keeperCatch();

                    servoCraneController.goToDeg(120);
                    servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);

                    Serial.println("DONE");
                } else if (param_1 == "SRS") {
                    /* Same Base Release */
                    servoCraneController.goToDeg(120);
                    servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                    keeperUncatch();
                    delay(500);
                    keeperCatch();

                    servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);

                    Serial.println("DONE");
                } else if (param_1 == "YRS") {
                    /* Yellow Release */
                    servoCraneController.goToDeg(70);
                    servoHliftController.runToLimit(ServoCNController::Direction::REVERSE, 255);

                    keeperUncatch();
                    delay(500);
                    keeperCatch();

                    servoHliftController.runToLimit(ServoCNController::Direction::FORWARD, 255);
                    servoCraneController.goToDeg(120);

                    Serial.println("DONE");
                } else if (param_1 == "BRS") {
                    /* Basket Release */
                    keeperUncatch();
                    delay(500);
                    keeperCatch();

                    Serial.println("DONE");
                }

                else if (param_1 == "MFBL") {
                    /* Move Front to Brake Line */
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
                } else if (param_1 == "MBBL") {
                    /* Move Back to Brake Line */
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
                } else if (param_1 == "MLBL") {
                    /* Move Left to Brake Line */
                    driveController.moveLeft(param_2.toInt());

                    bool irLeft = digitalRead(IR_LEFT_PIN);

                    while (!irLeft) {
                        irLeft = digitalRead(IR_LEFT_PIN);
                    }
                    driveController.stop(DCMotor::ZeroPowerBehavior::BRAKE);

                    Serial.println("DONE");
                } else if (param_1 == "MRBL") {
                    /* Move Right to Brake Line */
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

    motorKeeper.setSpeed(720);

    motorKeeper.start(255);

    /* Block Arduino process! */
    while (motorKeeper.isRunning()) {
        delay(1);
    }
}
void keeperUncatch() {
    motorKeeper.setRunMode(DCMotor::RunMode::RUN_TO_POSITION);

    motorKeeper.setTargetPosition(420);

    motorKeeper.setDirection(DCMotor::Direction::REVERSE);

    motorKeeper.setSpeed(720);

    motorKeeper.start(255);

    /* Block Arduino process! */
    while (motorKeeper.isRunning()) {
        delay(1);
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
