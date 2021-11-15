#include "project_defines.h"

#ifdef FULLSPEEDAHEAD


#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(4);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(3);

void setup()
{
    AFSM.begin();
    leftMotor->setSpeed(255);
    rightMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
}

void loop() { }

#endif