#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(4);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(3);

void RightMotorSpeed(int new_speed)
{
    // A function that sets the speed of the right motor, only when the desired speed changes
    static int old_speed = 0;

    if (old_speed == new_speed)
    {
        return;
    }

    else
    {
        rightMotor->setSpeed(new_speed);
    }

}