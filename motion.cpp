#include "motion.h"
#include "robot.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Motion::Motion(Adafruit_MotorShield *_AFSM, uint8_t leftMotorPort, uint8_t rightMotorPort)
{
    AFSM = _AFSM;
    leftMotor = AFSM->getMotor(leftMotorPort);
    rightMotor = AFSM->getMotor(rightMotorPort);
    // WTFfffffffffffF WTAFwtwfwtwftwtf
    //Serial.println("there's no way right");

    t = millis();
}

void Motion::Begin()
{
    AFSM->begin();
    
    speed = 0;
    distance = 0;
    lastDistance = 0;
    turnRadius = 0;
    bearing = 0;

    updated = false;
}

Motion::Motion(uint8_t leftMotorPort, uint8_t rightMotorPort) : Motion(new Adafruit_MotorShield(), leftMotorPort, rightMotorPort) { }

Motion::Motion() : Motion(PIN_NOT_SET, PIN_NOT_SET) { }

void Motion::Tick()
{
    double dt = millis() - t;
    t = millis();
    
    if (updated)
    {
        double velocityDifference;

        // Difference in wheel speeds calculated using wheel separation compared to turn radius
        // Wheel speed still positive for negative R because you divide it by itself.
        // Basically signs handle themselves here
    
        if (turnRadius != 0)
        {
            velocityDifference = WHEEL_SEPARATION / 2 / turnRadius;
        }
        else
        {
            velocityDifference = 0;
        }

        double rightWheelSpeed = speed * (1 + velocityDifference);
        double leftWheelSpeed = speed * (1 - velocityDifference);

        rightMotor->setSpeedFine((uint16_t) (rightWheelSpeed * VELOCITY_TO_MOTOR_SPEED));
        rightMotor->run(rightWheelSpeed > 0 ? FORWARD : BACKWARD);

        leftMotor->setSpeedFine((uint16_t) (leftWheelSpeed * VELOCITY_TO_MOTOR_SPEED));
        leftMotor->run(leftWheelSpeed > 0 ? FORWARD : BACKWARD);

        Serial.print("New wheel speeds: ");
        Serial.print(leftWheelSpeed);
        Serial.print(" O-O ");
        Serial.print(rightWheelSpeed);
        Serial.println(" ( / 4095 )");

        updated = false;
    }

    distance += speed * dt / 1000;
    if (turnRadius != 0)
    {
        // Minus because +ve R indicates turning left
        bearing -= speed / turnRadius * dt / 1000;
    }
}

double Motion::GetDistance() { return distance; }

double Motion::GetDeltaY()
{
    double deltaY = distance - lastDistance;
    lastDistance = distance;
    return deltaY;
}

double Motion::GetTargetSpeed() { return speed; }
double Motion::GetTrueSpeed() { return speed; }
void Motion::SetSpeed(double _speed)
{
    speed = _speed;
    updated = true;
}

double Motion::GetBearing() { return bearing; }

double Motion::GetTurnRadius() { return turnRadius; }
void Motion::SetTurnRadius(double _turnRadius)
{
    turnRadius = _turnRadius;
    updated = true;
}
