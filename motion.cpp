#include "motion.h"
#include "robot.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Motion::Motion(Adafruit_MotorShield AFSM, uint8_t leftMotorPort, uint8_t rightMotorPort)
{
    leftMotor = AFSM.getMotor(leftMotorPort);
    rightMotor = AFSM.getMotor(rightMotorPort);
    
    speed = 0;
    distance = 0;
    lastDistance = 0;
    turnRadius = 0;
    bearing = 0;

    updated = false;

    t = millis();
}

Motion::Motion(uint8_t leftMotorPort, uint8_t rightMotorPort) : Motion(Adafruit_MotorShield(), leftMotorPort, rightMotorPort) { }

Motion::Motion() : Motion(PIN_NOT_SET, PIN_NOT_SET) { }

void Motion::Tick()
{
    double dt = millis() - t;

    if (updated)
    {
        // Difference in wheel speeds calculated using wheel separation compared to turn radius
        // Wheel speed still positive for negative R because you divide it by itself.
        // Basically signs handle themselves here
        double rightWheelSpeed = speed * (turnRadius + WHEEL_SEPARATION / 2) / turnRadius;
        double leftWheelSpeed = speed * (turnRadius - WHEEL_SEPARATION / 2) / turnRadius;

        rightMotor->run(rightWheelSpeed > 0 ? FORWARD : BACKWARD);
        rightMotor->setSpeedFine((uint16_t) (rightWheelSpeed * VELOCITY_TO_MOTOR_SPEED));

        leftMotor->run(leftWheelSpeed > 0 ? FORWARD : BACKWARD);
        leftMotor->setSpeedFine((uint16_t) (leftWheelSpeed * VELOCITY_TO_MOTOR_SPEED));

        Serial.print("New wheel speeds: ");
        Serial.print(leftWheelSpeed);
        Serial.print(" O-O ");
        Serial.print(rightWheelSpeed);
        Serial.println(" ( / 4095 )");

        updated = false;
    }

    distance += speed * dt;
    if (turnRadius != 0)
    {
        // Minus because +ve R indicates turning left
        bearing -= speed / turnRadius * dt;
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
