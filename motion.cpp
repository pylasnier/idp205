#include "motion.h"
#include "robot.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "wheelencoder.h"

Motion::Motion(Adafruit_MotorShield *_AFSM, uint8_t leftMotorPort, uint8_t rightMotorPort, WheelEncoder *_leftWheelEncoder, WheelEncoder *_rightWheelEncoder)
{
    AFSM = _AFSM;
    leftMotor = AFSM->getMotor(leftMotorPort);
    rightMotor = AFSM->getMotor(rightMotorPort);

    leftWheelEncoder = _leftWheelEncoder;
    rightWheelEncoder = _rightWheelEncoder;

    t = millis();
    update = millis();
}

void Motion::Begin()
{
    AFSM->begin();
    
    targetSpeed = 0;
    targetTurnRadius = 0;
    averageTurnRadius = 0;
    pivotTurnRate = 0;
    bearing = 0;
}

Motion::Motion(uint8_t leftMotorPort, uint8_t rightMotorPort) : Motion(new Adafruit_MotorShield(), leftMotorPort, rightMotorPort, new WheelEncoder(), new WheelEncoder()) { }

Motion::Motion() : Motion(PIN_NOT_SET, PIN_NOT_SET) { }

void Motion::Tick()
{
    double velocityDifference;
    double turnRate;

    double rightWheelSpeed;
    double leftWheelSpeed;

    uint16_t rightMotorValue;
    uint16_t leftMotorValue;
    uint16_t highestMotorValue;

    leftWheelEncoder->Tick();
    rightWheelEncoder->Tick();

    // Serial.print("Wheel speeds: ");
    // Serial.print(leftWheelEncoder->GetSpeed());
    // Serial.print(" O-O ");
    // Serial.println(rightWheelEncoder->GetSpeed());

    double dt = millis() - t;
    t = millis();

    // Calibrating motor speed ratios and correcting distance and bearing based on wheel encoder information

    
    // Updating wheel motors based on wheel encoder information
    if (millis() - update > MOTOR_UPDATE_PERIOD)
    {
        update = millis();

        // From speed an turn radius
        if (pivotTurnRate == 0)
        {
            // Difference in wheel speeds calculated using wheel separation compared to turn radius
            if (targetTurnRadius != 0)
            {
                velocityDifference = WHEEL_SEPARATION / (2.0f * targetTurnRadius);
            }
            else
            {
                velocityDifference = 0;
            }

            rightWheelSpeed = targetSpeed * (1 + velocityDifference);
            leftWheelSpeed = targetSpeed * (1 - velocityDifference);
        }
        // If pivoting
        else
        {
            // Signs different here than above because positive turn rate
            // is turning right, but positive R (visually, pointing right from
            // the circle centre at the left) turns left.
            // Need to divide by 1000 because speed in m/s but separation given
            // in mm
            rightWheelSpeed = -pivotTurnRate * WHEEL_SEPARATION / 2000.0f;
            leftWheelSpeed = pivotTurnRate * WHEEL_SEPARATION / 2000.0f;
        }

        rightMotorValue = rightWheelEncoder->GetMotorValue(rightWheelSpeed);
        leftMotorValue = leftWheelEncoder->GetMotorValue(leftWheelSpeed);
        highestMotorValue = (leftMotorValue > rightMotorValue ? leftMotorValue : rightMotorValue);

        // If the motor value is too high, scale down speeds to make it suitable.
        // This inevitably caps the true speed some amount below the target speed,
        // but turn radius should be conserved.
        if (highestMotorValue > MAX_MOTOR_VALUE)
        {
            rightWheelSpeed *= (double) (MAX_MOTOR_VALUE) / (double) highestMotorValue;
            leftWheelSpeed *= (double) (MAX_MOTOR_VALUE) / (double) highestMotorValue;

            rightMotorValue = rightWheelEncoder->GetMotorValue(rightWheelSpeed);
            leftMotorValue = leftWheelEncoder->GetMotorValue(leftWheelSpeed);
        }

        rightMotor->setSpeedFine(rightWheelEncoder->GetMotorValue());
        rightMotor->run(rightWheelEncoder->GetMotorDirection());

        leftMotor->setSpeedFine(leftWheelEncoder->GetMotorValue());
        leftMotor->run(leftWheelEncoder->GetMotorDirection());

        // Serial.print("New wheel speeds: ");
        // Serial.print(leftWheelSpeed);
        // Serial.print(" O-O ");
        // Serial.print(rightWheelSpeed);
        // Serial.println(" ( / ~70 )");
    }

    // The true turn rate is determined from wheel encoder values,
    // giving an approximation to the real turn radius for calculations
    // in the navigation class. When calibrated properly, should closely
    // match the desired turn radius.
    turnRate = (leftWheelEncoder->GetSpeed() - rightWheelEncoder->GetSpeed()) / (double) WHEEL_SEPARATION;
    bearing += turnRate * dt;

    // Update average turn radius if trying to turn
    if (targetTurnRadius != 0)
    {
        // 0.002f is weighting of turn rate in average with respect to time
        averageTurnRadius = (averageTurnRadius - (GetTrueSpeed() / turnRate) * 0.002f * dt) / (1.0f + 0.002f * dt);
    }
}

// Much coarser distance measure for total distance.
double Motion::GetDistance()
{
    return (double) ((leftWheelEncoder->GetDivisionsPassed() + rightWheelEncoder->GetDivisionsPassed()) * CIRCUMFERENCE) / (2.0f * DIVISIONS);
}

double Motion::GetDeltaY()
{
    return (leftWheelEncoder->GetDeltaY() + rightWheelEncoder->GetDeltaY()) / 2.0f;
}

double Motion::GetTargetSpeed() { return targetSpeed; }
double Motion::GetTrueSpeed()
{
    return (leftWheelEncoder->GetSpeed() + rightWheelEncoder->GetSpeed()) / 2.0f;
}

void Motion::SetSpeed(double speed)
{
    targetSpeed = speed;
    pivotTurnRate = 0;
}

double Motion::GetBearing() { return bearing; }

double Motion::GetTargetTurnRadius() { return targetTurnRadius; }
double Motion::GetTrueTurnRadius() { return averageTurnRadius; }

void Motion::SetTurnRadius(double turnRadius)
{
    targetTurnRadius = turnRadius;
    averageTurnRadius = targetTurnRadius;
    pivotTurnRate = 0;
}

void Motion::SetPivotTurnRate(double turnRate)
{
    pivotTurnRate = turnRate;
    targetSpeed = 0;
    targetTurnRadius = 0;
}