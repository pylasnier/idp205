#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "wheelencoder.h"
#include "robot.h"

WheelEncoder::WheelEncoder(LineSensor *_lineSensor)
{
    lineSensor = _lineSensor;

    distancePerDivision = (double) CIRCUMFERENCE / (double) DIVISIONS;

    passed = 0;
    lastPassed = 0;
    interpolatedDistanceSinceLastDeltaY = 0;

    speed = 0;
    speedFromStartMean = 0;
    pointsForFirstMean = 0;

    // You can't read from pins during setup. Annoying
    //onWhite = lineSensor->Line();
    onWhite = false;
    firstRead = true;
    t = millis();                   // Measure if Line holds for threshold time
    timeOfLastChange = millis();    // Time since last division change

    motorValue = 0;
    speedMotorValueConversion = INITIAL_CONVERSION_VALUE;
}

WheelEncoder::WheelEncoder() : WheelEncoder(new LineSensor(PIN_NOT_SET)) { }

unsigned long WheelEncoder::GetDivisionsPassed() { return passed; }
unsigned long WheelEncoder::GetTimeSinceChange() { return millis() - timeOfLastChange; }

double WheelEncoder::GetSpeed() { return speed; }

double WheelEncoder::GetDeltaY()
{
    double distance;

    distance = (passed / DIVISIONS - lastPassed / DIVISIONS) * CIRCUMFERENCE + interpolatedDistanceSinceLastDeltaY;
    interpolatedDistanceSinceLastDeltaY = 0;

    lastPassed = passed;

    return distance;
}

uint16_t WheelEncoder::GetMotorValue(double targetSpeed)
{
    speed = targetSpeed;
    motorValue = (uint16_t) fabs(targetSpeed * speedMotorValueConversion);

    return abs(motorValue);
}

uint16_t WheelEncoder::GetMotorValue() { return motorValue; }
int WheelEncoder::GetMotorDirection() { return ((speed * speedMotorValueConversion) >= 0 ? FORWARD : BACKWARD); }

// This Tick updates:
//  Division passed using threshold to verify pass
//  True speed from divisions passed
//  Resets if wheel appears to have stopped
void WheelEncoder::Tick()
{
    int directionStep;

    double deltaT;
    double divisionPassSpeed;

    if (firstRead)
    {
        onWhite = lineSensor->Line();
        firstRead = false;
        t = millis();
        timeOfLastChange = millis();
    }

    if (onWhite != lineSensor->Line())
    {
        // If confirmed colour change
        if (millis() - t > ENCODER_TIME_THRESHOLD)
        {
            onWhite = !onWhite;
            directionStep = (speed >= 0 ? 1 : -1);
            passed += directionStep;
            // Serial.print(passed);
            // Serial.print(" (deltaT = ");

            deltaT = millis() - timeOfLastChange;
            divisionPassSpeed = distancePerDivision / deltaT;
            // Serial.print(deltaT);
            // Serial.println(")");

            // Calculates initial speed from first 4 points
            if (pointsForFirstMean < PASSES_UNTIL_FIRST_MEAN)
            {
                pointsForFirstMean++;

                speedFromStartMean += divisionPassSpeed;
                if (pointsForFirstMean == PASSES_UNTIL_FIRST_MEAN)
                {
                    speedFromStartMean /= PASSES_UNTIL_FIRST_MEAN;

                    updateTrueSpeed(speedFromStartMean);
                }
            }

            // For a sudden change in speed, reset mean because it might need to be updated rapidly
            else if (fabs(divisionPassSpeed / speed) > CHANGE_FOR_MEAN_RESET)
            {
                resetSpeedMean();
            }

            // After initial speed determined, update using average every pass
            else
            {
                updateTrueSpeed((speed + DIVISION_MEAN_WEIGHTING * distancePerDivision / deltaT) / (1.0f + DIVISION_MEAN_WEIGHTING));
            }

            // Also, only update interpolated distance if there has been a change
            interpolatedDistanceSinceLastDeltaY += speed * deltaT;
            if (passed % DIVISIONS == 0)
            {
                // Because it should only measure distance between revolutions
                interpolatedDistanceSinceLastDeltaY -= CIRCUMFERENCE * directionStep;
            }

            t = millis();
            timeOfLastChange = millis();
        }
    }
    else
    {
        t = millis();
    }

    // Reset if the wheel stops turning
    if (millis() - timeOfLastChange > MAX_WAIT_TIME_FOR_PASS)
    {
        // Previous motor conversion rate is maintained in this situation
        updateTrueSpeed(0);

        resetSpeedMean();

        timeOfLastChange = millis();
    }
}

void WheelEncoder::updateTrueSpeed(double trueSpeed)
{
    speed = trueSpeed;

    // Value probably won't be accurate at lower speeds.
    // Also prevents division by 0
    if (fabs(speed) > SPEED_CALIBRATION_THRESHOLD)
    {
        speedMotorValueConversion = (double) motorValue / speed;
        speedMotorValueConversion = (speedMotorValueConversion > MINIMUM_CONVERSION_VALUE ? speedMotorValueConversion : MINIMUM_CONVERSION_VALUE);
    }
}

void WheelEncoder::resetSpeedMean()
{
    pointsForFirstMean = 0;
    speedFromStartMean = 0;
}

