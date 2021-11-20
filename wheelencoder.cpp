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
    deltaYSinceLastChange = 0;

    speed = 0;
    speedFromStartMean = 0;

    // You can't read from pins during setup. Annoying
    //onWhite = lineSensor->Line();
    onWhite = false;
    firstRead = true;
    t = millis();
    timeOfLastChange = millis();

    pointsForFirstMean = 0;
    motorValue = 0;
    speedMotorValueConversion = INITIAL_CONVERSION_VALUE;
}

WheelEncoder::WheelEncoder() : WheelEncoder(new LineSensor(PIN_NOT_SET, ENCODER_LINE_THRESHOLD)) { }

unsigned long WheelEncoder::GetDivisionsPassed() { return passed; }
unsigned long WheelEncoder::GetTimeSinceChange() { return millis() - timeOfLastChange; }

double WheelEncoder::GetSpeed() { return speed; }

double WheelEncoder::GetDeltaY()
{
    double distance;

    // -1 because whole segments passed is -1 the number of segment boundaries passed
    distance = (passed - lastPassed - 1) * distancePerDivision;
    lastPassed = passed;

    // These values are very small but will give smooth interpolated distance values
    // between passes of divisions.
    distance += distancePerDivision - deltaYSinceLastChange;
    deltaYSinceLastChange = (millis() - timeOfLastChange) * speed;
    distance += deltaYSinceLastChange;

    // Proving this works when taking deltaY between divisions:
    // If no more divisions passed, distance = -distancePerDivison
    // Adding second line, distance = -deltaYSinceLastChange
    // Then adding NEW deltaYSinceLastChange so gives
    // difference between small deltaY change :)

    return distance;
}

uint16_t WheelEncoder::GetMotorValue(double targetSpeed)
{
    speed = targetSpeed;
    motorValue = (uint16_t) (targetSpeed * speedMotorValueConversion);

    return abs(motorValue);
}

uint16_t WheelEncoder::GetMotorValue() { return abs(motorValue); }
int WheelEncoder::GetMotorDirection() { return (motorValue >= 0 ? FORWARD : BACKWARD); }

// This Tick updates:
//  Division passed using threshold to verify pass
//  True speed from divisions passed
//  Resets if wheel appears to have stopped
void WheelEncoder::Tick()
{
    double deltaT;
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
            passed += (speed >= 0 ? 1 : -1);
            // Serial.print(passed);
            // Serial.print(" (deltaT = ");

            deltaT = (millis() - timeOfLastChange) / 1000.0f;
            // Serial.print(deltaT);
            // Serial.println(")");

            // Calculates initial speed from first 4 points
            if (pointsForFirstMean < PASSES_UNTIL_FIRST_MEAN)
            {
                pointsForFirstMean++;

                speedFromStartMean += distancePerDivision / deltaT;
                if (pointsForFirstMean == PASSES_UNTIL_FIRST_MEAN)
                {
                    speedFromStartMean /= PASSES_UNTIL_FIRST_MEAN;

                    updateTrueSpeed(speedFromStartMean);
                }
            }

            // After initial speed determined, update using average every pass
            else
            {
                updateTrueSpeed((speed + distancePerDivision / deltaT) / 2.0f);
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

        speedFromStartMean = 0;
        pointsForFirstMean = 0;

        timeOfLastChange = millis();
    }
}

void WheelEncoder::updateTrueSpeed(double trueSpeed)
{
    speed = trueSpeed;

    // Value probably won't be accurate at lower speeds.
    // Also prevents division by 0
    if (fabs(speed) > 30)
    {
        speedMotorValueConversion = (double) motorValue / speed;
    }
}

