#include "navigation.h"

Navigation::Calibrator::Calibrator(Navigation *navigation, Motion *_motion, LineSensor *_leftLineSensor, LineSensor *_rightLineSensor)
{
    enclosingNavigation = navigation;
    motion = _motion;
    leftLineSensor = _leftLineSensor;
    rightLineSensor = _rightLineSensor;

    calibratorState = DONE_CALIBRATING;

    waitingOnPivot = false;
}

Navigation::Calibrator::Calibrator() : Calibrator(new Navigation(), new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Calibrator::Tick()
{
    double angleDiscrepancy;

    bool currentLineLeft = leftLineSensor->Line();
    bool currentLineRight = rightLineSensor->Line();

    switch (calibratorState)
    {
        case CALIBRATING:
            if (motion->GetTargetSpeed() != CRUISE_SPEED)
            {
                motion->SetSpeed(CRUISE_SPEED);
            }
            if (motion->GetTargetTurnRadius() != 0)
            {
                motion->SetTurnRadius(0);
            }

            if (waitingOnPivot)
            {
                calibrationTimer = millis();
                waitingOnPivot = false;
            }

            // This section waits for a confident line detection then sets which sensor hit
            if (!(leftContact || rightContact))
            {
                if (currentLineLeft && currentLineRight)
                {
                    calibrationTimer = millis();
                    calibratorState = RESET_CALIBRATION;
                }
                else if (currentLineLeft || currentLineRight)   // Effectively XOR
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        leftContact = currentLineLeft;
                        rightContact = currentLineRight;

                        // Start deltaY measurement
                        motion->GetDeltaY();

                        Serial.print("Contact ");
                        Serial.print((leftContact ? "left" : "right"));
                        Serial.println(". Wait until next contact");
                    }
                }

                // If it takes too long, do a quick 180 and get looking further
                else if (millis() - calibrationTimer > calibrationWaitTime)
                {
                    calibrationWaitTime *= 2;
                    calibrationTimer = millis();

                    enclosingNavigation->Pivot((motion->GetBearing() > 0 ? -PI : PI)); // Will turn left or right depending on bearing. Keeps it around 0 and not off far
                    waitingOnPivot = true;
                }
                else
                {
                    initialChangeTime = millis();
                }
            }

            // Now align with line based on bearing previously set
            else if (leftContact && rightContact)
            {
                calibratorState = DONE_CALIBRATING;
            }

            // Once first contact has been established. Effectively XOR because AND case above
            else
            {
                // Same as above, needs to verify contact
                if (currentLineLeft && currentLineRight)
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        angleDiscrepancy = (leftContact ? -1 : 1) * atan(motion->GetDeltaY() / (double) SENSOR_SEPARATION);

                        // Right if greater, left if smaller
                        enclosingNavigation->Pivot((leftDesired ? -PI : PI) / 2.0f + angleDiscrepancy);

                        // This takes us to the above else if statement
                        leftContact = true;
                        rightContact = true;

                        Serial.println("Double contact! Start turning");
                        Serial.print("Angle discrepancy: ");
                        Serial.println(angleDiscrepancy * 180.0f / PI);
                    }
                }

                // This should only happen if pretty much on track already
                // The odd choice of logic is just in case it quickly swaps over
                // to contact on the other side, which wouldn't be caught just by checking
                // if one has contact
                else if ((!currentLineLeft) && leftContact || (!currentLineRight) && rightContact)
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        motion->Stop();
                        calibratorState = DONE_CALIBRATING;
                    }
                }

                else
                {
                    initialChangeTime = millis();
                }
            }

            break;
        
        case RESET_CALIBRATION:
            if (motion->GetTargetSpeed() != -CRUISE_SPEED)
            {
                motion->SetSpeed(-CRUISE_SPEED);
            }

            // Needs to come off completely
            if (!(currentLineLeft || currentLineRight))
            {
                if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                {
                    calibratorState = CALIBRATING;
                }
            }
            else
            {
                initialChangeTime = millis();
            }

            break;

        case DONE_CALIBRATING:
            Serial.println("Done!");
        
        default: break;
    }
}

void Navigation::Calibrator::Start(desired_direction_t desiredDirection)
{
    leftDesired = (desiredDirection == LEFT ? true : false);

    leftContact = false;
    rightContact = false;

    calibrationWaitTime = CALIBRATION_RESET_WAIT_TIME;
    initialChangeTime = millis();
    calibrationTimer = millis();

    calibratorState = CALIBRATING;
}

bool Navigation::Calibrator::IsCalibrated()
{
    return calibratorState == DONE_CALIBRATING;
}