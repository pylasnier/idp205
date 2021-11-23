#include <Wire.h>
#include "navigation.h"
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"

Navigation::Navigation(Motion *_motion, LineSensor *_leftLineSensor, LineSensor *_rightLineSensor)
{
    motion = _motion;
    leftLineSensor = _leftLineSensor;
    rightLineSensor = _rightLineSensor;

    trackFollower = TrackFollower(motion, leftLineSensor, rightLineSensor);

    navigationState = TRACK_FOLLOWING;
    trackFollower.Enable();

    calibrationLeftOnLine = false;
    calibrationRightOnLine = false;

    initialChangeTime = millis();
    calibrationWaitTime = CALIBRATION_RESET_WAIT_TIME;
}

Navigation::Navigation() : Navigation(new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Tick()
{
    bool leftContact = leftLineSensor->Line();
    bool rightContact = rightLineSensor->Line();

    switch (navigationState)
    {
        case CALIBRATING:
            if (motion->GetTargetSpeed() != CRUISE_SPEED)
            {
                motion->SetSpeed(CRUISE_SPEED);
            }

            // This section waits for a confident line detection then sets which sensor hit
            if (!(calibrationLeftOnLine || calibrationRightOnLine))
            {
                if (leftContact && rightContact)
                {
                    calibrationTimer = millis();
                    navigationState = RESET_CALIBRATION;
                }
                else if (leftContact || rightContact)   // Effectively XOR
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        calibrationLeftOnLine = leftContact;
                        calibrationRightOnLine = rightContact;

                        motion->GetDeltaY();
                    }
                }
                // If it takes too long, do a quick 180 and get looking further
                else if (millis() - calibrationTimer > calibrationWaitTime)
                {
                    calibrationWaitTime *= 2;
                    calibrationTimer = millis();

                    calibrationBearing = motion->GetBearing() + (motion->GetBearing() > 0 ? -PI : PI);
                    navigationState = PLS360;
                }
                else
                {
                    initialChangeTime = millis();
                }
            }

            // Now align with line based on bearing previously set
            else if (calibrationLeftOnLine && calibrationRightOnLine)
            {
                if (fabs(calibrationBearing - motion->GetBearing()) < STRAIGHT_THETA_MARGIN)
                {
                    motion->SetSpeed(0);
                    motion->SetTurnRadius(0);
                    navigationState = DONE_CALIBRATING;
                }
                else
                {
                    // Go forwards or backwards, depending on which sensor falls off line
                    // and which way we want to go (left or right)
                    if (leftContact && !rightContact)
                    {
                        motion->SetTurnRadius(0);
                        motion->SetSpeed(leftDesired ? CRUISE_SPEED : -CRUISE_SPEED);
                    }
                    else if (!leftContact && rightContact)
                    {
                        motion->SetTurnRadius(0);
                        motion->SetSpeed(leftDesired ? -CRUISE_SPEED : CRUISE_SPEED);
                    }

                    else
                    {
                        motion->SetPivotTurnRate((calibrationBearing > motion->GetBearing() ? 1 : -1) * DEFAULT_PIVOT_TURN_RATE);
                    }
                }
            }

            // Once first contact has been established. Effectively XOR because AND case above
            else
            {
                // Same as above, needs to verify contact
                if (leftContact && rightContact)
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        // Calculate angle it hits line at, set bearing
                        calibrationBearing = motion->GetBearing() + (leftDesired ? -PI : PI) / 2.0f + (calibrationLeftOnLine ? -1 : 1) * atan(motion->GetDeltaY() / (double) SENSOR_SEPARATION);

                        calibrationLeftOnLine = true;
                        calibrationRightOnLine = true;

                        // Right if greater, left if smaller
                        motion->SetPivotTurnRate((calibrationBearing > motion->GetBearing() ? 1 : -1) * DEFAULT_PIVOT_TURN_RATE);
                    }
                }

                // This should only happen if pretty much on track already
                // The odd choice of logic is just in case it quickly swaps over
                // to contact on the other side, which wouldn't be caught just by checking
                // if one has contact
                else if ((!leftContact) && calibrationLeftOnLine || (!rightContact) && calibrationRightOnLine)
                {
                    if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                    {
                        motion->SetSpeed(0);
                        motion->SetTurnRadius(0);
                        // trackFollower.Enable();
                        // navigationState = TRACK_FOLLOWING;
                        navigationState = DONE_CALIBRATING;
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
            if (!(leftContact || rightContact))
            {
                if (millis() - initialChangeTime > LINE_TIME_THRESHOLD)
                {
                    navigationState = CALIBRATING;
                }
            }
            else
            {
                initialChangeTime = millis();
            }

            break;
        
        case PLS360:
            if (fabs(calibrationBearing - motion->GetBearing()) < STRAIGHT_THETA_MARGIN)
            {
                motion->SetSpeed(0);
                motion->SetTurnRadius(0);
                navigationState = CALIBRATING;
                calibrationTimer = millis();
            }
            break;
        
        case TRACK_FOLLOWING:
            trackFollower.Tick();
            break;

        case DONE_CALIBRATING:
            Serial.println("Done!");
        
        default: break;
    }
}