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

    calibrator = new Calibrator(this, motion, leftLineSensor, rightLineSensor);
    trackFollower = new TrackFollower(this, motion, leftLineSensor, rightLineSensor);

    isPivoting = false;
    isCalibrating = false;

    // navigationState = TRACK_FOLLOWING;
    // trackFollower->Enable();

    // Calibrate(RIGHT);
    navigationState = STOPPED;
}

Navigation::Navigation() : Navigation(new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Tick()
{
    bool leftContact = leftLineSensor->Line();
    bool rightContact = rightLineSensor->Line();

    // Smart pivot, tries to stay on a line if there is one
    // Pivot takes priority over calibration, so everything can call it
    if (isPivoting)
    {
        if (fabs(pivotToAngle - motion->GetBearing()) < STRAIGHT_THETA_MARGIN/* && !leftContact && !rightContact*/)
        {
            motion->Stop();
            isPivoting = false;
            Serial.println("Done!");
        }
        else
        {
            // Go forwards or backwards, depending on which sensor falls off line
            // and which way we wanted to go (left or right)
            if (leftContact && !rightContact && pivotState == PIVOTING)
            {
                motion->SetTurnRadius(0);
                motion->SetSpeed(pivotToAngle > motion->GetBearing() ? CRUISE_SPEED : -CRUISE_SPEED);
                Serial.println(pivotToAngle > motion->GetBearing() ? "Forward" : "Back");
                pivotState = TRANSLATING;
            }
            else if (!leftContact && rightContact && pivotState == PIVOTING)
            {
                motion->SetTurnRadius(0);
                motion->SetSpeed(pivotToAngle > motion->GetBearing() ? -CRUISE_SPEED : CRUISE_SPEED);
                Serial.println(pivotToAngle > motion->GetBearing() ? "Back" : "Forward");
                pivotState = TRANSLATING;
            }

            else if (pivotState == TRANSLATING)
            {
                motion->SetPivotTurnRate((pivotToAngle > motion->GetBearing() ? DEFAULT_PIVOT_TURN_RATE : -DEFAULT_PIVOT_TURN_RATE));
                Serial.println("Pivoting");
                pivotState = PIVOTING;
            }
        }
    }

    else if (isCalibrating)
    {
        calibrator->Tick();
        if (calibrator->IsCalibrated())
        {
            motion->Stop();
            isCalibrating = false;
        }
    }
    else
    {
        switch (navigationState)
        {
            case TRACK_FOLLOWING:
                trackFollower->Tick();
                break;

            case STOPPED:
                break;
            
            default: break;
        }
    }
}

void Navigation::StartTrackFollowing()
{
    navigationState = TRACK_FOLLOWING;
    trackFollower->Enable();
}

void Navigation::Pivot(double theta)
{
    // lastNavigationState = navigationState;
    // navigationState = PLS360;
    isPivoting = true;

    pivotToAngle = motion->GetBearing() + theta;
    motion->SetPivotTurnRate((theta > 0 ? DEFAULT_PIVOT_TURN_RATE : -DEFAULT_PIVOT_TURN_RATE));
    pivotState = PIVOTING;
}

void Navigation::Calibrate(desired_direction_t desiredDirection)
{
    isCalibrating = true;

    calibrator->Start(desiredDirection);
}

void Navigation::Stop()
{
    trackFollower->Disable();
    isCalibrating = false;
    isPivoting = false;
    motion->Stop();
    navigationState = STOPPED;
}