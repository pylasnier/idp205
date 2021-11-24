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

    calibrator = Calibrator(this, motion, leftLineSensor, rightLineSensor);
    trackFollower = TrackFollower(this, motion, leftLineSensor, rightLineSensor);

    navigationState = CALIBRATING;
    calibrator.Start(LEFT);

    // OR

    // navigationState = TRACK_FOLLOWING;
    // trackFollower.Enable();
}

Navigation::Navigation() : Navigation(new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Tick()
{
    bool leftContact = leftLineSensor->Line();
    bool rightContact = rightLineSensor->Line();

    switch (navigationState)
    {
        case CALIBRATING:
            calibrator.Tick();
            break;
        
        case TRACK_FOLLOWING:
            trackFollower.Tick();
            break;
        
        case PLS360:
            if (fabs(pivotToAngle - motion->GetBearing()) < STRAIGHT_THETA_MARGIN)
            {
                motion->Stop();
                navigationState = lastNavigationState;
            }
            break;
        
        default: break;
    }
}

void Navigation::Pivot(double theta)
{
    lastNavigationState = navigationState;
    navigationState = PLS360;

    pivotToAngle = motion->GetBearing() + theta;
    motion->SetPivotTurnRate((theta > 0 ? DEFAULT_PIVOT_TURN_RATE : -DEFAULT_PIVOT_TURN_RATE));
}