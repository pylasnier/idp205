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

    navigationState = CALIBRATING;
}

Navigation::Navigation() : Navigation(new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Tick()
{
    switch (navigationState)
    {
        case CALIBRATING:
            break;
        
        case TRACK_FOLLOWING:
            trackFollower.Tick();
            break;
        
        default: break;
    }
}