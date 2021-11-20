#ifndef WHEELENCODER_H
#define WHEELENCODER_H

#define     DIVISIONS                   24
#define     CIRCUMFERENCE               300

#define     ENCODER_TIME_THRESHOLD      50      // Length of signal (ms) from line sensor to be considered real
#define     MAX_WAIT_TIME_FOR_PASS      500     // Length of time before considers wheel speed 0.

#define     PASSES_UNTIL_FIRST_MEAN     4

#define     INITIAL_CONVERSION_VALUE    69

#define     ENCODER_LINE_THRESHOLD      80

#include <Wire.h>
#include "line_detection.h"

class WheelEncoder
{
    private:
        LineSensor *lineSensor;

        double distancePerDivision; // So we only need to calculate it once
        unsigned long passed;
        unsigned long lastPassed;       // Take difference to get distance change
        unsigned long timeOfLastChange; // Milliseconds (time since last change)
        double deltaYSinceLastChange;   // For a (probably too) accurate deltaY between divisions

        double speed;                   // mm/s
        double speedFromStartMean;

        bool firstRead;
        bool onWhite;
        unsigned long t;        // Milliseconds (sees how long a signal lasts to verify it)

        int pointsForFirstMean;

        uint16_t motorValue;
        double speedMotorValueConversion;

        // Here because it happens a couple times. Sets interpolated speed to true speed
        // and updates conversion ratio.
        void updateTrueSpeed(double);

    public:
        WheelEncoder(LineSensor *);
        WheelEncoder();

        unsigned long GetDivisionsPassed();
        unsigned long GetTimeSinceChange();

        double GetSpeed();
        double GetDeltaY();
        
        // Give target speed, returns motor value;
        // updates calibration
        uint16_t GetMotorValue(double);

        // Or get motor value without updating speed
        // (don't see why this would happen but just in case)
        uint16_t GetMotorValue();
        int GetMotorDirection();

        void Tick();
};

#endif