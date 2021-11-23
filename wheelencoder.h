#ifndef WHEELENCODER_H
#define WHEELENCODER_H

#define     DIVISIONS                   24
#define     CIRCUMFERENCE               217

#define     ENCODER_TIME_THRESHOLD      50      // Length of signal (ms) from line sensor to be considered real
#define     MAX_WAIT_TIME_FOR_PASS      500     // Length of time before considers wheel speed 0.

#define     PASSES_UNTIL_FIRST_MEAN     4
#define     DIVISION_MEAN_WEIGHTING     0.02    // How much each division contributes to mean
// For 0.03, about 24 divisions will comprise 50% of the mean.
// At around 9 rpm, this damps oscillations due to variance in division size by 90%.

#define     CHANGE_FOR_MEAN_RESET       0.2     // Ratio
#define     SPEED_CALIBRATION_THRESHOLD 0.03    // m/s

#define     INITIAL_CONVERSION_VALUE    37000   // Gave approx 0.11 m/s for max motor output

#define     ENCODER_LINE_THRESHOLD      75

#include <Wire.h>
#include "line_detection.h"

class WheelEncoder
{
    private:
        LineSensor *lineSensor;

        double distancePerDivision; // So we only need to calculate it once
        unsigned long passed;
        unsigned long lastPassed;       // Take difference to get distance change
        double interpolatedDistanceSinceLastDeltaY;     // Helps in calculating distance.
        // Because of encoder pattern inaccuracy, only entire revolutions are reliable enough to take
        // proper distance measurements, so between these the amount passed must be interpolated.

        unsigned long timeOfLastChange; // Milliseconds (time since last change)

        double speed;                   // mm/s
        double speedFromStartMean;
        int pointsForFirstMean;

        bool firstRead;
        bool onWhite;
        unsigned long t;        // Milliseconds (sees how long a signal lasts to verify it)


        uint16_t motorValue;
        double speedMotorValueConversion;

        // Here because it happens a couple times. Sets interpolated speed to true speed
        // and updates conversion ratio.
        void updateTrueSpeed(double);

        // Happens twice two (see what I did there? jks it was a typo but i left it)
        void resetSpeedMean();

    public:
        WheelEncoder(LineSensor *);
        WheelEncoder();

        unsigned long GetDivisionsPassed();
        unsigned long GetTimeSinceChange();

        double GetSpeed();
        double GetDeltaY();
        
        // Give target speed, returns motor value based on calibration
        uint16_t GetMotorValue(double);

        // Or get motor value without updating speed
        uint16_t GetMotorValue();
        int GetMotorDirection();

        void Tick();
};

#endif