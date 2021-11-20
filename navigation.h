#ifndef NAVIGATION_H
#define NAVIGATION_H

#define     SENSOR_SEPARATION       20      // 'd' in analysis. Given in mm
#define     LINE_THICKNESS          17      // 't' in analysis. Given in mm

#define     ON_LINE_TURN_RADIUS     100     // 'R' in analysis, should be less than dt/(d-t). Can use smaller theoretical d. Given in mm
#define     OFF_LINE_TURN_RADIUS    200     // 'R' in analysis, but for off line; 2000 is reasonable limit. Given in mm

#define     CRUISE_SPEED            55      // 'v' in analysis. Given in mm/s

#define     INITIAL_THETA_WEIGHT    0
#define     DISTANCE_THETA_WEIGHT   1

#define     STRAIGHT_THETA_MARGIN   0.02f   // Margin of theta for which robot is considered straight

#define     NAVIGATION_LINE_THRESHOLD   250

#include <Wire.h>
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"

enum navigation_state_t
{
    ON_TRACK = 0,
    CONTACT = 1,
    DOUBLE_CONTACT = 2,
    LOST = 3
};

class Navigation
{
    private:
        Motion *motion;

        LineSensor *leftLineSensor;
        LineSensor *rightLineSensor;

        //UltrasoundSensor distanceSensor;

        navigation_state_t navigationState;
        bool firstContact;                  // True on start, false after first contact made. Helps weight
        int contactDirectionMultiplier;     // 1 or -1 depending on which sensor made contact, giving direction to turn

        // Desired bearing, which is unknown but is 'guessed' and updated each tick.
        // motion.GetBearing() - this = theta, which is what is considered in calculations.
        double lineBearing;

        // Initial guess at theta when running onto the line.
        double initialTheta;
        // Bearing at line contact
        double initalBearing;

    public:
        Navigation(Motion *, LineSensor *, LineSensor */*, UltrasoundSensor*/);
        Navigation();

        void Tick();
};

#endif