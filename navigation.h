#ifndef NAVIGATION_H
#define NAVIGATION_H

#define     SENSOR_SEPARATION       25      // 'd' in analysis. Given in mm
#define     LINE_THICKNESS          17      // 't' in analysis. Given in mm

#define     ON_LINE_TURN_RADIUS     150      // 'R' in analysis, should be less than dt/(d-t). Can use smaller theoretical d. Given in mm
#define     OFF_LINE_TURN_RADIUS    200     // 'R' in analysis, but for off line; 2000 is reasonable limit. Given in mm

#define     CRUISE_SPEED            0.08    // 'v' in analysis. Given in mm/s
#define     DEFAULT_PIVOT_TURN_RATE 0.7     // rad/s

#define     INITIAL_THETA_WEIGHT    0
#define     DISTANCE_THETA_WEIGHT   1

#define     STRAIGHT_THETA_MARGIN   0.02f   // Margin of theta for which robot is considered straight

#define     LINE_TIME_THRESHOLD     5       // ms
#define     CALIBRATION_RESET_WAIT_TIME         2000    // ms
#define     NAVIGATION_LINE_THRESHOLD   130

#define     MAX_LINE_ANGLE_DISCREPANCY  0.75

#include <Wire.h>
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"


enum desired_direction_t
{
    LEFT = 0,
    RIGHT = 1
};

class Navigation
{
    private:
        enum navigation_state_t
        {
            TRACK_FOLLOWING = 0,
            STOPPED = 1
        };

        enum pivot_state_t
        {
            PIVOTING = 0,
            TRANSLATING = 1
        };

        class TrackFollower
        {
            private:
                enum track_follower_state_t
                {
                    ON_TRACK = 0,
                    CONTACT = 1,
                    DOUBLE_CONTACT = 2,
                    LOST = 3
                };

                bool enabled;

                Navigation *enclosingNavigation;
            
                Motion *motion;

                LineSensor *leftLineSensor;
                LineSensor *rightLineSensor;

                //UltrasoundSensor distanceSensor;

                track_follower_state_t trackFollowerState;
                bool firstContact;                  // True on start, false after first contact made. Helps weight
                int contactDirectionMultiplier;     // 1 or -1 depending on which sensor made contact, giving direction to turn

                // Desired bearing, which is unknown but is 'guessed' and updated each tick.
                // motion.GetBearing() - this = theta, which is what is considered in calculations.
                double lineBearing;

                // Initial guess at theta when running onto the line.
                double initialTheta;
                // Bearing at line contact
                double initalBearing;

                bool leftContact;
                bool rightContact;

                unsigned long leftInitialChangeTime;       // To verify signal
                unsigned long rightInitialChangeTime;

            public:
                TrackFollower(Navigation *, Motion *, LineSensor *, LineSensor *);
                TrackFollower();

                void Tick();

                void Enable();
                void Disable();

                bool IsAtJunction();
        };

        class Calibrator
        {
            private:
                enum calibrator_state_t
                {
                    CALIBRATING = 0,
                    RESET_CALIBRATION = 1,
                    DONE_CALIBRATING = 2
                };

                Motion *motion;

                Navigation *enclosingNavigation;

                LineSensor *leftLineSensor;
                LineSensor *rightLineSensor;

                calibrator_state_t calibratorState;

                double calibrationBearing;

                bool leftContact;
                bool rightContact;
                bool leftDesired;                   // On calibration, want to go left or right?

                unsigned long initialChangeTime;    // From this time, needs to last until threshold to verify signal
                unsigned long calibrationWaitTime;
                unsigned long calibrationTimer;     // Used to see if time exceeds threshold, and reset needed

                bool waitingOnPivot;

            public:
                Calibrator(Navigation *, Motion *, LineSensor *, LineSensor *);
                Calibrator();

                void Tick();

                void Start(desired_direction_t);
                bool IsCalibrated();
        };

        Motion *motion;

        LineSensor *leftLineSensor;
        LineSensor *rightLineSensor;

        Calibrator *calibrator;
        TrackFollower *trackFollower;

        navigation_state_t navigationState;
        navigation_state_t lastNavigationState;     // If in a state (e.g. pivot) expected to return to the last state, usually from which some subroutine is called

        bool isCalibrating;
        bool isPivoting;
        pivot_state_t pivotState;
        double pivotToAngle;

    public:
        Navigation(Motion *, LineSensor *, LineSensor */*, UltrasoundSensor*/);
        Navigation();

        void Tick();

        void StartTrackFollowing();
        void Pivot(double);
        void Calibrate(desired_direction_t);

        bool IsAtJunction();

        void Stop();
};

#endif