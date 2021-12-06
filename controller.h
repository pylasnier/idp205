#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "navigation.h"
#include "motion.h"
#include "leds.h"
#include "ultrasound.h"
#include "Servo.h"

#define     DUMMY_DISTANCE_THRESHOLD    50

#define     IR_SAMPLE_TOTAL_TIME        78000       // us
#define     IR_TIME_BETWEEN_SAMPLES     650         // us
#define     DUMMY_IDENTIFICATION_WAIT_TIME      5000

#define     SERVO_DELAY_TIME            10
#define     PICK_TO_LIFT_PAUSE_TIME     1000

#define     PINCER_TRAVERSE_RATE        2
#define     PINCER_START_POSITION       20
#define     PINCER_END_POSITION         55

#define     ELEVATION_TRAVERSE_RATE     -0.5
#define     ELEVATION_START_POSITION    170
#define     ELEVATION_END_POSITION      90

#define     PAPA_DUMMY_THRESHOLD        1700
#define     MAMA_DUMMY_THRESHOLD        1250
#define     BABY_DUMMY_THRESHOLD        400

class Controller
{
    private:
        enum controller_state_t
        {
            FIRST_EMBARKMENT = 0,
            DUMMY_DETECTION = 0xD,
            DUMMY_PICKUP = 9,
            ON_TRACK_DUMMY_RECALIBRATION = 1,
            DUMMY_EXTRACTION = 2,
            BABY_DROPOFF_RECALIBRATION = 0xB,
            PARENT_DROPOFF = 99,
            PARENT_DROPOFF_RECALIBRATION = 9000,
            THE_LONG_JOURNEY_BACK = 69,
            BOX_ME = 420,
            DEATH = 666
        };

        Motion *motion;
        Navigation *navigation;
        Leds *leds;
        UltrasoundSensor *ultrasoundSensor;

        Servo pincer;
        Servo pulley;

        // Various subroutine variables
        bool paused;
        unsigned long pauseTimer;

        bool pickingUp;
        bool droppingOff;
        bool transitionReached;
        bool picked;
        unsigned long pickingTimer;

        double pincerPosition;
        double pulleyPosition;

        controller_state_t controllerState;

        unsigned long t;
        bool isMoving;
        bool started;

        dummy_t dummy;
        bool isFirstSample;
        int finalTsopSample;

        double targetAngle;

        double lastDistance;
        bool firstJunctionPassed;
    
    public:
        Controller(Motion *, Navigation *, UltrasoundSensor *, Leds *);
        Controller();

        void Tick();
        void Start();
        void Pause(unsigned long);

        void PickUp();
        void DropOff();
};

#endif