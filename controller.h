#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "navigation.h"
#include "motion.h"
#include "leds.h"
#include "ultrasound.h"

#define     DUMMY_DISTANCE_THRESHOLD    50

#define     IR_SAMPLE_TOTAL_TIME        78000       // us
#define     IR_TIME_BETWEEN_SAMPLES     650         // us

class Controller
{
    private:
        enum controller_state_t
        {
            FIRST_EMBARKMENT = 0,
            DUMMY_DETECTION = 0xD
        };

        Motion *motion;
        Navigation *navigation;
        Leds *leds;
        UltrasoundSensor *ultrasoundSensor;

        controller_state_t controllerState;

        unsigned long t;
        bool isMoving;
        bool started;
    
    public:
        Controller(Motion *, Navigation *, UltrasoundSensor *, Leds *);
        Controller();

        void Tick();
        void Start();
};

#endif