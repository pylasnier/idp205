#ifndef LEDS_H
#define LEDS_H

#define     AMBER_LED_PERIOD        250

#include "robot.h"

class Leds
{
    private:
        pin_size_t amberLedPin;
        pin_size_t redLedPin;
        pin_size_t greenLedPin;

        unsigned long amberLedTimer;
        bool amberLedOn;

        bool moving;
    
    public:
        Leds(pin_size_t, pin_size_t, pin_size_t);
        Leds();

        void SetDummy(dummy_t);
        void MovingStart();
        void MovingStop();

        void Tick();
};

#endif