#ifndef ULTRASOUND_H
#define ULTRASOUND_H

#include <Wire.h>

#define     TRIGGER_INPUT_PULSE_LENGTH      10      // Length of trigger pulse in microseconds
#define     DELAY_WAIT_LENGTH               60000   // Length of delay in microseconds
#define     TIMEOUT_LENGTH                  20000   // Length before sensor times out
#define     DISTANCE_CONVERSION_DIVISOR     5.8f    // To calibrate distance calculation. Change as needed


// https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
class UltrasoundSensor
{
    private:
        bool enabled;
        unsigned long distance;     // Distance in millimetres
        unsigned long t;            // Timer using microseconds (lasts ~5 hours)

        pin_size_t ultrasound_trig;
        pin_size_t ultrasound_echo;
    
    public:
        // You pass the out/in pins for the sensor
        UltrasoundSensor(pin_size_t, pin_size_t);
        UltrasoundSensor();
        
        // Call this in loop. It times itself and updates distance accordingly
        void Tick();
        
        void Enable();
        void Disable();
        unsigned long GetDistance();

        // In case you didn't set pins/need to change them
        void SetTrigPin(pin_size_t);
        void SetEchoPin(pin_size_t);
};

#endif