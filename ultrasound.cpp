#include <Wire.h>

#include "robot.h"
#include "ultrasound.h"

UltrasoundSensor::UltrasoundSensor(pin_size_t _ultrasound_out, pin_size_t _ultrasound_in)
{
    ultrasound_out = _ultrasound_out;
    ultrasound_in = _ultrasound_in;

    pinMode(ultrasound_out, OUTPUT);
    pinMode(ultrasound_in, INPUT);

    enabled = false;
    distance = ULONG_MAX;

    status = RESET;
}

void UltrasoundSensor::Tick()
{
    if (enabled)
    {
        switch (status)
        {
            case RESET:
                t = micros();
                digitalWrite(ULTRASOUND_OUT, HIGH);
                status = TRIGGER_INPUT;
                break;
            
            case TRIGGER_INPUT:
                if (t - micros() > TRIGGER_INPUT_PULSE_LENGTH)
                {
                    digitalWrite(ULTRASOUND_OUT, LOW);
                    status = AWAITING_RESPONSE;
                }
                break;
            
            case AWAITING_RESPONSE:
                if (digitalRead(ULTRASOUND_IN) == HIGH)
                {
                    t = micros();
                    status = TIMING_RESPONSE;
                }
                break;
            
            case TIMING_RESPONSE:
                if (digitalRead(ULTRASOUND_IN) == LOW)
                {
                    distance = (unsigned long) ((double) (t - micros()) / 5.8f);
                    status = DELAY;
                }
                break;
            
            case DELAY:
                if (t - micros() > DELAY_WAIT_LENGTH)
                {
                    status = RESET;
                }
                break;
            
            default: break;
        }
    }
}

void UltrasoundSensor::Enable()
{
    enabled = true;
    status = RESET;
}

void UltrasoundSensor::Disable()
{
    enabled = false;
    distance = ULONG_MAX;
    digitalWrite(ULTRASOUND_OUT, LOW);
    status = RESET;
}

unsigned long UltrasoundSensor::GetDistance()
{
    return distance;
}