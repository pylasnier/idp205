#include <Wire.h>

#include "robot.h"
#include "ultrasound.h"

UltrasoundSensor::UltrasoundSensor(pin_size_t _ultrasound_trig, pin_size_t _ultrasound_echo)
{
    SetTrigPin(_ultrasound_trig);
    SetEchoPin(_ultrasound_echo);

    enabled = false;
    distance = ULONG_MAX;
    t = micros();
}

UltrasoundSensor::UltrasoundSensor() : UltrasoundSensor(PIN_NOT_SET, PIN_NOT_SET) { }

void UltrasoundSensor::Tick()
{
    // It's required to run the entire detection in one cycle because
    // the Arduino runs slowly enough that a single loop may miss any of the
    // timings required by this sensor. It operates on the order of microseconds,
    // and the Arduino, if it executes a lot of lines in a tick (which it will),
    // operates in 100s of microseconds to milliseconds.

    bool reset = false;

    // Shouldn't ever happen if I don't suck
    if (ultrasound_trig == PIN_NOT_SET || ultrasound_echo == PIN_NOT_SET)
    {
        Serial.write("ULTRASOUND PINS NOT SET");
    }

    // Delay considered here; just don't run if it's too soon
    else if (enabled && micros() - t > DELAY_WAIT_LENGTH)
    {
        // Send a pulse of 10 us by waiting using delayMicroseconds function
        digitalWrite(ultrasound_trig, HIGH);
        delayMicroseconds(TRIGGER_INPUT_PULSE_LENGTH);
        digitalWrite(ultrasound_trig, LOW);

        // Use while loops to delay until digital input changes
        t = micros();
        while (digitalRead(ultrasound_echo) == LOW)
        {
            // This break system is for if the return signal never comes
            if (micros() - t > TIMEOUT_LENGTH)
            {
                reset = true;
                break;
            }
        }
        t = micros();
        if (!reset)
        {
            while (digitalRead(ultrasound_echo) == HIGH);

            distance = (unsigned long) ((double) (micros() - t) / DISTANCE_CONVERSION_DIVISOR);
            t = micros();
        }
        else reset = false;
    }
}

void UltrasoundSensor::Enable()
{
    enabled = true;
}

void UltrasoundSensor::Disable()
{
    enabled = false;
    distance = ULONG_MAX;
    digitalWrite(ultrasound_trig, LOW);
}

unsigned long UltrasoundSensor::GetDistance()
{
    return distance;
}

void UltrasoundSensor::SetTrigPin(pin_size_t _ultrasound_trig)
{
    if (ultrasound_trig != PIN_NOT_SET)
    {
        pinMode(ultrasound_trig, INPUT);
    }

    ultrasound_trig = _ultrasound_trig;

    if (ultrasound_trig != PIN_NOT_SET)
    {
        pinMode(ultrasound_trig, OUTPUT);
    }
}

void UltrasoundSensor::SetEchoPin(pin_size_t _ultrasound_echo)
{
    ultrasound_echo = _ultrasound_echo;
    
    if (ultrasound_echo != PIN_NOT_SET)
    {
        pinMode(ultrasound_echo, INPUT);
    }
}