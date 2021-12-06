#include <Wire.h>
#include "leds.h"

Leds::Leds(pin_size_t _amberLedPin, pin_size_t _redLedPin, pin_size_t _greenLedPin)
{
    amberLedPin = _amberLedPin;
    if (amberLedPin != PIN_NOT_SET)
    {
        pinMode(amberLedPin, OUTPUT);
    }
    redLedPin = _redLedPin;
    if (redLedPin != PIN_NOT_SET)
    {
        pinMode(redLedPin, OUTPUT);
    }
    greenLedPin = _greenLedPin;
    if (greenLedPin != PIN_NOT_SET)
    {
        pinMode(greenLedPin, OUTPUT);
    }

    amberLedTimer = millis();
    amberLedOn = false;

    moving = false;
}

Leds::Leds() : Leds(PIN_NOT_SET, PIN_NOT_SET, PIN_NOT_SET) { }

void Leds::SetDummy(dummy_t dummy)
{
    digitalWrite(redLedPin, (dummy == PAPA || dummy == BABY ? HIGH : LOW));
    digitalWrite(greenLedPin, (dummy == MAMA || dummy == BABY ? HIGH : LOW));
}

void Leds::Tick()
{
    if (moving)
    {
        if (millis() - amberLedTimer > AMBER_LED_PERIOD)
        {
            amberLedTimer = millis();
            if (amberLedOn)
            {
                digitalWrite(amberLedPin, LOW);
                amberLedOn = false;
            }
            else
            {
                digitalWrite(amberLedPin, HIGH);
                amberLedOn = true;
            }
        }
    }
}

void Leds::MovingStart()
{
    digitalWrite(amberLedPin, HIGH);
    amberLedOn = true;
    moving = true;
    amberLedTimer = millis();
}

void Leds::MovingStop()
{
    digitalWrite(amberLedPin, LOW);
    amberLedOn = false;
    moving = false;
}

