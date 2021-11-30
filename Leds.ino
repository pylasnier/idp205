#include "project_defines.h"

#ifdef LEDS

#include "robot.h"
#include "leds.h"

Leds *leds;

void setup()
{
    leds = new Leds(AMBER_LED_OUTPUT_PIN, RED_LED_OUTPUT_PIN, GREEN_LED_OUTPUT_PIN);

    leds->SetDummy(MAMA);
    leds->MovingStart();
}

void loop()
{
    leds->Tick();
}

#endif