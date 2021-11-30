#include "controller.h"
#include "robot.h"
#include "motion.h"
#include "navigation.h"
#include "leds.h"

Controller::Controller(Motion *_motion, Navigation *_navigation, UltrasoundSensor *_ultrasoundSensor, Leds *_leds)
{
    motion = _motion;
    navigation = _navigation;
    ultrasoundSensor = _ultrasoundSensor;
    leds = _leds;

    started = false;
}

void Controller::Tick()
{
    unsigned long sampleTimer;      // us
    unsigned long tsopSampleSum;
    unsigned long phototransistorSampleSum;
    int sampleCount;

    if (started)
    {
        ultrasoundSensor->Tick();
        navigation->Tick();
        motion->Tick();
        leds->Tick();

        switch (controllerState)
        {
            case FIRST_EMBARKMENT:
                if (ultrasoundSensor->GetDistance() < 100)
                {
                    navigation->Stop();
                    controllerState = DUMMY_DETECTION;
                }
                break;
            
            case DUMMY_DETECTION:
                sampleTimer = micros();
                tsopSampleSum = 0;
                phototransistorSampleSum = 0;
                sampleCount = 0;

                while (micros() - sampleTimer < IR_SAMPLE_TOTAL_TIME)
                {
                    if ((micros() - sampleTimer) / IR_TIME_BETWEEN_SAMPLES > sampleCount)
                    {
                        tsopSampleSum += 182 - analogRead(TSOP_INPUT_PIN);
                        phototransistorSampleSum += 1023 - analogRead(IR_DETECTOR_INPUT_PIN);
                        sampleCount++;
                    }
                }

                Serial.print("Distance: ");
                Serial.print(ultrasoundSensor->GetDistance());
                Serial.print("   Samples: ");
                Serial.print(sampleCount);
                Serial.print("   Values: ");
                Serial.print(tsopSampleSum);
                Serial.print(" - ");
                Serial.print(phototransistorSampleSum);
                Serial.print("   Ratio: ");
                Serial.println((double) tsopSampleSum / (double) phototransistorSampleSum);
                break;
        }

        
    }

    // if (motion->GetDistance() > 1000)
    // {
    //     navigation->Stop();
    // }

    // if (millis() - t > 1000)
    // {
    //     t = millis();

    //     if (motion->IsMoving())
    //     {
    //         motion->Stop();
    //     }
    //     else
    //     {
    //         motion->SetSpeed(CRUISE_SPEED);
    //     }
    // }

    // if (ultrasoundSensor->GetDistance() < DUMMY_DISTANCE_THRESHOLD || motion->GetDistance() > 2000)
    // {
    //     navigation->Stop();
    //     leds->SetDummy(PAPA);
    // }
}

void Controller::Start()
{
    if (!started)
    {
        started = true;

        isMoving = false;
        motion->Stop();
        navigation->Stop();

        // navigation->StartTrackFollowing();
        // navigation->Calibrate(LEFT);

        // navigation->Pivot(2 * PI);

        ultrasoundSensor->Enable();

        controllerState = DUMMY_DETECTION;
    }
}