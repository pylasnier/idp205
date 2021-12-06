#include "controller.h"
#include "robot.h"
#include "motion.h"
#include "navigation.h"
#include "leds.h"
#include "Servo.h"

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

        if (paused)
        {
            if (millis() - t > pauseTimer)
            {
                paused = false;
                t = millis();
            }
        }

        else if (pickingUp)
        {
            if (millis() - pickingTimer > SERVO_DELAY_TIME)
            {
                pickingTimer = millis();
                if (!transitionReached)
                {
                    pincerPosition += PINCER_TRAVERSE_RATE;
                    pincer.write((int) pincerPosition);
                    if (pincerPosition >= PINCER_END_POSITION)
                    {
                        transitionReached = true;
                        Pause(PICK_TO_LIFT_PAUSE_TIME);
                    }
                }
                else
                {
                    pulleyPosition += ELEVATION_TRAVERSE_RATE;
                    pulley.write((int) pulleyPosition);
                    // Inverse equality, the elevation mechanism works backwards
                    if (pulleyPosition <= ELEVATION_END_POSITION)
                    {
                        pickingUp = false;
                        picked = true;
                    }
                }
            }
        }

        else if (droppingOff)
        {
            if (millis() - pickingTimer > SERVO_DELAY_TIME)
            {
                pickingTimer = millis();
                if (!transitionReached)
                {
                    pulleyPosition -= ELEVATION_TRAVERSE_RATE;
                    pulley.write((int) pulleyPosition);
                    // Inverse equality, the elevation mechanism works backwards
                    if (pulleyPosition >= ELEVATION_START_POSITION)
                    {
                        transitionReached = true;
                        Pause(PICK_TO_LIFT_PAUSE_TIME);
                    }
                }
                else
                {
                    pincerPosition -= PINCER_TRAVERSE_RATE;
                    pincer.write((int) pincerPosition);
                    if (pincerPosition <= PINCER_START_POSITION)
                    {
                        droppingOff = false;
                        picked = false;
                    }
                }
            }
        }

        else switch (controllerState)
        {
            case FIRST_EMBARKMENT:
                if (ultrasoundSensor->GetDistance() < 50 && analogRead(IR_DETECTOR_INPUT_PIN) < 900)
                {
                    navigation->Stop();
                    t = millis();
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

                if (!isFirstSample)
                {
                    isFirstSample = true;
                    finalTsopSample = tsopSampleSum;
                }
                else
                {
                    finalTsopSample = (finalTsopSample + 0.05f * tsopSampleSum) / 1.05f;
                }

                if (finalTsopSample > PAPA_DUMMY_THRESHOLD)
                {
                    dummy = PAPA;
                }
                else if (finalTsopSample > MAMA_DUMMY_THRESHOLD)
                {
                    dummy = MAMA;
                }
                else if (finalTsopSample > BABY_DUMMY_THRESHOLD)
                {
                    dummy = BABY;
                }
                else
                {
                    dummy = NONE;
                }

                leds->SetDummy(dummy);

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

                if (millis() - t > DUMMY_IDENTIFICATION_WAIT_TIME)
                {
                    controllerState = DUMMY_PICKUP;
                    navigation->StartTrackFollowing();
                }
                break;
            
            case DUMMY_PICKUP:
                if (digitalRead(ALIGNMENT_SENSOR_PIN))
                {
                    navigation->Stop();
                    PickUp();
                    Serial.println("Picking up!");
                    if (picked)
                    {
                        if (dummy == BABY)
                        {
                            Serial.println("Extracting baby!");
                            controllerState = DUMMY_EXTRACTION;
                            navigation->StartTrackFollowing();
                        }
                        else
                        {
                            Serial.println("Ain't no baby, i'm heading back");
                            controllerState = ON_TRACK_DUMMY_RECALIBRATION;
                            motion->SetSpeed(-CRUISE_SPEED);
                            motion->SetTurnRadius(ON_LINE_TURN_RADIUS);
                            targetAngle = motion->GetBearing() + PI / 2.0f;
                        }
                    }
                }
                break;
            
            case ON_TRACK_DUMMY_RECALIBRATION:
                if (fabs(motion->GetBearing() - targetAngle) < STRAIGHT_THETA_MARGIN)
                {
                    navigation->StartTrackFollowing();
                    navigation->Calibrate(RIGHT);
                    controllerState = DUMMY_EXTRACTION;
                }
                break;
            
            case DUMMY_EXTRACTION:
                if (navigation->IsAtJunction())
                {
                    navigation->Stop();
                    motion->Stop();
                    Serial.println("Reached junction");
                    if (dummy == BABY)
                    {
                        DropOff();
                        if (!picked)
                        {
                            controllerState = BABY_DROPOFF_RECALIBRATION;
                            motion->SetSpeed(-CRUISE_SPEED);
                            motion->SetTurnRadius(ON_LINE_TURN_RADIUS);
                            targetAngle = motion->GetBearing() + PI / 2.0f;
                        }
                    }
                    else if (dummy == MAMA)
                    {
                        navigation->Pivot(-PI);
                        controllerState = PARENT_DROPOFF;
                    }
                    else
                    {
                        navigation->Pivot(PI);
                        Serial.println("Turn me right round");
                        controllerState = PARENT_DROPOFF;
                    }
                }
                break;
            
            case PARENT_DROPOFF:
                if (navigation->IsAtJunction())
                {
                    navigation->Stop();
                    motion->Stop();
                    if (!picked)
                    {
                        controllerState = PARENT_DROPOFF_RECALIBRATION;
                        motion->SetSpeed(-CRUISE_SPEED);
                        firstJunctionPassed = true;
                        delay(100);
                    }
                }
                break;
            
            case BABY_DROPOFF_RECALIBRATION:
                if (fabs(motion->GetBearing() - targetAngle) < STRAIGHT_THETA_MARGIN)
                {
                    firstJunctionPassed = false;
                    navigation->StartTrackFollowing();
                    navigation->Calibrate(RIGHT);
                    controllerState = THE_LONG_JOURNEY_BACK;
                    lastDistance = motion->GetDistance();
                }
                break;
            
            case PARENT_DROPOFF_RECALIBRATION:
                if (navigation->IsAtJunction())
                {
                    navigation->Pivot((dummy == MAMA ? PI : -PI));
                    controllerState = THE_LONG_JOURNEY_BACK;
                    navigation->StartTrackFollowing();
                    lastDistance = motion->GetDistance();
                }
                break;
            
            case THE_LONG_JOURNEY_BACK:
                Serial.println("On my way back");
                if (navigation->IsAtJunction())
                {
                    if (!firstJunctionPassed)
                    {
                        firstJunctionPassed = true;
                        lastDistance = motion->GetDistance();
                    }
                    if (motion->GetDistance() - lastDistance > 200)
                    {
                        lastDistance = motion->GetDistance();
                        navigation->Stop();
                        motion->SetTurnRadius(0);
                        motion->SetSpeed(CRUISE_SPEED);
                    }
                }
                break;
            
            case BOX_ME:
                if (motion->GetDistance() - lastDistance > 220)
                {
                    navigation->Stop();
                    motion->Stop();
                    controllerState = DEATH;
                }
                break;
            
            case DEATH:
                // Serial.println("DONE DONE I'M DONE I'M DONE LMAO");
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
        Serial.println("Button");

        isMoving = false;
        motion->Stop();
        navigation->Stop();

        navigation->StartTrackFollowing();
        // navigation->Calibrate(LEFT);

        // navigation->Pivot(2 * PI);

        ultrasoundSensor->Enable();

        controllerState = FIRST_EMBARKMENT;

        paused = false;
        pickingUp = false;
        droppingOff = false;
        picked = false;

        isFirstSample = false;
    }
}

void Controller::Pause(unsigned long pauseTime)
{
    paused = true;
    t = millis();

    pauseTimer = pauseTime;
}

void Controller::PickUp()
{
    if (!picked)
    {
        pincer.attach(PINCER_SERVO_PIN);
        pincerPosition = PINCER_START_POSITION;
        pulley.attach(ELEVATION_SERVO_PIN);
        pulleyPosition = ELEVATION_START_POSITION;
        pickingTimer = millis();

        pincer.write(pincerPosition);
        pulley.write(pulleyPosition);

        pickingUp = true;
        transitionReached = false;

        picked = true;
    }
}

void Controller::DropOff()
{
    if (picked)
    {
        pincer.attach(PINCER_SERVO_PIN);
        pincerPosition = PINCER_END_POSITION;
        pulley.attach(ELEVATION_SERVO_PIN);
        pulleyPosition = ELEVATION_END_POSITION;
        pickingTimer = millis();

        pincer.write(pincerPosition);
        pulley.write(pulleyPosition);

        droppingOff = true;
        transitionReached = false;

        picked = false;
    }
}