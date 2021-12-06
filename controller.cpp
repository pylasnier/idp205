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
        // Tick all time-dependant components
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

        // Servo control co-routines
        else if (pickingUp)
        {
            // Only attempts to update the servo once every so often (still very often)
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

        // Main switch, each stage of the journey is controlled here
        else switch (controllerState)
        {
            case FIRST_EMBARKMENT:
                // Check IR in case what ultrasound is detecting is the ramp, and not a dummy
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
                    // The robot isn't close enough to pick up the dummy at this point.
                    // It moves forward until it is
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
                        // Depending on the dummy, the robot may need to turn back
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
                            // This is a repeated set of lines you will see more of.
                            // Pivoting 180 degrees is inconsistent, so instead the robot
                            // Turns off and away backwards to 90 degrees then tries to find
                            // the line again, which it can do consistently from about 90 degrees
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
                    // This is the second part to the repeated lines from above
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
                    // Branch in procedure where the baby dummy can be dropped off immediately,
                    // but the others are in boxes on either side of the junction which need to be
                    // traversed to next (parent drop-off)
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
                // This is probably where the bug in competition was,
                // this code doesn't wait at all before checking again if
                // it is on a junction. It still is so immediately stops.
                // Also forgot to drop the dummy off at all, so nothing happens
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
                // Depending on where we came from, we may need to pass
                // a junction before reaching the final box, which must
                // be tracked
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
                // Gets in box sufficiently
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

// Red button
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

// These methods initialise the dummy pick-up/drop-off coroutines
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