#include <Wire.h>
#include "navigation.h"
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"

Navigation::TrackFollower::TrackFollower(Navigation *navigation, Motion *_motion, LineSensor *_leftLineSensor, LineSensor *_rightLineSensor)
{
    //distanceSensor = _distanceSensor;

    enabled = false;

    enclosingNavigation = navigation;
    motion = _motion;
    leftLineSensor = _leftLineSensor;
    rightLineSensor = _rightLineSensor;

    firstContact = true;
    trackFollowerState = ON_TRACK;
    lineBearing = motion->GetBearing();

    leftContact = false;
    rightContact = false;

    leftInitialChangeTime = millis();
    rightInitialChangeTime = millis();
}

Navigation::TrackFollower::TrackFollower() : TrackFollower(new Navigation(), new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::TrackFollower::Tick()
{
    bool currentLineLeft;
    bool currentLineRight;

    // ON_TRACK case
    double theta;
    double R;

    // CONTACT case
    double newTheta;
    // double weightedTheta;
    double idealR;

    // Serial.print(state);
    // Serial.print(" ");
    // Serial.print(leftContact);
    // Serial.print(" ");
    // Serial.println(rightContact);

    if (enabled)
    {
        currentLineLeft = leftLineSensor->Line();
        currentLineRight = rightLineSensor->Line();

        if (leftContact == currentLineLeft)
        {
            leftInitialChangeTime = millis();
        }
        else if (millis() - leftInitialChangeTime > LINE_TIME_THRESHOLD)
        {
            leftContact = currentLineLeft;
            leftInitialChangeTime = millis();
        }

        // leftContact = currentLineLeft;
        // rightContact = currentLineRight;

        if (rightContact == currentLineRight)
        {
            rightInitialChangeTime = millis();
        }
        else if (millis() - rightInitialChangeTime > LINE_TIME_THRESHOLD)
        {
            rightContact = currentLineRight;
            rightInitialChangeTime = millis();
        }

        if (motion->GetTargetSpeed() != CRUISE_SPEED && trackFollowerState != LOST)
        {
            Serial.println("FORWARD!!");
            motion->SetSpeed(CRUISE_SPEED);
        }

        // If the angle from line is large, recalibrate
        if (leftContact && rightContact && fabs(motion->GetBearing() - lineBearing) > MAX_LINE_ANGLE_DISCREPANCY)
        {
            Serial.println("Recalibrating");
            enclosingNavigation->Calibrate((motion->GetBearing() > lineBearing ? LEFT : RIGHT));
        }

        switch (trackFollowerState)
        {
            case ON_TRACK:
                // While there is a discrepancy between the robot bearing and the
                // line bearing, the robot should turn to adjust.
                // Previous R from last contact should be maintained until straight.
                theta = motion->GetBearing() - lineBearing;
                R = motion->GetTargetTurnRadius();
                if (fabs(theta) < STRAIGHT_THETA_MARGIN)
                {
                    theta = 0;
                }

                // Stop turning if straight
                if (theta == 0)
                {
                    if (R != 0)
                    {
                        Serial.print("Straightened out. New bearing: ");
                        Serial.println(motion->GetBearing() / PI * 180);
                        motion->SetTurnRadius(0);
                    }
                }
                else
                {
                    // If not turning or if, for some bizarre reason, turning the wrong way
                    if (R == 0 || R * theta < -1)
                    {
                        motion->SetTurnRadius((theta > 0 ? OFF_LINE_TURN_RADIUS : -OFF_LINE_TURN_RADIUS));
                    }
                }

                // On contact, start turning
                if (leftContact || rightContact)
                {
                    contactDirectionMultiplier = (leftContact ? 1 : -1);
                    // initialTheta = contactDirectionMultiplier * (SENSOR_SEPARATION - LINE_THICKNESS) / (2.0f * motion->GetDeltaY());
                    initalBearing = motion->GetBearing();

                    trackFollowerState = CONTACT;
                    Serial.print("Contact! Turn ");
                    Serial.println((leftContact ? "left" : "right"));
                    motion->SetTurnRadius(contactDirectionMultiplier * ON_LINE_TURN_RADIUS);
                    // motion->GetDeltaY();    // To start deltaY from this point
                }
                break;
            
            case CONTACT:
                // Once off the line again, use distance travelled for new guess at theta
                // and apply it to previous theta using weighted average. For contact tick,
                // weighting is entirely on this guess and nothing else; for all contact
                // after first, weight by distance travelled on line, with previous theta
                // holding constant weight.
                if (!(leftContact || rightContact))
                {
                    // newTheta = motion->GetDeltaY() / (2.0f * motion->GetTargetTurnRadius());

                    if (firstContact)
                    {
                        firstContact = false;
                        // weightedTheta = newTheta;
                        lineBearing = (initalBearing + motion->GetBearing()) / 2;
                    }
                    else
                    {
                        // weightedTheta = (initialTheta * INITIAL_THETA_WEIGHT + newTheta * DISTANCE_THETA_WEIGHT) / (INITIAL_THETA_WEIGHT + DISTANCE_THETA_WEIGHT);
                        // lineBearing = (lineBearing + initalBearing + motion->GetBearing()) / 3;
                        if (motion->GetDeltaY() > 30)
                        {
                            lineBearing = (lineBearing + initalBearing + motion->GetBearing()) / 3;
                        }
                    }
                    
                    // Choose R to align centrally with line, but not over typical turn radius off the line
                    newTheta = lineBearing - motion->GetBearing();
                    idealR = (SENSOR_SEPARATION - LINE_THICKNESS) / (newTheta * newTheta);
                    motion->SetTurnRadius(-contactDirectionMultiplier * (idealR < OFF_LINE_TURN_RADIUS ? idealR : OFF_LINE_TURN_RADIUS));

                    trackFollowerState = ON_TRACK;

                    Serial.println("Back on track.");
                    Serial.print("Line bearing: ");
                    Serial.print(lineBearing / PI * 180);
                    Serial.print("; Wheel-e bearing: ");
                    Serial.println(motion->GetBearing() / PI * 180);
                }
                break;

            default:
                break;
        }
    }
}

void Navigation::TrackFollower::Enable()
{
    enabled = true;
    
    firstContact = true;
    trackFollowerState = ON_TRACK;
    lineBearing = motion->GetBearing();
}

void Navigation::TrackFollower::Disable()
{
    enabled = false;
}