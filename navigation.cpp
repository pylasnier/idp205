#include <Wire.h>
#include "navigation.h"
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"

Navigation::Navigation(Motion *_motion, LineSensor *_leftLineSensor, LineSensor *_rightLineSensor/*, UltrasoundSensor _distanceSensor*/)
{
    //distanceSensor = _distanceSensor;

    motion = _motion;
    leftLineSensor = _leftLineSensor;
    rightLineSensor = _rightLineSensor;

    firstContact = true;
    navigationState = ON_TRACK;
    lineBearing = motion->GetBearing();
}

Navigation::Navigation() : Navigation(new Motion(), new LineSensor(), new LineSensor()) { }

void Navigation::Tick()
{
    bool leftContact = leftLineSensor->Line();
    bool rightContact = rightLineSensor->Line();

    // ON_TRACK case
    double theta;
    double R;

    // CONTACT case
    double thetaFromDistance;
    double weightedTheta;
    double idealR;

    // Serial.print(state);
    // Serial.print(" ");
    // Serial.print(leftContact);
    // Serial.print(" ");
    // Serial.println(rightContact);

    if (motion->GetTargetSpeed() != CRUISE_SPEED)
    {
        Serial.println("FORWARD!!");
        motion->SetSpeed(CRUISE_SPEED);
    }

    if (leftContact && rightContact)
    {
        navigationState = DOUBLE_CONTACT;
    }

    switch (navigationState)
    {
        case ON_TRACK:
            // While there is a discrepancy between the robot bearing and the
            // line bearing, the robot should turn to adjust.
            // Previous R from last contact should be maintained until straight.
            theta = motion->GetBearing() - lineBearing;
            R = motion->GetTurnRadius();
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
                    motion->SetTurnRadius((theta > 0 ? 1 : -1) * OFF_LINE_TURN_RADIUS);
                }
            }

            // On contact, make initial guess at theta and start turning
            if (leftContact || rightContact)
            {
                contactDirectionMultiplier = (leftContact ? 1 : -1);
                initialTheta = contactDirectionMultiplier * (SENSOR_SEPARATION - LINE_THICKNESS) / (2.0f * motion->GetDeltaY());
                initalBearing = motion->GetBearing();

                navigationState = CONTACT;
                Serial.print("Contact! Turn ");
                Serial.println((leftContact ? "left" : "right"));
                motion->SetTurnRadius(contactDirectionMultiplier * ON_LINE_TURN_RADIUS);
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
                thetaFromDistance = motion->GetDeltaY() / (2.0f * motion->GetTurnRadius());

                if (firstContact)
                {
                    firstContact = false;
                    weightedTheta = thetaFromDistance;
                    lineBearing = initalBearing + weightedTheta;
                }
                else
                {
                    weightedTheta = (initialTheta * INITIAL_THETA_WEIGHT + thetaFromDistance * DISTANCE_THETA_WEIGHT) / (INITIAL_THETA_WEIGHT + DISTANCE_THETA_WEIGHT);
                    lineBearing = (lineBearing + initalBearing + weightedTheta) / 2;
                }
                
                // Choose R to align centrally with line, but not over typical turn radius off the line
                idealR = (SENSOR_SEPARATION - LINE_THICKNESS) / (weightedTheta * weightedTheta);
                motion->SetTurnRadius(-contactDirectionMultiplier * (idealR > OFF_LINE_TURN_RADIUS ? OFF_LINE_TURN_RADIUS : idealR));

                navigationState = ON_TRACK;
                Serial.print("Back on track; angle discrepancy: ");
                Serial.print(fabs(thetaFromDistance) / PI * 180);
                Serial.println(" degrees");
                Serial.print("Line bearing: ");
                Serial.print(lineBearing / PI * 180);
                Serial.print("; Wheel-e bearing: ");
                Serial.println(motion->GetBearing() / PI * 180);
            }
            break;
        
        case DOUBLE_CONTACT:
            Serial.println("I'M STUCK (DON'T HUG ME I'M SCARED) I.E. DOUBLE CONTACT!!!");
            navigationState = LOST;
            break;
        
        case LOST:
            break;

        default:
            Serial.println("No navigation state");
            break;
    }
}