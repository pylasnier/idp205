#include <Wire.h>
#include "navigation.h"
#include "motion.h"
#include "line_detection.h"
#include "ultrasound.h"

Navigation::Navigation(Motion _motion, LineSensor _leftLineSensor, LineSensor _rightLineSensor/*, UltrasoundSensor _distanceSensor*/)
{
    motion = _motion;
    leftLineSensor = _leftLineSensor;
    rightLineSensor = _rightLineSensor;
    //distanceSensor = _distanceSensor;

    firstContact = true;
    navigationState = ON_TRACK;
    lineBearing = motion.GetBearing();
}

Navigation::Navigation() : Navigation(Motion(), LineSensor(), LineSensor()/*, UltrasoundSensor()*/) { }

void Navigation::Tick()
{
    bool leftContact = leftLineSensor.Line();
    bool rightContact = rightLineSensor.Line();

    if (motion.GetTargetSpeed() != CRUISE_SPEED)
    {
        motion.SetSpeed(CRUISE_SPEED);
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
            double theta = motion.GetBearing() - lineBearing;
            double R = motion.GetTurnRadius();
            if (abs(theta) < STRAIGHT_THETA_MARGIN)
            {
                theta = 0;
            }

            // Stop turning if straight
            if (theta == 0)
            {
                if (R != 0)
                {
                    motion.SetTurnRadius(0);
                }
            }
            else
            {
                // If not turning or if, for some bizarre reason, turning the wrong way
                if (R == 0 || R * theta < -1)
                {
                    motion.SetTurnRadius((theta > 0 ? 1 : -1) * OFF_LINE_TURN_RADIUS);
                }
            }

            // On contact, make initial guess at theta and start turning
            if (leftContact || rightContact)
            {
                contactDirectionMultiplier = (leftContact ? 1 : -1);
                initialTheta = contactDirectionMultiplier * (SENSOR_SEPARATION - LINE_THICKNESS) / (2.0f * motion.GetDeltaY());
                initalBearing = motion.GetBearing();

                navigationState = CONTACT;
                motion.SetTurnRadius(contactDirectionMultiplier * ON_LINE_TURN_RADIUS);
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
                double thetaFromDistance;
                double weightedTheta;
                double idealR;

                thetaFromDistance = motion.GetDeltaY() / (2.0f * motion.GetTurnRadius());

                if (firstContact)
                {
                    firstContact = false;
                    lineBearing = initalBearing + thetaFromDistance;
                    weightedTheta = thetaFromDistance;
                }
                else
                {
                    weightedTheta = (initialTheta * INITIAL_THETA_WEIGHT + thetaFromDistance * DISTANCE_THETA_WEIGHT) / (INITIAL_THETA_WEIGHT + DISTANCE_THETA_WEIGHT);
                    lineBearing = (lineBearing + initalBearing + weightedTheta) / 2;
                }
                
                // Choose R to align centrally with line, but not over typical turn radius off the line
                idealR = (SENSOR_SEPARATION - LINE_THICKNESS) / (2.0f * weightedTheta * weightedTheta);
                motion.SetTurnRadius(-contactDirectionMultiplier * (idealR > OFF_LINE_TURN_RADIUS ? OFF_LINE_TURN_RADIUS : idealR));

                navigationState = ON_TRACK;
            }
            break;
        
        case DOUBLE_CONTACT:
            Serial.println("I'M STUCK (DON'T HUG ME I'M SCARED) I.E. DOUBLE CONTACT!!!");
            break;
    }
}