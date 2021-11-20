#include "project_defines.h"

#ifdef NAVIGATIONTEST

#include <Wire.h>
#include "robot.h"
#include "navigation.h"
#include "motion.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Motion motion;
Navigation navigation;

void setup()
{
    Serial.begin(DEFAULT_BAUD_RATE);
    Serial.println("Starting!");
    
    motion = Motion(new Adafruit_MotorShield(), LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT, new WheelEncoder(), new WheelEncoder());
    motion.Begin();
    navigation = Navigation(&motion, new LineSensor(LEFT_LINE_SENSOR_PIN, NAVIGATION_LINE_THRESHOLD), new LineSensor(RIGHT_LINE_SENSOR_PIN, NAVIGATION_LINE_THRESHOLD));
}

void loop()
{
    navigation.Tick();
    motion.Tick();
}

#endif