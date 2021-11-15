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
unsigned long tick;

void setup()
{
    motion = Motion(Adafruit_MotorShield(), LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT);
    navigation = Navigation(motion, LineSensor(LEFT_LINE_SENSOR_PIN), LineSensor(RIGHT_LINE_SENSOR_PIN));
    Serial.begin(DEFAULT_BAUD_RATE);
    Serial.println("Starting!");

    tick = 0;
}

void loop()
{
    //Serial.println(tick);
    tick++;
    navigation.Tick();
    motion.Tick();
}

#endif