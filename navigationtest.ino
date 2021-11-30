#include "project_defines.h"

#ifdef NAVIGATIONTEST

#include <Wire.h>
#include "robot.h"
#include "navigation.h"
#include "motion.h"
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Motion *motion;
Navigation *navigation;

void setup()
{
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial);
    Serial.println("Starting!");
    
    motion = new Motion(new Adafruit_MotorShield(), LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT,
        new WheelEncoder(new LineSensor(LEFT_WHEEL_ENCODER_PIN)),
        new WheelEncoder(new LineSensor(RIGHT_WHEEL_ENCODER_PIN)),
        new Leds());
    motion->Begin();
    navigation = new Navigation(motion,
        new LineSensor(LEFT_LINE_SENSOR_PIN), new LineSensor(RIGHT_LINE_SENSOR_PIN));
}

void loop()
{
    motion->Tick();
    navigation->Tick();
}

#endif