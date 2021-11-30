#include "project_defines.h"

#ifdef MAIN

#include "robot.h"
#include "motion.h"
#include "navigation.h"
#include "leds.h"
#include "line_detection.h"
#include "ultrasound.h"
#include "controller.h"

Leds *leds;
Motion *motion;
Navigation *navigation;
Controller *controller;

bool start = false;

void setup()
{
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial);
    Serial.println("Starting");

    pinMode(PUSH_BUTTON_INPUT_PIN, INPUT);

    leds = new Leds(AMBER_LED_OUTPUT_PIN, RED_LED_OUTPUT_PIN, GREEN_LED_OUTPUT_PIN);
    motion = new Motion(new Adafruit_MotorShield(), LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT,
        new WheelEncoder(new LineSensor(LEFT_WHEEL_ENCODER_PIN)),
        new WheelEncoder(new LineSensor(RIGHT_WHEEL_ENCODER_PIN)),
        leds);
    motion->Begin();
    
    navigation = new Navigation(motion,
        new LineSensor(LEFT_LINE_SENSOR_PIN),
        new LineSensor(RIGHT_LINE_SENSOR_PIN));

    controller = new Controller(motion, navigation,
        new UltrasoundSensor(ULTRASOUND_TRIG_PIN, ULTRASOUND_ECHO_PIN), leds);

    delay(1000);
}

void loop()
{
    controller->Tick();
    if (digitalRead(PUSH_BUTTON_INPUT_PIN) == HIGH)
    {
        controller->Start();
        Serial.println("BUTTON");
    }
}

#endif