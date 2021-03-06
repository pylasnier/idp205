#include "project_defines.h"

#ifdef FULLSPEEDAHEAD

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "wheelencoder.h"
#include "robot.h"
#include "motion.h"

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(4);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(3);

Motion motion = Motion(&AFSM, LEFT_MOTOR_PORT, RIGHT_MOTOR_PORT,
    new WheelEncoder(new LineSensor(LEFT_WHEEL_ENCODER_PIN)),
    new WheelEncoder(new LineSensor(RIGHT_WHEEL_ENCODER_PIN)),
    new Leds(AMBER_LED_OUTPUT_PIN, RED_LED_OUTPUT_PIN, GREEN_LED_OUTPUT_PIN));

LineSensor tester = LineSensor(LEFT_WHEEL_ENCODER_PIN);
WheelEncoder wheelEncoder = WheelEncoder(&tester);

unsigned long t;

void setup()
{
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial);
    Serial.println("starting");

    motion.Begin();
    // motion.SetSpeed(0.07);
    // motion.SetTurnRadius(-200);
    motion.SetPivotTurnRate(-0.8);

    // AFSM.begin();
    // leftMotor->setSpeed(255);
    // leftMotor->run(FORWARD);

    Serial.println("running");
    t = millis();
}

void loop()
{
    motion.Tick();
    // Serial.println("ticking");
    // Serial.print(tester.Line());
    // Serial.print(" (");
    // Serial.print(tester.Sensor_reading());
    // Serial.println(")");
    // Serial.println(motion.GetTargetSpeed());
    // Serial.println(motion.GetTrueSpeed());
    // if (millis() - t > 3000)
    // {
    //     t = millis();
    //     Serial.println(wheelEncoder.GetDeltaY());
    // }
}

#endif