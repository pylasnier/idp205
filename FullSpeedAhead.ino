#include "project_defines.h"

#ifdef FULLSPEEDAHEAD

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "wheelencoder.h"
#include "robot.h"

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(4);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(3);

LineSensor tester = LineSensor(A2, ENCODER_LINE_THRESHOLD);
WheelEncoder wheelEncoder = WheelEncoder(&tester);

void setup()
{
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial);
    Serial.println("starting");

    AFSM.begin();
    Serial.println("begun");
    leftMotor->setSpeed(255);
    rightMotor->setSpeed(255);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    Serial.println("running");
}

void loop()
{
    wheelEncoder.Tick();
    // Serial.print(tester.Line());
    // Serial.print(" (");
    // Serial.print(tester.Sensor_reading());
    // Serial.println(")");
    Serial.println(wheelEncoder.GetSpeed());
}

#endif