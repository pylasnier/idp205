#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "robot.h"
#include "ultrasound.h"

enum configuration_t
{
  TEST_STARTER = 0,
  ULTRASOUND = 1,
  MOVING = 2
};

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

unsigned long t; //To measure time
bool ledOn;

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(4);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(3);

configuration_t Configuration = MOVING;     // Keep it like this to test the ultrasound; check pins in ultrasound.h file.

UltrasoundSensor mySensor = UltrasoundSensor(ULTRASOUND_TRIG, ULTRASOUND_ECHO);

void setup() {
  switch (Configuration)
  {
  case TEST_STARTER:
    // declare the ledPin as an OUTPUT:
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    ledOn = false;
    Serial.begin(9600);

    t = millis();

    AFSM.begin();
    //leftMotor->setSpeed(150);
    //leftMotor->run(FORWARD);
    break;

  case ULTRASOUND:
    Serial.begin(DEFAULT_BAUD_RATE);

    mySensor.Enable();
    t = micros();
    break;

  case MOVING:
    AFSM.begin();
    leftMotor->setSpeed(150);
    rightMotor->setSpeed(150);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    break;
  default: break;
  }
}

void loop() {
  switch (Configuration)
  {
    case TEST_STARTER:
      // read the value from the sensor:
      sensorValue = analogRead(sensorPin);
      //Serial.println(sensorValue);
      //Serial.println(millis() - t);
      
      if ((millis() - t) > 250 && !ledOn)
      {
        digitalWrite(LED_BUILTIN, HIGH);
        Serial.println("HIGH");
        ledOn = true;
      }
      else if ((millis() - t) > 500 && ledOn)
      {
        digitalWrite(LED_BUILTIN, LOW);
        Serial.println("LOW");
        ledOn = false;
        t = millis();
      }
      break;
    
    case ULTRASOUND:
      mySensor.Tick();

      Serial.println(mySensor.GetDistance());
      break;
    
    default: break;
  }
}
