#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "robot.h"
#include "ultrasound.h"

enum configuration_t
{
  TEST_STARTER = 0,
  ULTRASOUND = 1
};

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

unsigned long t; //To measure time
bool ledOn;

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *pMotor = AFSM.getMotor(4);

configuration_t Configuration = ULTRASOUND;     // Keep it like this to test the ultrasound; check pins in ultrasound.h file.

UltrasoundSensor mySensor = UltrasoundSensor(ULTRASOUND_OUT, ULTRASOUND_IN);

void setup() {
  switch (Configuration)
  {
  case TEST_STARTER:
    // declare the ledPin as an OUTPUT:
    pinMode(ledPin, OUTPUT);
    digitalWrite(ledPin, LOW);
    ledOn = false;
    Serial.begin(9600);

    t = millis();

    AFSM.begin();
    //pMotor->setSpeed(150);
    //pMotor->run(FORWARD);
    break;

  case ULTRASOUND:
    Serial.begin(DEFAULT_BAUD_RATE);

    mySensor.Enable();
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
      Serial.println(sensorValue);
      //Serial.println(millis() - t);
      
      if ((millis() - t) > 250 && !ledOn)
      {
        digitalWrite(ledPin, HIGH);
        //Serial.println("HIGH");
        ledOn = true;
      }
      else if ((millis() - t) > 500 && ledOn)
      {
        digitalWrite(ledPin, LOW);
        //Serial.println("LOW");
        ledOn = false;
        t = millis();
      }
      break;
    
    case ULTRASOUND:
      mySensor.Tick();

      Serial.print(mySensor.GetDistance());
      Serial.print(" mm\n");
      break;
    
    default: break;
  }
}
