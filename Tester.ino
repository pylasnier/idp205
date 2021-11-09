/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13 through 220 ohm resistor
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  https://www.arduino.cc/en/Tutorial/BuiltInExamples/AnalogInput
*/

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

unsigned long t; //To measure time
bool ledOn;

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *pMotor = AFSM.getMotor(4);

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  ledOn = false;
  Serial.begin(9600);

  t = millis();

  AFSM.begin();
  //pMotor->setSpeed(150);
  //pMotor->run(FORWARD);
}

void loop() {
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
}
