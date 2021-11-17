// IMPORTANT
// The header included below sets definitions, the existence of which determine
// whether or not this current file is included in compilation.
// In this case, if TESTER is defined, this file will be included.
// If not, it won't be. Most of the time, you just change the
// definition in the header to the name of the .ino file you want
// to use, and make sure to use an #ifdef #endif clause around the code.
// Do remember also to change the sketch file in the bottom right.
#include "project_defines.h"

#ifdef TESTER

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include "robot.h"
#include "ultrasound.h"
#include "line_detection.h"
#include "IR_reciever.h"

enum configuration_t
{
  TEST_STARTER = 0,
  ULTRASOUND = 1,
  MOVING = 2,
  LINE_SENSOR = 3,
  IR_RECIEVER = 4,
  SAY_SOMETHING = 5
};

int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor
const int num_recorded = 200;     // number of readings to be recorded from the reciever
int IR_log [num_recorded];        // log of previous readings
int count = 0;                        // location in log
int sum;

unsigned long t; //To measure time
bool ledOn;

Adafruit_MotorShield AFSM = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFSM.getMotor(3);
Adafruit_DCMotor *rightMotor = AFSM.getMotor(4);

configuration_t Configuration = IR_RECIEVER;     // SET THIS TO MODE YOU WANT TO TEST

UltrasoundSensor mySensor = UltrasoundSensor(ULTRASOUND_TRIG, ULTRASOUND_ECHO);

LineSensor line1 = LineSensor(A0);
LineSensor line2 = LineSensor(A1);

IRReciever reciever1 = IRReciever(A0);

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
    leftMotor->setSpeed(150);
    leftMotor->run(FORWARD);
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

    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
    ledOn = false;
    break;
  
  case LINE_SENSOR:
    Serial.begin(DEFAULT_BAUD_RATE);
    Serial.println("Line sensor test");

    // Just for Ben x
    AFSM.begin();
    leftMotor->setSpeed(255);
    //leftMotor->run(FORWARD);

    // declare the ledPin as an OUTPUT:
    pinMode(12, OUTPUT);
    digitalWrite(12, LOW);
    ledOn = false;
    break;
  
  case IR_RECIEVER:
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
    break;

  case SAY_SOMETHING:
    Serial.begin(DEFAULT_BAUD_RATE);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
    }
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
        digitalWrite(LED_BUILTIN, HIGH);
        //Serial.println("HIGH");
        ledOn = true;
      }
      else if ((millis() - t) > 500 && ledOn)
      {
        digitalWrite(LED_BUILTIN, LOW);
        //Serial.println("LOW");
        ledOn = false;
        t = millis();
      }
      break;
    
    case ULTRASOUND:
      mySensor.Tick();

      Serial.println(mySensor.GetDistance());
      break;
    
    case MOVING:
      if (line1.Line())
      {
        rightMotor->setSpeed(1);

        digitalWrite(12, HIGH);
        //Serial.println("HIGH");
        ledOn = true;
      }
      else
      {
        rightMotor->setSpeed(150);

        digitalWrite(12, LOW);
        //Serial.println("LOW");
        ledOn = false;
      }
      break;

    case LINE_SENSOR:
      Serial.print(line1.Sensor_reading());
      Serial.print(" - ");
      Serial.println(line2.Sensor_reading());

      if (line1.Line())
      {
        digitalWrite(12, HIGH);
        //Serial.println("HIGH");
        ledOn = true;
      }
      else
      {
        digitalWrite(12, LOW);
        //Serial.println("LOW");
        ledOn = false;
      }
      
      break;
    
    case IR_RECIEVER:
      //Serial.println("In here");
      //Serial.println(reciever1.Reciever_Reading());
      IR_log[count] = reciever1.Reciever_Reading();
      count += 1;
      if (count == num_recorded)
      {
        count = 0;
      }
      if (count % (num_recorded - 1) == 0)
      {
        sum = 0;
        for (int i = 0; i < num_recorded; i++)
        {
          sum += IR_log[i];
        }
        Serial.println(sum);
      }
      break;
    
    case SAY_SOMETHING:
      Serial.println("Hello");
      break;
    
    default: break;
  }
}

#endif