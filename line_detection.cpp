#include <Wire.h>

#include "robot.h"
#include "line_detection.h"

LineSensor::LineSensor(pin_size_t sensorpin)
{
    SetSensorPin(sensorpin);
}

LineSensor::LineSensor() : LineSensor(PIN_NOT_SET) { }

unsigned long LineSensor::Sensor_reading()
{
    return analogRead(line_sensor_pin);
}

bool LineSensor::Line()
{
    //Serial.println(analogRead(line_sensor_pin));
    return analogRead(line_sensor_pin) > LINE_SENSOR_THRESHOLD;
}

void LineSensor::SetSensorPin(pin_size_t _line_sensor_pin)
{
    // Analogue,  don't need to set as input
    line_sensor_pin = _line_sensor_pin;
}