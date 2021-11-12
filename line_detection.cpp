#include <Wire.h>

#include "robot.h"
#include "line_detection.h"

LineSensor::LineSensor(pin_size_t sensorpin)
{
    line_sensor_pin = sensorpin;
}

unsigned long LineSensor::Sensor_reading()
{
    return analogRead(line_sensor_pin);
}

bool LineSensor::Line()
{
    return analogRead(line_sensor_pin) > 100;
}