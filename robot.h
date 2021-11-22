#ifndef ROBOT_H
#define ROBOT_H

#define     ULONG_MAX           UINT32_MAX      // Specific to the Arduino; max size 32 bits
#define     PIN_NOT_SET         UINT8_MAX       // 255, indicating pin is not set

#define     DEFAULT_BAUD_RATE   9600    // For serial read/write

#define     LEFT_MOTOR_PORT         3
#define     RIGHT_MOTOR_PORT        4

#define     LEFT_LINE_SENSOR_PIN    A0
#define     RIGHT_LINE_SENSOR_PIN   A1
#define     LEFT_WHEEL_ENCODER_PIN  A2
#define     RIGHT_WHEEL_ENCODER_PIN A3

#endif