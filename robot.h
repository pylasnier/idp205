#ifndef ROBOT_H
#define ROBOT_H

#define     ULONG_MAX           UINT32_MAX      // Specific to the Arduino; max size 32 bits
#define     PIN_NOT_SET         UINT8_MAX       // 255, indicating pin is not set

#define     DEFAULT_BAUD_RATE   9600    // For serial read/write

#define     LEFT_MOTOR_PORT         4
#define     RIGHT_MOTOR_PORT        3

#define     LEFT_LINE_SENSOR_PIN    2
#define     RIGHT_LINE_SENSOR_PIN   3
#define     LEFT_WHEEL_ENCODER_PIN  4
#define     RIGHT_WHEEL_ENCODER_PIN 5

#define     AMBER_LED_OUTPUT_PIN    13
#define     RED_LED_OUTPUT_PIN      12
#define     GREEN_LED_OUTPUT_PIN    11

#define     ULTRASOUND_TRIG_PIN     6      // Digital I/O output pin
#define     ULTRASOUND_ECHO_PIN     7      // Digital I/O input pin

#define     ALIGNMENT_SENSOR_PIN    16      // Equivalent to A2, but for digital operations

#define     PUSH_BUTTON_INPUT_PIN   8

#define     PINCER_SERVO_PIN         9
#define     ELEVATION_SERVO_PIN     10

#define     DUMMY_SEEKER_INPUT_PIN  A0
#define     IR_DETECTOR_INPUT_PIN   A1
#define     TSOP_INPUT_PIN          A3

enum dummy_t
{
    NONE = 0,
    BABY = 1,
    MAMA = 2,
    PAPA = 3
};

#endif