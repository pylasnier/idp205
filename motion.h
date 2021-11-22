#ifndef MOTION_H
#define MOTION_H

#define     WHEEL_SEPARATION            185     // in mm
#define     VELOCITY_TO_MOTOR_SPEED     60      // Temporary, while we don't have wheel encoders. Calibrate appropriately. In mm/s

#define     MOTOR_UPDATE_PERIOD         100     // ms
#define     MAX_MOTOR_VALUE             4000    // Just a bit under max uint12 value of 4095

#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include "wheelencoder.h"

class Motion
{
    private:
        Adafruit_MotorShield *AFSM;
        Adafruit_DCMotor *leftMotor;
        Adafruit_DCMotor *rightMotor;

        WheelEncoder *leftWheelEncoder;
        WheelEncoder *rightWheelEncoder;

        double targetSpeed;
        double targetTurnRadius;    // R
        double averageTurnRadius;
        double bearing;

        unsigned long t;        // Milliseconds, for integration
        unsigned long update;   // Milliseconds, for time between motor updates

    public:
        Motion(Adafruit_MotorShield *, uint8_t, uint8_t, WheelEncoder *, WheelEncoder *);
        Motion(uint8_t, uint8_t);
        Motion();

        void Begin();

        double GetDistance();
        double GetDeltaY();     // This functions returns distance travelled since it was last called

        double GetTargetSpeed();
        double GetTrueSpeed();
        void SetSpeed(double);

        double GetBearing();
        
        // NOTE for 0 turning, infinite R cannot be stored so we just use 0
        double GetTargetTurnRadius();
        double GetTrueTurnRadius();
        void SetTurnRadius(double);

        void Tick();
};

#endif