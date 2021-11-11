#define     ULTRASOUND_TRIG     1      // Digital I/O output pin
#define     ULTRASOUND_ECHO     2      // Digital I/O input pin

#define     TRIGGER_INPUT_PULSE_LENGTH      10      // Length of trigger pulse in microseconds
#define     DELAY_WAIT_LENGTH               60000   // Length of delay in microseconds
#define     TIMEOUT_LENGTH                  20000   // Length before sensor times out
#define     DISTANCE_CONVERSION_DIVISOR     5.8f    // To calibrate distance calculation. Change as needed


// https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
class UltrasoundSensor
{
    private:
        bool enabled;
        unsigned long distance;     // Distance in millimetres
        unsigned long t;            // Timer using microseconds (lasts ~5 hours)

        pin_size_t ultrasound_trig;
        pin_size_t ultrasound_echo;
    
    public:
        // You pass the out/in pins for the sensor
        UltrasoundSensor(pin_size_t, pin_size_t);
        
        // Call this in loop. It times itself and updates distance accordingly
        void Tick();
        
        void Enable();
        void Disable();
        unsigned long GetDistance();
};