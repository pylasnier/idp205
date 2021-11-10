#define     ULTRASOUND_OUT      14      // Digital I/O output pin
#define     ULTRASOUND_IN       13      // Digital I/O input pin

#define     TRIGGER_INPUT_PULSE_LENGTH      10      // Length of trigger pulse in microseconds
#define     DELAY_WAIT_LENGTH               60000   // Length of delay in microseconds


enum status_t
{
    RESET = 0,              // Ready to start again
    TRIGGER_INPUT = 1,      // Sending pulse to trigger input
    AWAITING_RESPONSE = 2,  // Wait until response starts
    TIMING_RESPONSE = 3,    // Measure response time
    DELAY = 4               // Wait for a while before starting again
};

// https://cdn.sparkfun.com/datasheets/Sensors/Proximity/HCSR04.pdf
class UltrasoundSensor
{
    private:
        bool enabled;
        unsigned long distance;     // Distance in millimetres

        unsigned long t;            // Timer using microseconds (lasts ~5 hours)
        status_t status;

        pin_size_t ultrasound_out;
        pin_size_t ultrasound_in;
    
    public:
        // You pass the out/in pins for the sensor
        UltrasoundSensor(pin_size_t, pin_size_t);
        
        // Call this in loop. It times itself and updates distance accordingly
        void Tick();
        
        void Enable();
        void Disable();
        unsigned long GetDistance();
};