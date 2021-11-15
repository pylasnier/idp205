#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#define     LINE_SENSOR_THRESHOLD       100

//https://www.ttelectronics.com/TTElectronics/media/ProductFiles/Datasheets/OPB703-70_70A-70F-B-704.pdf
class LineSensor
{
    private:
        pin_size_t line_sensor_pin;

    public:
        LineSensor(pin_size_t);
        LineSensor();

        unsigned long Sensor_reading();
        bool Line();

        void SetSensorPin(pin_size_t);
};

#endif