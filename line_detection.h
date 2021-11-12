
//https://www.ttelectronics.com/TTElectronics/media/ProductFiles/Datasheets/OPB703-70_70A-70F-B-704.pdf
class LineSensor
{
    public:
        LineSensor(pin_size_t);

        unsigned long Sensor_reading();
        bool Line(); 
        pin_size_t line_sensor_pin;
};
