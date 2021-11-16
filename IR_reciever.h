
class IRReciever
{
    private:
        pin_size_t IR_reciever_pin;

    public:
        IRReciever(pin_size_t);
        IRReciever();

        unsigned long Reciever_Reading();

        void SetRecieverPin(pin_size_t);

};
