#include <Wire.h>

#include "robot.h"
#include "IR_reciever.h"

IRReciever::IRReciever(pin_size_t recieverpin)
{
    SetRecieverPin(recieverpin);
}

IRReciever::IRReciever() : IRReciever(PIN_NOT_SET) { }

unsigned long IRReciever::Reciever_Reading()
{
    //Serial.println(analogRead(IR_reciever_pin));
    return analogRead(IR_reciever_pin);

}

void IRReciever::SetRecieverPin(pin_size_t _reciever_pin)
{
    IR_reciever_pin = _reciever_pin;
}
