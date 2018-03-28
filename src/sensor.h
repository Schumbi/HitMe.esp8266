#ifndef SCHALTER_H
#define SCHALTER_H

#include <Arduino.h>
#include <ESP8266WiFi.h>


const uint8_t ACCPIN = D3;

// list of used registers
enum BMA020REGISTER
{
    E_CHIPID = 0x00,
    E_VERSION = 0x01,
    E_DATA_LSBX = 0x02, // acc data x
    E_DATA_MSBX = 0x03, // acc data x
    E_DATA_LSBY = 0x04, // acc data y
    E_DATA_MSBY = 0x05, // acc data y
    E_DATA_LSBZ = 0x06, // acc data z
    E_DATA_MSBZ = 0x07, // acc data z
    E_CTRL_0A = 0x0a,
    E_SETUP_ACC = 0x14, // range bandwidth
    E_CONTROL_OP = 0x15 // new data interrupt
};


#endif // SCHALTER_H
