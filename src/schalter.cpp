#include "schalter.h"

#include <WiFiUdp.h>
#include <Arduino.h>
#include <Wire.h>
#include <osapi.h>
#include <user_interface.h>

#include "../../wlan.hpp"

void setup();
void loop();

const uint8_t BMA029ADDR = 0x38;
TwoWire wire;
WiFiUDP udp;

const IPAddress dest = IPAddress (192, 168, 1, 5);
const uint16_t port = 10000;

void setup_wifi()
{
    digitalWrite (BUILTIN_LED, LOW);
    delay (10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print ("Connecting to ");
    Serial.println (wifi_config::ssid);
    WiFi.hostname ("schlag_1");
    WiFi.mode (WiFiMode::WIFI_STA);
    WiFi.begin (wifi_config::ssid, wifi_config::ssid_password);

    while (WiFi.status() != WL_CONNECTED)
    {
        digitalWrite (BUILTIN_LED, LOW);
        delay (500);
        Serial.print (".");
        digitalWrite (BUILTIN_LED, HIGH);
    }

    Serial.println ("");
    Serial.println ("WiFi connected");
    Serial.println ("IP address: ");
    Serial.println (WiFi.localIP());
    digitalWrite (BUILTIN_LED, HIGH);
    delay (100);
}

union conv_t
{
    uint32_t data1x32;
    uint16_t data2x16[2];
    uint8_t data4x8[4];
} conv;

const uint16_t sendBufSize = 500;
uint8_t* sendBuf;
uint16_t maxAccBuffLen;

uint8_t writeReg (TwoWire &wire, uint8_t reg, uint8_t value)
{
    wire.beginTransmission (BMA029ADDR);
    wire.write (reg);
    wire.write (value);
    return wire.endTransmission();
}

uint8_t readReg (TwoWire &wire, uint8_t reg, uint8_t *buf, uint8_t &size)
{
    wire.beginTransmission (BMA029ADDR);
    wire.write (reg);
    wire.endTransmission();
    wire.requestFrom (BMA029ADDR, (uint8_t)size);

    uint8_t ctr = 0;

    while (wire.available() && (ctr < size))
    {
        buf[ctr] = wire.read();
        ctr++;
    }

    size = ctr;

    return size;
}

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

bool isBMAReadable (TwoWire & wire)
{
    uint8_t id = 0;
    uint8_t size = 1;
    readReg (wire, E_CHIPID, &id, size);
    return  id == 2;
}

// R14h bandwidth
constexpr uint8_t _BW0 = 0x0u;
constexpr uint8_t _BW1 = 0x1u;
constexpr uint8_t _BW2 = 0x2u;
constexpr uint8_t _DONTUSE0 = 0x5u;
constexpr uint8_t _DONTUSE1 = 0x6u;
constexpr uint8_t _DONTUSE2 = 0x7u;
constexpr uint8_t _bit_mask_Bandwidth = (1 << _BW0) | (1 << _BW1) | (1 << _BW2);
constexpr uint8_t att_mask = (uint8_t) ((1 << _DONTUSE0) | (1 << _DONTUSE1) |
                                        (1 << _DONTUSE2));

enum BMA020BANDWIDTH
{
    BMA020_BW_25HZ = 0x00,  // 000b (mean 23 Hz)
    BMA020_BW_50HZ = 0x01,  // 001b (mean 47 Hz)
    BMA020_BW_100HZ = 0x02, // 010b (mean 94 Hz)
    BMA020_BW_190HZ = 0x03, // 011b (mean 188 Hz)
    BMA020_BW_375HZ = 0x04, // 100b (mean 375 Hz)
    BMA020_BW_750HZ = 0x05, // 101b (mean 750 Hz)
    BMA020_BW_1500HZ = 0x06 // 110b (mean 1500 Hz)
};

BMA020BANDWIDTH getBandwidth (TwoWire &wire)
{
    uint8_t buf;
    uint8_t size = 1;
    readReg (wire, E_SETUP_ACC, &buf, size);
    return (BMA020BANDWIDTH) ((buf & _bit_mask_Bandwidth) >> _BW0);
}

BMA020BANDWIDTH setBandwidth (TwoWire& wire, BMA020BANDWIDTH bandwidth)
{
    Serial.printf ("BW: %d\n", bandwidth);
    uint8_t buf;
    uint8_t size = 1;
    auto reg_14h = readReg (wire, E_SETUP_ACC, &buf, size);
    // preserve highest bits
    uint8_t attention = reg_14h & att_mask;
    // 11100111b = 11111111b ^ 00011000b
    auto delMask =  ~_bit_mask_Bandwidth;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= ((uint8_t)bandwidth << _BW0);
    // restore highest bits
    delMask = ~att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;
    writeReg (wire, E_SETUP_ACC, reg_14h);
    delay (50);
    auto bw = getBandwidth (wire);
    Serial.printf ("BW: %d\n", bw);
    return bw;
}


enum BMA020RANGE
{
    BMA020_RANGE_2G = 0x00, // 00b
    BMA020_RANGE_4G = 0x01, // 01b
    BMA020_RANGE_8G = 0x02  // 10b
};

// R14h ranges
constexpr uint8_t _RANGE0 = 0x3u;
constexpr uint8_t _RANGE1 = 0x4u;
constexpr uint8_t _bit_mask_Range = (1 << _RANGE0) | (1 << _RANGE1);

BMA020RANGE getRange (TwoWire &wire)
{
    uint8_t reg_14h;
    uint8_t size = 1;
    readReg (wire, E_SETUP_ACC, &reg_14h, size);
    return static_cast<BMA020RANGE> ((reg_14h & _bit_mask_Range) >> _RANGE0);
}

BMA020RANGE setRange (TwoWire &wire, BMA020RANGE range)
{
    Serial.printf ("RG: %d\n", range);
    uint8_t reg_14h;
    uint8_t size = 1;
    readReg (wire, E_SETUP_ACC, &reg_14h, size);
    // preserve highest bits
    uint8_t attention = reg_14h & att_mask;
    // 11100111b = 11111111b ^ 00011000b
    auto delMask = ~_bit_mask_Range;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= range << _RANGE0;
    // restore highest bits
    delMask = ~att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;
    writeReg (wire, E_SETUP_ACC, reg_14h);
    auto rg = getRange (wire);
    Serial.printf ("RG: %d\n", rg);
    return rg;
}

constexpr uint8_t _ACC_Start_ADR = 0x02u;
constexpr uint8_t _ACC_uint8_t_Count = 4;

volatile uint16_t isrBufSize = 120;
volatile uint16_t isrBufCtr = 0;
volatile uint16_t isrBufLastAccessCtr = 0;
volatile uint8_t* isrBuf;
volatile uint8_t accessFlag = 1;

void ISR_newdata (void)
{
    // interrupt subroutine
    //wire.beginTransmission (BMA029ADDR);
    //wire.write (_ACC_Start_ADR);
    //wire.endTransmission();
    //wire.requestFrom (BMA029ADDR, _ACC_uint8_t_Count);
    uint8_t ctr = 0;

    conv_t t;
    t.data1x32 = millis();

    //wire.available() &&
    while (ctr < _ACC_uint8_t_Count)
    {
        if (isrBufCtr >= isrBufSize)
        {
            isrBufCtr = 0;
        }

        isrBuf[isrBufCtr] = t.data4x8[ctr];//wire.read();
        isrBufCtr++;
        ctr++;
    }

    if (accessFlag == 1)
    {
        isrBufLastAccessCtr = (isrBufCtr);
    }

}

void resetInterruptBMA (TwoWire &wire)
{
    uint8_t r15h = 0;
    uint8_t size = 1;
    readReg (wire, BMA020REGISTER::E_CTRL_0A, &r15h, size);
    r15h |=  (1 << 6);
    writeReg (wire, BMA020REGISTER::E_CTRL_0A, r15h);
}

// acc new data interrupt
constexpr uint8_t _Lateched_INT_BIT = 4u;
constexpr uint8_t _NewData_INT_BIT = 5u;
constexpr uint8_t _EnableAdv_INT_BIT = 6u;
constexpr uint8_t _SPI4_INT_BIT = 7u;
void enableNewDataInterrupt (TwoWire &wire, uint8_t pin, bool enable)
{
    uint8_t r15h = 0;
    uint8_t size = 1;
    readReg (wire, E_CONTROL_OP, &r15h, size);
    // disable any motion interrupt
    //r15h &= ~ (1 << _EnableAdv_INT_BIT);
    //r15h |= (1 << _SPI4_INT_BIT);

    if (enable)
    {
        pinMode (pin, INPUT);
        attachInterrupt (digitalPinToInterrupt (pin), ISR_newdata, RISING);
        r15h |= (1 << _NewData_INT_BIT);// | (1 << _Lateched_INT_BIT);
    }
    else
    {
        detachInterrupt (digitalPinToInterrupt (pin));
        r15h &= ~ ((1 << _NewData_INT_BIT) | (1 << _Lateched_INT_BIT));
    }

    writeReg (wire, E_CONTROL_OP, r15h);
    resetInterruptBMA (wire);
}

uint16_t getLastAccessCtr()
{
    accessFlag = 0;
    auto lastAccessCtr = isrBufLastAccessCtr;
    accessFlag = 1;
    return lastAccessCtr;
}

uint16_t dataCtr = 0;
uint16_t getAccData (uint8_t* accData, uint16_t& size)
{
    uint16_t lastAccessIsr = getLastAccessCtr();
    uint16_t sctr = 0;

    //Serial.printf ("dataCtr: %4d lastAccessIsr: %d \n", dataCtr, lastAccessIsr);

    while ( (dataCtr != lastAccessIsr) && (sctr < size) )
    {
        if (dataCtr >= isrBufSize)
        {
            dataCtr = 0;
        }

        accData[sctr] = isrBuf[dataCtr];

        dataCtr++;
        sctr ++;
    }

    size = sctr;
    return size;
}

void bmaSoftReset (TwoWire & wire)
{
    writeReg (wire, BMA020REGISTER::E_CTRL_0A, 1 << 1);
    delay (2);
}

void setup()
{
    pinMode (BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    // Serial Stuff
    Serial.begin (9600);
    Serial.setDebugOutput (true);
    Serial.println ("Start...");
    setup_wifi();
    delay (100);

    wire.begin (SDA, SCL);
    bmaSoftReset (wire);

    isrBuf = (uint8_t*)malloc (isrBufSize * sizeof (uint8_t));
    sendBuf = (uint8_t*)malloc (sendBufSize * sizeof (uint8_t));
    int ms = 100;

    // initialize BMA020
    while (!isBMAReadable (wire))
    {
        Serial.print ('#');
        delay (ms);
    }

    Serial.println ("BMA online!");
    delay (2 * ms);
    // configure
    setBandwidth (wire, BMA020BANDWIDTH::BMA020_BW_190HZ);
    Serial.println ("BW configured!");
    delay (ms);
    setRange (wire, BMA020RANGE::BMA020_RANGE_8G);
    Serial.println ("Range configured!");
    delay (ms);
    digitalWrite (BUILTIN_LED, !digitalRead (LED_BUILTIN));
    udp.begin (port);
    Serial.printf ("UDP port %d\n", udp.localPort());

    enableNewDataInterrupt (wire, D5, false);
    delay (ms);
    enableNewDataInterrupt (wire, D5, true);

    system_print_meminfo();
}

void loop()
{
    conv.data1x32 = millis();
    sendBuf[0] = conv.data4x8[3];
    sendBuf[1] = conv.data4x8[2];
    sendBuf[2] = conv.data4x8[1];
    sendBuf[3] = conv.data4x8[0];

    uint16_t s = sendBufSize - 4;
    getAccData (sendBuf + 4, s);

    udp.beginPacket (dest, port);
    udp.write (sendBuf, s + 4);
    auto ret = udp.endPacket();

    uint8_t a;
    uint8_t size = 1;
    readReg (wire, BMA020REGISTER::E_CTRL_0A, &a, size);
    Serial.printf ("%5s%5d HEAP:%6d 0a:%6d\r", " ", s + 4,
                   system_get_free_heap_size(), a);

    if (ret == 0)
    {
        Serial.println (ret);
    }

    delay (50);
}
