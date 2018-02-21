#ifndef BMA020_H
#define BMA020_H

#include <Arduino.h>
#include <Wire.h>
#include <functional>

const uint8_t BMA029ADDR = 0x38;
const uint16_t ACCBUFFLEN = 120;

typedef void (*isr_t) ();

class BMA020 : private TwoWire {
public:
    // 2g, 4g, 8g available
    enum BMA020RANGE
    {
        BMA020_RANGE_2G = 0x00, // 00b
        BMA020_RANGE_4G = 0x01, // 01b
        BMA020_RANGE_8G = 0x02  // 10b
    };
    // 25 - 1500 Hz available
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
    // acc data struct as received from BMA020 (bits are in right order but 2complements)
    struct raw_acc_t
    {
        // original from bma
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };
    // to decimal converted acc values
    struct dec_acc_t
    {
        // converted
        int16_t acc_x;
        int16_t acc_y;
        int16_t acc_z;
    };
    // acc data struct for data converted to g
    struct g_data_t
    {
        double_t acc_x;
        double_t acc_z;
        double_t acc_y;
    };

    uint8_t isrFlag;

private:
    // setup state
    bool _isSetup;
    // internal acc data struct
    raw_acc_t _raw_data;
    // device adress
    uint8_t _adr;
    // calculate raw int16_t acc values into g data
    double _gfactor;
    // calculate gfactor
    bool calculateGFactor();
    // ask for chip id (010b)
    uint8_t getChipId();
    // get internal data
    bool update_acc (raw_acc_t &data);
    // buffers for the interrupt data
    volatile uint8_t* isrData;
    //uint8_t isrDataCtr = 0;
    const uint16_t isrDataMax = ACCBUFFLEN;
    // isr callback "member"
    void _isr_callback();
public:
    volatile uint16_t posToWriteDataAt;
    volatile uint16_t ctrSaveToReadTo;
    uint16_t last;
    volatile uint8_t canUpdateIntCtr;

protected:
    // write data to register
    virtual void writeReg (BMA020REGISTER reg, uint8_t value);
    // read uint8_t from reg
    virtual uint8 readReg (BMA020REGISTER reg);

public:
    BMA020();
    BMA020 (uint8_t adr, uint8_t sda = SDA, uint8_t scl = SCL);
    // setup i2c
    bool setup (uint8_t sda = SDA, uint8_t scl = SCL, uint8_t adr = BMA029ADDR,
                BMA020RANGE range = BMA020_RANGE_8G,
                BMA020BANDWIDTH bandwidth = BMA020_BW_25HZ);
    // get status of bma020
    bool isOk();
    // acc range
    bool setRange (BMA020RANGE range = BMA020_RANGE_8G);
    BMA020RANGE getRange();
    // acc bandwidth ("polling rate")
    bool setBandwidth (BMA020BANDWIDTH bandwidth = BMA020_BW_25HZ);
    BMA020BANDWIDTH getBandWidth();
    // get acc data
    raw_acc_t &getRawData();
    // set interrupt to fire at new data
    bool setNewDataInterrupt (bool fire = true);
    bool getNewDataInterrupt();
    // return status as string
    virtual String getStatus();
    // setup interrupt
    bool setupInterruptNewDataMode (uint8_t interruptOutPin, bool enable);
    // returns the current interrupt write position - 1
    uint16_t getCtrSaveToReadTo();
    // get bufflen
    uint16_t getAccBuflen();
    // return some data from internal buffer
    bool getData (uint8_t* buf, uint16_t& size);

    //// conversions
    // get acc in dec data (converted from 2s complement)
    dec_acc_t getDecData (const raw_acc_t &raw) const;
    // get acc in g data
    g_data_t getGData (const raw_acc_t &raw) const;

};

#endif // BMA020_H
