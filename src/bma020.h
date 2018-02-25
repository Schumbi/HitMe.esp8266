#ifndef BMA020_H
#define BMA020_H

#include <Wire.h>

static const uint8_t BMA029ADDR = 0x38;
extern const uint8_t BMA029ADDR;

class BMA020 : public TwoWire {
public:
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

    enum BMA020RANGE
    {
        BMA020_RANGE_2G = 0x00, // 00b
        BMA020_RANGE_4G = 0x01, // 01b
        BMA020_RANGE_8G = 0x02  // 10b
    };

    static const uint8_t accPacket = 2u + 2u + 2u;  // 2x + 2y + 2z = 6 byte

    BMA020();

    uint8_t writeReg (uint8_t reg, uint8_t value);
    uint8_t readReg (uint8_t reg, uint8_t *buf, uint8_t &size);
    void resetAcc();
    // bandwidth
    bool setBandwidth (BMA020BANDWIDTH bandwidth);
    BMA020BANDWIDTH getBandwidth ();
    bool setRange (BMA020RANGE range);
    BMA020RANGE getRange ();
    bool isBMAReadable ();
    bool tryFetchNewData (uint8_t* accBuf, uint16_t& curCount,
                          uint16_t bufSize);
    String getConfig();
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
    static BMA020 Bma020;
    extern BMA020 Bma020;
#endif

#endif // BMA020_H
