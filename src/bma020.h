#ifndef BMA020_H
#define BMA020_H

#include <Wire.h>

#include "sensortypes.h"

static const uint8_t BMA029ADDR = 0x38;
extern const uint8_t BMA029ADDR;

using namespace sensor;

class BMA020 : public TwoWire {
public:

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
