#ifndef COMMANDER_H
#define COMMANDER_H

#include "sensor.h"
#include "sensortypes.h"

#include <ArduinoJson.hpp>

static const uint16_t cmd_max_Size = 300;
extern const uint16_t cmd_max_Size;

class Commander {
public:
    typedef struct {
        sensor::MSGTYPE messageType;
        String errMessage;
        bool success;
    } processState_t;

private:
    bool _started;

    bool reboot();
    bool setBMARange (sensor::BMA020RANGE);
    bool setBMABandWidth (sensor::BMA020BANDWIDTH bw);

public:
    processState_t process (const String &inp);
    bool started();
    void printHelp();

};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
    static Commander commander;
    extern Commander commander;
#endif

#endif // COMMANDER_H
