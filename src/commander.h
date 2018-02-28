#ifndef COMMANDER_H
#define COMMANDER_H

#include "sensor.h"

#include "cmd_set.h"

static const uint8_t cmd_max_Size = 128;
extern const uint8_t cmd_max_Size;

class Commander {
public:

    struct command_t
    {
        bool isValid;
        commands::ctl_commands cmd;
        String arg = "";
        String ret = "";
        String err = "";
    };

private:
    bool _started;

    void validateCommandWithNumericArg (command_t &cmd);

    bool reboot();
    void getBMAConfig (command_t &cmd);
    void setBMARange (command_t &cmd);
    void setBMABandWidth (command_t &cmd);

public:
    Commander();
    command_t parse (String inp);
    bool execute (command_t &cmd);
    bool started();
    void printHelp();
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
    static Commander commander;
    extern Commander commander;
#endif

#endif // COMMANDER_H
