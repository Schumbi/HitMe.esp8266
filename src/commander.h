#ifndef COMMANDER_H
#define COMMANDER_H

#include "cmd_set.h"

static const uint8_t cmd_max_Size = 128;
extern const uint8_t cmd_max_Size;

class Commander {

    bool reboot();
    bool _started;

public:

    struct command_t
    {
        bool isValid;
        commands::ctl_commands cmd;
        String arg = "";
        String ret = "";
        String err = "";
    };

    Commander();
    command_t parse (String inp);
    bool execute (const command_t &cmd);
    bool started();
    void printHelp();
};

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_TWOWIRE)
    static Commander commander;
    extern Commander commander;
#endif

#endif // COMMANDER_H
