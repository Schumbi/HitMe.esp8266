#ifndef CMD_SET_H
#define CMD_SET_H

#include "sensor.h"

namespace commands {

// commands
enum ctl_commands
{
    cmd_start_acc = 0,
    cmd_reset_acc,
    // commands with args
    cmd_reboot = 99,
};

}

#endif // CMD_SET_H
