#ifndef CMD_SET_H
#define CMD_SET_H

namespace commands {

// commands
enum ctl_commands
{
    cmd_start_acc = 0,
    cmd_reset_acc,
    cmd_get_config,
    cmd_set_range,
    cmd_set_bandwidth,
    // commands with args
    cmd_reboot = 99,
};

enum ctl_success
{
    OK,
    NOK
};

}

#endif // CMD_SET_H
