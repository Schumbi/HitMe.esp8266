#ifndef SENSORTYPES_H
#define SENSORTYPES_H

#define JKEY_type "type"
#define JKEY_error "error"
#define JKEY_cmd "cmd"
#define JKEY_ret "ret"
#define JKEY_err "err"
#define JKEY_msg "msg"
#define JKEY_readable "readable"
#define JKEY_bandwidth "bandwidth"
#define JKEY_range "range"
#define JKEY_start "start"
#define JKEY_millis "millis"

namespace sensor {

enum SUCCESS
{
    OK = 0,
    NOK
};

enum MSGTYPE
{
    PARSEERR = -1,
    REQUEST = 0,
    ANSWER,
    STATUS
};

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
}

namespace commands {

// commands
enum ctl_commands
{
    cmd_nocommand = -1,
    cmd_start_acc = 0,
    cmd_reset_acc,
    cmd_reboot = 99,
};

}

#endif // SENSORTYPES_H
