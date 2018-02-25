#include "commander.h"
#include "bma020.h"

#include <Arduino.h>

bool Commander::reboot()
{
    ESP.restart();
    return true;
}

void Commander::getBMAConfig (command_t &cmd)
{
    String config = Bma020.getConfig();

    cmd.isValid = config.length() > 0;

    if (cmd.isValid)
    {
        cmd.ret = config;
    }
    else
    {
        cmd.ret = "";
        cmd.err = F ("Error reading BMA020 config!");
    }
}


void Commander::validateCommandWithNumericArg (Commander::command_t &cmd)
{
    if (cmd.arg.length() == 0)
    {
        cmd.err = F ("Arg is empty!");
        cmd.isValid = false;
        return;
    }

    String arg = cmd.arg;
    bool isValid = true;

    for (size_t ctr = 0; ctr < arg.length(); ctr++)
    {
        isValid &= isDigit (arg[ctr]);
    }

    if (isValid == false)
    {
        cmd.err = F ("Arg is not a number!");
        cmd.isValid = false;
        return;
    }

    cmd.isValid = true;

}

void Commander::setBMARange (Commander::command_t &cmd)
{

    validateCommandWithNumericArg (cmd);
    bool isValid = cmd.isValid;

    if (!isValid)
    {
        return;
    }

    isValid = false;
    int range = atoi (cmd.arg.c_str());

    switch (range)
    {
    case BMA020::BMA020RANGE::BMA020_RANGE_2G:
    case BMA020::BMA020RANGE::BMA020_RANGE_4G:
    case BMA020::BMA020RANGE::BMA020_RANGE_8G:
        isValid = true;
        break;

    default:
        isValid = false;
    }

    if (isValid == false)
    {
        cmd.err = F ("Arg is not in range!");
        cmd.isValid = false;
        return;
    }

    if (Bma020.isBMAReadable() == false)
    {
        cmd.err = F ("BMA communication error!");
        cmd.isValid = false;
        return;
    }

    Bma020.setRange ((BMA020::BMA020RANGE)range);

    cmd.ret = String ("{\"range\":\"") +  Bma020.getRange() + "\"}";
    cmd.isValid = true;

}

void Commander::setBMABandWidth (Commander::command_t &cmd)
{
    validateCommandWithNumericArg (cmd);
    bool isValid = cmd.isValid;

    if (!isValid)
    {
        return;
    }

    isValid = false;
    int bw = atoi (cmd.arg.c_str());

    switch (bw)
    {
    case BMA020::BMA020BANDWIDTH::BMA020_BW_25HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_50HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_100HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_190HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_375HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_750HZ:
    case BMA020::BMA020BANDWIDTH::BMA020_BW_1500HZ:
        isValid = true;
        break;

    default:
        isValid = false;
    }

    if (isValid == false)
    {
        cmd.err = F ("Arg is not in range!");
        cmd.isValid = false;
        return;
    }

    if (Bma020.isBMAReadable() == false)
    {
        cmd.err = F ("BMA communication error!");
        cmd.isValid = false;
        return;
    }

    Bma020.setBandwidth ((BMA020::BMA020BANDWIDTH)bw);
    cmd.ret = String ("{\"bandwidth\":\"") +  Bma020.getBandwidth() + "\"}";
    cmd.isValid = true;
}

Commander::Commander ()
{}

Commander::command_t Commander::parse (String inp)
{
    command_t cmd;
    cmd.isValid = false;

    if (inp.length() > cmd_max_Size)
    {
        cmd.err = F ("Input sequence is too long!");
        cmd.isValid = false;
        return cmd;
    }

    const char startChar = '{';
    const char endChar = '}';

    if (inp[0] != startChar && inp[inp.length() - 1] != endChar)
    {
        cmd.err = F ("Not a vaild command! {<cmd>:<arg>}");
        cmd.isValid = false;
        return cmd;
    }

    inp.remove (inp.indexOf (endChar));
    inp.remove (inp.lastIndexOf (startChar), 1);

    char sepChar = ':';
    int sepPos = inp.indexOf (sepChar);

    if (sepPos == -1)
    {
        cmd.err = F ("Seperator char ':' mising!");
        cmd.isValid = false;
        return cmd;
    }


    if (inp.length() <= 2)
    {
        cmd.err = F ("Command is invalid!");
        cmd.isValid = false;
        return cmd;
    }

    String cmdS;

    for (uint8_t ctr = 0; ctr < sepPos; ctr++ )
    {
        char c = inp[ctr];

        if (isDigit (c))
        {
            cmdS += inp[ctr];
        }
    }

    if (cmdS.length() == 0)
    {
        cmd.err = F ("No command found! (Is empty)");
        cmd.isValid = false;
        return cmd;
    }

    int t = cmdS.toInt();

    bool isAValidCmd = false;

    switch (t)
    {
    case commands::cmd_start_acc:
    case commands::cmd_reset_acc:
    case commands::cmd_set_range:
    case commands::cmd_set_bandwidth:
    case commands::cmd_get_config:
    case commands::cmd_reboot:
        isAValidCmd = true;
        break;

    default:
        isAValidCmd = false;
    }

    if (isAValidCmd == false)
    {
        cmd.err = F ("Command is unknown!");
        cmd.isValid = false;
        return cmd;
    }

    cmd.cmd = (commands::ctl_commands)t;

    for (uint8_t ctr = sepPos + 1; ctr < inp.length(); ctr++ )
    {
        cmd.arg += inp[ctr];
    }

    cmd.isValid = true;

    return cmd;
}

bool Commander::execute (command_t &cmd)
{
    bool ret = false;
    using namespace  commands;

    switch (cmd.cmd)
    {
    case cmd_reboot:
        ret = reboot();
        break;

    case cmd_reset_acc:
        Bma020.resetAcc();
        return  cmd.isValid;
        break;

    case cmd_start_acc:
        _started = cmd.arg == String ('1');
        return cmd.isValid;
        break;

    case cmd_set_range:
        setBMARange (cmd);
        return cmd.isValid;
        break;

    case cmd_set_bandwidth:
        setBMABandWidth (cmd);
        return  cmd.isValid;
        break;

    case cmd_get_config:
        getBMAConfig (cmd);
        return cmd.isValid;
        break;

    default:
        ret = false;
    }

    return ret;
}

bool Commander::started()
{
    return _started;
}

void Commander::printHelp()
{
    Serial.printf ("%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n",
                   "Start ACC (Arg: 0/1)", commands::cmd_start_acc,
                   "Reset BMA", commands::cmd_reset_acc,
                   "Print BMA config", commands::cmd_get_config,
                   "Set BMA range (Arg:0-2)", commands::cmd_set_range,
                   "Set BMA bandwidth (Arg:0-6)", commands::cmd_set_bandwidth,
                   "Reboot", commands::cmd_reboot);
}
