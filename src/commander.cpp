#include "commander.h"
#include "bma020.h"

#include <Arduino.h>

bool Commander::reboot()
{
    ESP.restart();
    return true;
}

Commander::Commander ()
{}

Commander::command_t Commander::parse (String inp)
{
    command_t cmd;
    cmd.isValid = false;

    if (inp.length() > cmd_max_Size)
    {
        cmd.err = F ("No input sequence detected!");
        cmd.isValid = false;
        return cmd;
    }

    char sepChar = ':';
    char endChar = ';';
    int sepPos = inp.indexOf (sepChar);

    if (sepPos == -1)
    {
        cmd.err = F ("Seperator char ':' mising!");
        cmd.isValid = false;
        return cmd;
    }

    int endPos = inp.indexOf (endChar);

    if (endPos == -1)
    {
        cmd.err = F ("End char ';' mising!");
        cmd.isValid = false;
        return cmd;
    }

    if (endPos <= 2)
    {
        cmd.err = F ("Command is missing!");
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

    for (uint8_t ctr = sepPos + 1; ctr < endPos; ctr++ )
    {
        cmd.arg += inp[ctr];
    }

    cmd.isValid = true;

    return cmd;
}

bool Commander::execute (const command_t &cmd)
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
        ret = true;
        break;

    case cmd_start_acc:
        _started = cmd.arg == String ('1');
        return true;
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
    Serial.printf ("%-20s:%4d\n%-20s:%4d\n%-20s:%4d\n",
                   "Start ACC (Arg: 0/1)", commands::cmd_start_acc,
                   "Reset BMA", commands::cmd_reset_acc,
                   "Reboot", commands::cmd_reboot);
}
