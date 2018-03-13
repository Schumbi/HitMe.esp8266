#include "commander.h"
#include "bma020.h"

#include <Arduino.h>
#include <ArduinoJson.hpp>

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
    case sensor::BMA020RANGE::BMA020_RANGE_2G:
    case sensor::BMA020RANGE::BMA020_RANGE_4G:
    case sensor::BMA020RANGE::BMA020_RANGE_8G:
        isValid = true;
        break;

    default:
        cmd.err = F ("Arg is not in range!");
        isValid = false;
        return;
    }

    if (Bma020.isBMAReadable() == false)
    {
        cmd.err = F ("BMA communication error!");
        cmd.isValid = false;
        return;
    }

    Bma020.setRange ((sensor::BMA020RANGE)range);

    cmd.ret = String (Bma020.getRange());
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
    case sensor::BMA020BANDWIDTH::BMA020_BW_25HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_50HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_100HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_190HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_375HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_750HZ:
    case sensor::BMA020BANDWIDTH::BMA020_BW_1500HZ:
        isValid = true;
        break;

    default:
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

    Bma020.setBandwidth ((sensor::BMA020BANDWIDTH)bw);
    cmd.ret = String (Bma020.getBandwidth());
    cmd.isValid = true;
}

Commander::Commander ()
{}

Commander::command_t Commander::parse (String inp)
{
    command_t cmd;
    cmd.isValid = false;
    cmd.arg = "";
    cmd.err = "";
    cmd.msg = "";
    cmd.ret = "";

    if (inp.length() > cmd_max_Size)
    {
        cmd.err = F ("Input sequence is too long!");
        cmd.isValid = false;
        return cmd;
    }

    using namespace ArduinoJson;
    StaticJsonBuffer<cmd_max_Size> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject (inp);

    root.printTo (Serial);
    Serial.println();

    String jparseerror (F ("Could not parse json!"));

    if (!root.success())
    {
        cmd.err = jparseerror;
        cmd.isValid = false;
        return cmd;
    }

    JsonVariant var = root["type"];

    if (!var.success() || (!var.is<int>()))
    {
        cmd.err = jparseerror;
        cmd.isValid = false;
        return cmd;
    }

    int ty = root["type"];

    if (static_cast<sensor::MSGTYPE> (ty) != sensor::MSGTYPE::REQUEST)
    {
        cmd.err =  F ("Not a control package!");
        cmd.isValid = false;
        return cmd;
    }

    var = root["cmd"];

    if ((!var.success()) || (!var.is<int>()))
    {
        cmd.err =  jparseerror;
        cmd.isValid = false;
        return cmd;
    }

    int t = var.as<int>();

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

    var = root["args"];

    if (var.success())
    {
        cmd.arg = String (var.as<char*>());
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
        return cmd.isValid;
        break;

    case cmd_start_acc:
        startStopMeasure (cmd);
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

void Commander::startStopMeasure (command_t &cmd)
{
    int arg = atoi (cmd.arg.c_str());
    cmd.isValid = false;

    switch (arg)
    {
    case 0:
        _started = false;
        cmd.isValid = true;
        break;

    case 1:
        _started = true;
        cmd.isValid = true;
        break;

    default:
        _started = false;
        cmd.err = F ("Could not parse arguments (0 -> off; 1 -> on)!");
    }
}

bool Commander::started()
{
    return _started;
}

void Commander::printHelp()
{
    String ip = WiFi.localIP().toString();
    Serial.printf ("%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n"
                   "%-30s:%4d\n%-30s:%4d\n%-30s:%4d\n%s\n%s\n",
                   "Start ACC (Arg: 0/1)", commands::cmd_start_acc,
                   "Reset BMA", commands::cmd_reset_acc,
                   "Print BMA config", commands::cmd_get_config,
                   "Set BMA range (Arg:0-2)", commands::cmd_set_range,
                   "Set BMA bandwidth (Arg:0-6)", commands::cmd_set_bandwidth,
                   "Reboot", commands::cmd_reboot,
                   String ("Contrl Con: nc -n -u " + ip + " 10001").c_str(),
                   String ("Listen Con: nc -l -u -p 10001").c_str());
}
