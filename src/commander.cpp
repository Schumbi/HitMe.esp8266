#include "commander.h"
#include "bma020.h"

#include <sensortypes.h>

#include <Arduino.h>

bool Commander::reboot()
{
    ESP.restart();
    return true;
}

bool Commander::setBMARange (sensor::BMA020RANGE range)
{
    Bma020.setRange (range);
    return Bma020.getRange() == range;
}

bool Commander::setBMABandWidth (sensor::BMA020BANDWIDTH bw)
{
    Bma020.setBandwidth (bw);
    return Bma020.getBandwidth() == bw;
}

void Commander::process (const String &inp, ArduinoJson::JsonObject& rootOut)
{
    using namespace ArduinoJson;
    StaticJsonBuffer<cmd_max_Size> jsonBufferIn;
    JsonObject& rootIn = jsonBufferIn.parseObject (inp);

    rootOut[JKEY_type] = sensor::MSGTYPE::ANSWER;

    if (inp.length() > cmd_max_Size)
    {
        rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
        rootOut[JKEY_err] = F ("Input sequence is too long!");
        return;
    }

    String jparseerror (F ("Could not parse json!"));

    if (!rootIn.success())
    {
        rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
        rootOut[JKEY_err] = jparseerror;
        return;
    }

    JsonVariant var = rootIn[JKEY_type];

    if (!var.success() || (!var.is<int>()))
    {
        rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
        rootOut[JKEY_err] = jparseerror;
        return;
    }

    // required
    int ty = rootIn[JKEY_type];

    if (static_cast<sensor::MSGTYPE> (ty) != sensor::MSGTYPE::REQUEST)
    {
        rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
        rootOut[JKEY_err] = F ("Not a control package!");
        return;
    }

    // not required
    var = rootIn[JKEY_bandwidth];

    if (var.success() && (var.is<int>()))
    {
        if (!setBMABandWidth (static_cast<BMA020BANDWIDTH> (var.as<int>())))
        {
            rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
            rootOut[JKEY_err] = F ("Could not set bandwidth!");
            return;
        }
    }

    var = rootIn[JKEY_range];

    if (var.success() && var.is<int>() )
    {
        if (!setBMARange (static_cast<BMA020RANGE> (var.as<int>())))
        {
            rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
            rootOut[JKEY_err] = F ("Could not set bandwidth!");
            return;
        }
    }

    var = rootIn[JKEY_start];

    if (var.success() && var.is<bool>() )
    {
        _started = var.as<bool>();
    }

    // command stuff
    var = rootIn[JKEY_cmd];

    if (var.success() && var.is<int>())
    {
        ty =  var.as<int>();

        switch (ty)
        {
        case commands::cmd_nocommand:
            break;

        case commands::cmd_reset_acc:
            Bma020.resetAcc();
            break;

        case commands::cmd_reboot:
            reboot();
            break;

        default:
            rootOut[JKEY_type] = sensor::MSGTYPE::PARSEERR;
            rootOut[JKEY_err] = F ("Command is unknown!");
            return;
        }
    }

    rootOut[JKEY_ret] = true;
}

bool Commander::started()
{
    return _started;
}

void Commander::printHelp()
{
    String ip = WiFi.localIP().toString();
    Serial.printf ("%-30s\n%-30s\n",
                   String ("Contrl Con: nc -n -u " + ip + " 10001").c_str(),
                   String ("Listen Con: nc -l -u -p 10001").c_str());
}
