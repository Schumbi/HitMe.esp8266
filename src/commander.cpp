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

Commander:: processState_t Commander::process (const String &inp)
{
    using namespace ArduinoJson;
    
    processState_t result;
    result.messageType = sensor::MSGTYPE::ANSWER_MSG;
    
    StaticJsonDocument<cmd_max_Size> rootIn;
    auto parseErr = deserializeJson(rootIn, inp);
    if(parseErr)
    {
        result.messageType = sensor::MSGTYPE::PARSEERR;
        result.errMessage = F ("Parser");
        result.success = false;
        return result;        
    }

    // required
    int ty = rootIn[JKEY_type];

    if (static_cast<sensor::MSGTYPE> (ty) != sensor::MSGTYPE::REQUEST_MSG)
    {
        result.messageType = sensor::MSGTYPE::PARSEERR;
        result.errMessage = F ("Not a control package!");
        result.success = false;
        return result;
    }

    // not required
    int bandWidth = rootIn[JKEY_bandwidth];
    if (!setBMABandWidth (static_cast<BMA020BANDWIDTH> (bandWidth)))
    {
        result.messageType = sensor::MSGTYPE::PARSEERR;
        result.errMessage = F ("Could not set bandwidth!");
        result.success = false;
        return result;
    }

    int range =  rootIn[JKEY_range];
    if (!setBMARange (static_cast<BMA020RANGE> (range)))
    {
        result.messageType = sensor::MSGTYPE::PARSEERR;
        result.errMessage = F ("Could not set bandwidth!");
        result.success = false;
        return result;
    }

    _started = rootIn[JKEY_start];


    // command stuff
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
        result.messageType = sensor::MSGTYPE::PARSEERR;
        result.errMessage = F ("Command is unknown!");
        result.success = false;
        return result;
    }

    result.success = true;

    return result;
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
