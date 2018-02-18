#ifndef CONF_HPP
#define CONF_HPP

#include "../../pwd.hpp"
#include "../../wlan.hpp"

#define WEBNAME "schlag_1"

namespace net_dev
{

    extern const char* hostname;
    const char* hostname = WEBNAME;

    namespace update
    {

        extern const char* web_update_path;
        const char* web_update_path = WEBNAME ".local/firmware";

        extern const unsigned int update_port;
        const unsigned int update_port = 80u;

        extern const char* user;
        const char* user = UPDATE_USERNAME;

        extern const char* pass;
        const char* pass = UPDATE_PASSWORT;
    }
}
#endif
