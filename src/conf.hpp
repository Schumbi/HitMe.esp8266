#ifndef CONF_HPP
#define CONF_HPP

// adapt wlan_example.conf and change line here or rename to wlan.conf
#include "../wlan.conf"

#define WEBNAME "sensor_1"

namespace net_dev {

extern const char* hostname;
const char* hostname = WEBNAME;

namespace update {

extern const char* web_update_path;
const char* web_update_path = WEBNAME ".local/firmware";


}
}
#endif
