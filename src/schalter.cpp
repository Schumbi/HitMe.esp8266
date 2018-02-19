#include "schalter.h"

#include <WiFiUdp.h>
#include "bma020.h"

#include "../../wlan.hpp"

void setup();
void loop();

BMA020 acc;

uint8_t ctr_ra = BMA020::BMA020_RANGE_8G;
uint8_t ctr_bw = BMA020::BMA020_BW_1500HZ;

BMA020::BMA020RANGE r = BMA020::BMA020_RANGE_2G;

WiFiUDP udp;

const IPAddress dest = IPAddress (192, 168, 1, 5);
const uint16_t port = 10000;

bool sendUdp (IPAddress ip, uint16_t port, String data)
{
    udp.beginPacket (ip, port);
    udp.write (data.c_str());
    return udp.endPacket() == 1;
}

void setup_wifi()
{
    digitalWrite (BUILTIN_LED, LOW);
    delay (10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print ("Connecting to ");
    Serial.println (wifi_config::ssid);
    WiFi.hostname ("schlag_1");
    WiFi.mode (WiFiMode::WIFI_STA);
    WiFi.begin (wifi_config::ssid, wifi_config::ssid_password);

    while (WiFi.status() != WL_CONNECTED)
    {
        digitalWrite (BUILTIN_LED, LOW);
        delay (500);
        Serial.print (".");
        digitalWrite (BUILTIN_LED, HIGH);
    }

    Serial.println ("");
    Serial.println ("WiFi connected");
    Serial.println ("IP address: ");
    Serial.println (WiFi.localIP());
    digitalWrite (BUILTIN_LED, HIGH);
    delay (100);
}

void setup()
{
    pinMode (BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    // Serial Stuff
    Serial.begin (9600);
    Serial.setDebugOutput (false);
    Serial.println ("Start...");
    delay (100);
    setup_wifi();
    udp.begin (port);
    Serial.printf ("UDP port %d\n", udp.localPort());
    int ms = 100;
    // initialize BMA020
    Serial.println (acc.setup());
    delay (ms);
    digitalWrite (BUILTIN_LED, !digitalRead (LED_BUILTIN));
    Serial.println (acc.getStatus());
    acc.setBandwidth (BMA020::BMA020BANDWIDTH::BMA020_BW_25HZ);
    Serial.print ("Bandwidth: ");
    Serial.println (acc.getBandWidth());
    acc.setRange (r);
    Serial.print ("Range: ");
    Serial.println (acc.getRange());
    acc.setupInterruptNewDataMode (D5, false);
    acc.setNewDataInterrupt (true);
    //sendUdp (dest, port, String ("#### Geht los!!! ####") + '\n');
}

//uint8_t buf[255];
void loop()
{
    Serial.println (acc.isrDataCtr);
    delay (10);
    /*
        auto accData = acc.getDecData (acc.getRawData());
        union data16_t
        {
            uint16_t data;
            uint8_t parts[2];
        } conv16;

        union data32_t
        {
            uint32_t millis;
            uint8_t parts[4];
        } conv32;
        //    memcpy (buf, conv32.parts, 4);
        conv32.millis = millis();
        buf[0] = conv32.parts[0];
        buf[1] = conv32.parts[1];
        buf[2] = conv32.parts[2];
        buf[3] = conv32.parts[3];
        conv16.data = accData.acc_x;
        buf[4] = conv16.parts[0];
        buf[5] = conv16.parts[1];
        conv16.data = accData.acc_y;
        buf[6] = conv16.parts[0];
        buf[7] = conv16.parts[1];
        conv16.data = accData.acc_z;
        buf[8] = conv16.parts[0];
        buf[9] = conv16.parts[1];
        buf[10] = 0;
        buf[11] = 0;

    //    snprintf (buf, 254, "%10ld:%4d,%4d,%4d\n", millis(), accData.acc_x,
    //              accData.acc_y, accData.acc_z);
    //    String data (buf);

        udp.beginPacket (dest, port);
        udp.write (buf, 12);
        auto ret = udp.endPacket();

    //    auto ret = sendUdp (dest, port, data);
    //    Serial.println (data);

        if (ret == 0)
        {
            Serial.println (ret);
        }
        */
}
