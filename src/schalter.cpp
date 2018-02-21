#include "schalter.h"

#include <WiFiUdp.h>
#include <osapi.h>
#include <user_interface.h>

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

const uint16_t sendBufSize = 500;
uint8_t* sendBuf;

uint16_t maxAccBuffLen;
void setup()
{
    pinMode (BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    // Serial Stuff
    Serial.begin (9600);
    Serial.setDebugOutput (true);
    Serial.println ("Start...");
    delay (100);

    sendBuf = (uint8_t*)malloc (sendBufSize * sizeof (uint8_t));

    int ms = 100;
    // initialize BMA020
    Serial.println (acc.setup());
    delay (ms);
    digitalWrite (BUILTIN_LED, !digitalRead (LED_BUILTIN));
    acc.setBandwidth (BMA020::BMA020BANDWIDTH::BMA020_BW_25HZ);
    Serial.print ("Bandwidth: ");
    Serial.println (acc.getBandWidth());
    acc.setRange (r);
    Serial.print ("Range: ");
    Serial.println (acc.getRange());
    acc.setupInterruptNewDataMode (D5, false);

    while (acc.isOk() == false)
    {
        Serial.printf ("%s \r", acc.getStatus().c_str());
        delay (500);
    }

    Serial.printf ("%s \n", acc.getStatus().c_str());
    Serial.println ("Sensor ok\n");
    maxAccBuffLen = acc.getAccBuflen();
    //sendUdp (dest, port, String ("#### Geht los!!! ####") + '\n');

    setup_wifi();
    udp.begin (port);
    Serial.printf ("UDP port %d\n", udp.localPort());
    acc.setNewDataInterrupt (true);

    Serial.println (system_get_free_heap_size());

}

union
{
    uint32_t data1x32;
    uint16_t data2x16[2];
    uint8_t data4x8[4];
} conv;

void loop()
{
    conv.data1x32 = millis();
    sendBuf[0] = conv.data4x8[3];
    sendBuf[1] = conv.data4x8[2];
    sendBuf[2] = conv.data4x8[1];
    sendBuf[3] = conv.data4x8[0];

    uint16_t s = sendBufSize - 4;
    acc.getData (sendBuf + 4, s);

    udp.beginPacket (dest, port);
    udp.write (sendBuf, s + 4);
    auto ret = udp.endPacket();

    Serial.print (acc.posToWriteDataAt);
    Serial.print ('\n');

    if (ret == 0)
    {
        Serial.println (ret);
    }

    delay (5);
}
