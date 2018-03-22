#include "sensor.h"

#include "bma020.h"
#include "commander.h"

#include <WiFiUdp.h>
#include <Arduino.h>

#include <ArduinoJson.hpp>

#include "./conf.hpp"

void setup();
void loop();

WiFiUDP udpData;
WiFiUDP udpCmd;

const IPAddress dest = IPAddress (192, 168, 1, 5);
const uint16_t udpDataPort = 10000;
const uint16_t udpCmdPort = 10001;

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

    Serial.println ();
    Serial.println ("WiFi connected");
    Serial.println ("IP address: ");
    Serial.println (WiFi.localIP());
    digitalWrite (BUILTIN_LED, HIGH);
    delay (100);
}

union conv_t
{
    uint32_t data1x32;
    uint16_t data2x16[2];
    uint8_t data4x8[4];
} conv;

const uint8_t sizeOfTimeStamp = sizeof (uint32_t); // byte
const uint8_t sizeOfPacketId = sizeof (uint32_t); // byte

const uint8_t bufferedAccData = 30;
const uint16_t accDataBufSize = BMA020::accPacket * bufferedAccData;
// start and end of data block, packetid and acc data buffer size
const uint16_t sendBufSize = sizeOfTimeStamp + sizeOfTimeStamp +
                             sizeOfPacketId + accDataBufSize;
uint8_t* sendBuf;
uint8_t* accDataBuf;

void setup()
{
    pinMode (BUILTIN_LED, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    // Serial Stuff
    Serial.begin (9600);
    Serial.setDebugOutput (true);
    Serial.println ("Start...");
    setup_wifi();
    delay (100);

    // pin to BMA
    pinMode (ACCPIN, OUTPUT);
    digitalWrite (ACCPIN, HIGH);
    Bma020.begin (SDA, SCL);

    accDataBuf = (uint8_t*)malloc (accDataBufSize * sizeof (uint8_t));
    sendBuf = (uint8_t*)malloc (sendBufSize * sizeof (uint8_t));
    int ms = 100;

    // initialize BMA020
    while (!Bma020.isBMAReadable ())
    {
        Serial.print ("Restarting BMA -> \t");
        Bma020.resetAcc();
        delay (5 * ms);
    }

    Serial.println ("BMA online!");
    delay (2 * ms);

    // configure
    bool suc = Bma020.setBandwidth (sensor::BMA020BANDWIDTH::BMA020_BW_25HZ);
    Serial.printf ("BW configured: %3s\t", suc ? "OK" : "NOK");
    suc = Bma020.setRange (sensor::BMA020RANGE::BMA020_RANGE_8G);
    Serial.printf ("Range configured: %3s\n", suc ? "OK" : "NOK");
    // network stuff
    udpData.begin (udpDataPort);
    udpCmd.begin (udpCmdPort);
    Serial.printf ("UDP port data %d\t", udpData.localPort());
    Serial.printf ("UDP port cmd %d\n", udpCmd.localPort());

    commander.printHelp();

    Serial.printf ("Start looping!\n");
}

uint8_t statusCtr = 0;
uint16_t accDataBufCtr = 0;
uint32_t packetCtr = 0;
bool start = false;
uint32_t startTime = 0;
void loop()
{
    using namespace ArduinoJson;

    if (commander.started() == true)
    {
        if (startTime == 0)
        {
            // set start time, when it was reset
            startTime = micros();
        }

        if (Bma020.tryFetchNewData (accDataBuf, accDataBufCtr, accDataBufSize))
        {
            // data measuring completed for this block
            uint8_t cpyStart = 0;
            // cpy start time
            conv.data1x32 = startTime;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfTimeStamp);
            cpyStart += sizeof (startTime);
            // get time and set end time
            uint32_t endtime = micros();
            conv.data1x32 = endtime;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfTimeStamp);
            cpyStart += sizeof (endtime);
            // cpy packet id
            conv.data1x32 = packetCtr;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfPacketId);
            cpyStart += sizeof (packetCtr);
            // cpy acc data
            memcpy (sendBuf + cpyStart, accDataBuf, accDataBufSize);
            // send stuff
            packetCtr++;
            udpData.beginPacket (dest, udpDataPort);
            udpData.write (sendBuf, sendBufSize);
            auto ret = udpData.endPacket();

            if (ret == 0)
            {
                Serial.println ("Packet not send!");
            }

            // reset start time
            startTime = 0;
            // reset data ctr
            accDataBufCtr = 0;
        }
    }

    // check and parse ctrl packets
    uint16_t len = udpCmd.parsePacket();

    if (len)
    {
        String data = udpCmd.readString();
        Serial.println (data);
        StaticJsonBuffer<cmd_max_Size> jsonBuffer;
        JsonObject& root = jsonBuffer.createObject();
        commander.process (data, root);
        // start of status msg
        root[JKEY_readable] = Bma020.isBMAReadable();
        root[JKEY_bandwidth].set<int> (Bma020.getBandwidth());
        root[JKEY_range].set<int> (Bma020.getRange());
        root[JKEY_millis] = millis();

        String buf;
        root.printTo (buf);

        udpCmd.beginPacket (dest, udpCmdPort);
        udpCmd.write (buf.c_str());
        udpCmd.endPacket();
    }

    // send state is not started, send sensor state
    if (commander.started() == false)
    {
        if (statusCtr > 10)
        {
            StaticJsonBuffer<cmd_max_Size> jsonBuffer;
            JsonObject& root = jsonBuffer.createObject();

            root[JKEY_type].set<int> (sensor::MSGTYPE::STATUS);
            root[JKEY_readable] = Bma020.isBMAReadable();
            root[JKEY_range].set<int> (Bma020.getRange());
            root[JKEY_bandwidth].set<int> (Bma020.getBandwidth());
            root[JKEY_millis] = millis();

            String buf;
            root.printTo (buf);
            udpCmd.beginPacket (dest, udpCmdPort);
            udpCmd.write (buf.c_str());
            auto ret = udpCmd.endPacket();

            if (ret == 0)
            {
                Serial.println ("Packet not send!");
            }

            statusCtr = 0;
        }

        statusCtr++;
        delay (10);
    }
}
