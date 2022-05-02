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

IPAddress dest;
const uint16_t udpDataPort = HITME_DATAPORT;
const uint16_t udpCmdPort = HITME_CTRLPORT;

void setup_wifi()
{
    digitalWrite (LED_BUILTIN, LOW);
    delay (10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print (F ("Connecting to "));
    Serial.println (HITME_SSID);
    WiFi.hostname ("schlag_1");
    WiFi.mode (WiFiMode::WIFI_STA);
    WiFi.begin (HITME_SSID, HITME_PASS);

    while (WiFi.status() != WL_CONNECTED)
    {
        digitalWrite (LED_BUILTIN, LOW);
        delay (500);
        Serial.print (".");
        digitalWrite (LED_BUILTIN, HIGH);
    }

    if (!dest.fromString (HITME_STDUIIP))
    {
        Serial.println (F ("Could not parse dest IP!"));
    }

    Serial.println ();
    Serial.println (F ("WiFi connected"));
    Serial.println (F ("IP address: "));
    Serial.println (WiFi.localIP());
    digitalWrite (LED_BUILTIN, HIGH);
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

const uint8_t bufferedAccData = 90;
const uint16_t accDataBufSize = BMA020::accPacket * bufferedAccData;
// start and end of data block, packetid and acc data buffer size
const uint16_t sendBufSize = sizeOfTimeStamp + sizeOfTimeStamp +
                             sizeOfPacketId + accDataBufSize;
uint8_t* sendBuf;
uint8_t* accDataBuf;

void setup()
{
    pinMode (LED_BUILTIN, OUTPUT); // Initialize the BUILTIN_LED pin as an output
    // Serial Stuff
    Serial.begin (9600);
    Serial.setDebugOutput (true);
    Serial.println (F ("Start..."));
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
        Serial.print (F ("Restarting BMA -> \t"));
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
    Serial.print (F ("UDP port data "));
    Serial.print (udpData.localPort());
    Serial.println();
    Serial.print (F ("UDP port cmd "));
    Serial.print (udpCmd.localPort());
    Serial.println();

    Cmder.printHelp();

    Serial.println (F ("Start looping!"));
}

uint8_t statusCtr = 0;
uint16_t dataBlockCounter = 0;
uint32_t packetCtr = 0;
uint32_t startTime = micros();
void loop()
{
    using namespace ArduinoJson;

    if (Cmder.started() == true)
    {
        // set start time, when last block was sesnd
        if (dataBlockCounter == 0)
        {
            startTime = micros();
        }

        // return true, if accDataBufSize of data was read
        if (Bma020.tryFetchNewData (accDataBuf, dataBlockCounter, accDataBufSize))
        {
            // get time and set end time
            uint32_t endTime = micros();

            // data measuring completed for this block
            uint8_t cpyStart = 0;
            // cpy start time
            conv.data1x32 = startTime;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfTimeStamp);
            cpyStart += sizeof (startTime);
            conv.data1x32 = endTime;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfTimeStamp);
            cpyStart += sizeof (endTime);
            // cpy packet id
            conv.data1x32 = packetCtr;
            memcpy (sendBuf + cpyStart, conv.data4x8, sizeOfPacketId);
            cpyStart += sizeof (packetCtr);
            // cpy acc data
            memcpy (sendBuf + cpyStart, accDataBuf, accDataBufSize);
            // send stuff
            udpData.beginPacket (dest, udpDataPort);
            udpData.write (sendBuf, sendBufSize);
            auto ret = udpData.endPacket();

            if (ret == 0)
            {
                Serial.println ("Packet not send!");
            }

            // reset data ctr
            dataBlockCounter = 0;
            packetCtr++;
        }
    }

    // check and parse ctrl packets
    uint16_t len = udpCmd.parsePacket();

    if (len)
    {
        String data = udpCmd.readString();
        Serial.println (data);
        // todo: create some return class, that contains the error states of processing
       
        auto res = Cmder.process (data);
        Serial.printf("%s %s\n", res.errMessage.c_str(), res.success ? "ok" : "error");

        // todo: Calculate cmd_max_Size with JSON_OBJECT_SIZE
        // todo: Introduce response message to config changes
        StaticJsonDocument<cmd_max_Size> root;
        // start of status msg
        root[JKEY_type].set<int> (res.messageType);
        root[JKEY_start] = Cmder.started();
        root[JKEY_readable] = Bma020.isBMAReadable();
        root[JKEY_bandwidth].set<int> (Bma020.getBandwidth());
        root[JKEY_range].set<int> (Bma020.getRange());
        root[JKEY_millis] = millis();

        String buf;
        serializeJson(root, buf);

        // todo: pass udp client directly
        udpCmd.beginPacket (dest, udpCmdPort);
        udpCmd.write (buf.c_str());
        udpCmd.endPacket();
    }

    // send state is not started, send sensor state
    if (Cmder.started() == false)
    {
        if (statusCtr > 10)
        {
            StaticJsonDocument<cmd_max_Size> root;
            root[JKEY_type].set<int> (sensor::MSGTYPE::STATUS_MSG);
            root[JKEY_start] = Cmder.started();
            root[JKEY_readable] = Bma020.isBMAReadable();
            root[JKEY_range].set<int> (Bma020.getRange());
            root[JKEY_bandwidth].set<int> (Bma020.getBandwidth());
            root[JKEY_millis] = millis();

            String buf;
            serializeJson(root, buf);
            udpCmd.beginPacket (dest, udpCmdPort);
            udpCmd.write (buf.c_str());
            auto ret = udpCmd.endPacket();

            if (ret == 0)
            {
                Serial.println (F ("Packet not send!"));
            }
            statusCtr = 0;
        }

        statusCtr++;
        delay (10);
    }
}
