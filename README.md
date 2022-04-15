# Sensor board

## This project contains a platformIO project for the sensor board for the HitMe project.

It contains an ESP8266 and a connected BMA020 acceleration sensor.

## Hints for me

### Communications
- 3 udp channels
- 1 command `nc -n -u 192.168.137.69 10001`
- 2 command back channel `nc -l -u -p 10001`
- 3 data channel `nc -l -u -p 10001`

### json command format

Command package
```
{
    "type"  : 0
    "start" : 0
    "range" : 1
    "bandwidth" : 0
}
```
_type_ (for request and response)  
```
enum MSGTYPE
{
    PARSEERR = -1,
    REQUEST_MSG = 0,
    ANSWER_MSG,
    STATUS_MSG
};
```

_start_  
0 -> stops measurement  
1 -> starts measurement

_ range_
```
enum BMA020RANGE
{
    BMA020_RANGE_2G = 0x00, // 00b
    BMA020_RANGE_4G = 0x01, // 01b
    BMA020_RANGE_8G = 0x02  // 10b
};
```
_bandwidth_  
```
enum BMA020BANDWIDTH
{
    BMA020_BW_25HZ = 0x00,  // 000b (mean 23 Hz)
    BMA020_BW_50HZ = 0x01,  // 001b (mean 47 Hz)
    BMA020_BW_100HZ = 0x02, // 010b (mean 94 Hz)
    BMA020_BW_190HZ = 0x03, // 011b (mean 188 Hz)
    BMA020_BW_375HZ = 0x04, // 100b (mean 375 Hz)
    BMA020_BW_750HZ = 0x05, // 101b (mean 750 Hz)
    BMA020_BW_1500HZ = 0x06 // 110b (mean 1500 Hz)
};
```


Status package
```
{
    "type":2,
    "readable":true,
    "range":0,
    "bandwidth":0,
    "millis":687678
}
```





Data package  
Some weird byte mix I'll have to figure out...
