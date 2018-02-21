#include "bma020.h"

#include <Arduino.h>

// register and low level stuff

// default chip id (r/o)
constexpr uint8_t _DEFAULT_CHIPID = 0x02u; // 010b

// R14h ranges
constexpr uint8_t _RANGE0 = 0x3u;
constexpr uint8_t _RANGE1 = 0x4u;
constexpr uint8_t _bit_mask_Range = (1 << _RANGE0) | (1 << _RANGE1);

// R14h bandwidth
constexpr uint8_t _BW0 = 0x0u;
constexpr uint8_t _BW1 = 0x1u;
constexpr uint8_t _BW2 = 0x2u;
constexpr uint8_t _bit_mask_Bandwidth = (1 << _BW0) | (1 << _BW1) | (1 << _BW2);

// ACC data
constexpr uint8_t _ACC_Start_ADR = 0x02u;
constexpr uint8_t _ACC_uint8_t_Count = 6u;
constexpr uint8_t _acc_reg_resolution = 10u;
// acc new data interrupt
constexpr uint8_t _NewData_INT_BIT = 5u;
constexpr uint8_t _bit_mask_NewData_INT = (1 << _NewData_INT_BIT);
std::function<void (void)> isr_func = nullptr;

// transfer two complement into dec
int16_t twoCompToDec (uint16_t in, uint8_t highestBit)
{
    int16_t erg = in;

    if ((erg >> (highestBit - 1)) > 0)
    {
        erg |= 1 << (highestBit - 1);
        erg -= 1 << highestBit;
    }

    return erg;
}
// assemble value from register content
int16_t getDataFromACCRegisters (uint8_t lsb, uint8_t msb)
{
    return (msb << 2) + (lsb >> 6);
}
// isr sub routine
void ISR_newdata (void)
{
    if (isr_func != nullptr)
    {
        isr_func();
    }
}

// member to process isr data
void BMA020::_isr_callback()
{
    // schreib das mal außerhalb einer int routine hin...
    beginTransmission (_adr);
    write (_ACC_Start_ADR);
    endTransmission();

    requestFrom (_adr, _ACC_uint8_t_Count);

    while (available())
    {
        if (posToWriteDataAt >= getAccBuflen())
        {
            posToWriteDataAt = 0;
        }

        isrData[posToWriteDataAt] = read();
        posToWriteDataAt ++;
    }

// latched interrupts
//    uint8_t rah = readReg (BMA020REGISTER::E_CTRL_0A);
//    rah |= (1 << 6);
//    writeReg (BMA020REGISTER::E_CTRL_0A, rah);

    ctrSaveToReadTo = posToWriteDataAt;
}

uint16_t BMA020::getCtrSaveToReadTo()
{
    uint16_t retVal;
    noInterrupts();
    retVal = ctrSaveToReadTo;
    interrupts();
    return retVal;
}

uint16_t BMA020::getAccBuflen()
{
    return  ACCBUFFLEN;
}

bool BMA020::getData (uint8_t* buf, uint16_t& size)
{
    uint16_t readUntil = getCtrSaveToReadTo();
    uint16_t sctr = 0;

    while ( (last != readUntil) && (sctr < size))
    {
        if (last >= getAccBuflen())
        {
            last = 0;
        }

        buf[sctr] = isrData[last];
        sctr++;
        last++;

    }

    return true;
}

// access to registers
void BMA020::writeReg (BMA020REGISTER reg, uint8_t value)
{
    beginTransmission (_adr);
    write (reg);
    write (value);
    endTransmission();
}

uint8_t BMA020::readReg (BMA020::BMA020REGISTER reg)
{
    beginTransmission (_adr);
    write (reg);
    endTransmission();
    requestFrom (_adr, (uint8_t)1);
    return read();
}

// get or set "properties" of BMA020

uint8_t BMA020::getChipId()
{
    return readReg (BMA020::E_CHIPID);
}

bool BMA020::update_acc (BMA020::raw_acc_t &data)
{
    if (_isSetup == false)
    {
        return false;
    }

    beginTransmission (_adr);
    write (_ACC_Start_ADR);
    uint8_t succ = endTransmission();
    uint8_t acc_regs[_ACC_uint8_t_Count];
    uint8_t cnt = requestFrom (_adr, _ACC_uint8_t_Count);

    for (uint8_t ctr = 0; ctr < cnt && ctr < _ACC_uint8_t_Count; ctr++)
    {
        acc_regs[ctr] = read();
    }

    data.acc_x = getDataFromACCRegisters (acc_regs[0], acc_regs[1]);
    data.acc_y = getDataFromACCRegisters (acc_regs[2], acc_regs[3]);
    data.acc_z = getDataFromACCRegisters (acc_regs[4], acc_regs[5]);

    return succ == 0;
}

bool BMA020::calculateGFactor()
{
    bool ok = isOk();
    auto r = getRange();
    _gfactor = 1.0 * (1 << (static_cast<int> (r) + 1)) /
               static_cast<double_t> (((1 << (_acc_reg_resolution - 1))));
    return ok;
}

bool BMA020::setNewDataInterrupt (bool fire)
{
    if (_isSetup == false)
    {
        return false;
    }

    uint8_t r15h = readReg (BMA020REGISTER::E_CONTROL_OP);

    if (fire)
    {
        r15h |= _bit_mask_NewData_INT;
    }
    else
    {
        r15h &= ~_bit_mask_NewData_INT;
    }

    // no motion detectiosn
    r15h &= ~ (1 << 6);
    // latched interrupts
    r15h |=  (1 << 4);

    writeReg (BMA020REGISTER::E_CONTROL_OP, r15h);

    return true;
}

bool BMA020::getNewDataInterrupt()
{
    uint8_t r15h = readReg (BMA020REGISTER::E_CONTROL_OP);
    return (r15h & _bit_mask_NewData_INT) > 0;
}

constexpr uint8_t att_mask = (uint8_t) ((1 << 7) | (1 << 5) | (1 << 6));
bool BMA020::setRange (BMA020::BMA020RANGE range)
{
    uint8_t reg_14h = readReg (E_SETUP_ACC);
    // preserve highest bits
    uint8_t attention = reg_14h & att_mask;
    // 11100111b = 11111111b ^ 00011000b
    uint8_t delMask = ~_bit_mask_Range;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= range << _RANGE0;
    // restore highest bits
    delMask = ~att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;
    writeReg (E_SETUP_ACC, reg_14h);
    // update g factor
    calculateGFactor();

    return true;
}

BMA020::BMA020RANGE BMA020::getRange()
{
    uint8_t reg_14h = (readReg (E_SETUP_ACC) & _bit_mask_Range) >> _RANGE0;
    return static_cast<BMA020RANGE> (reg_14h);
}

bool BMA020::setBandwidth (BMA020::BMA020BANDWIDTH bandwidth)
{
    auto reg_14h = readReg (E_SETUP_ACC);
    // preserve highest bits
    uint8_t attention = reg_14h & att_mask;
    // 11100111b = 11111111b ^ 00011000b
    auto delMask =  ~_bit_mask_Bandwidth;
    // clear wanted bits
    reg_14h &= delMask;
    // and reset them with new value
    reg_14h |= ((uint8_t)bandwidth << _BW0);
    // restore highest bits
    delMask = ~att_mask;
    reg_14h &= delMask;
    reg_14h |= attention;
    writeReg (E_SETUP_ACC, reg_14h);
    return true;
}

BMA020::BMA020BANDWIDTH BMA020::getBandWidth()
{
    uint8_t reg_14h = (readReg (E_SETUP_ACC) & _bit_mask_Bandwidth) >> _BW0;
    return static_cast<BMA020BANDWIDTH> (reg_14h);
}

BMA020::raw_acc_t &BMA020::getRawData()
{
    update_acc (_raw_data);
    return _raw_data;
}

BMA020::dec_acc_t BMA020::getDecData (const raw_acc_t &raw) const
{
    dec_acc_t data;
    data.acc_x = twoCompToDec (raw.acc_x, _acc_reg_resolution);
    data.acc_y = twoCompToDec (raw.acc_y, _acc_reg_resolution);
    data.acc_z = twoCompToDec (raw.acc_z, _acc_reg_resolution);
    return data;
}

BMA020::g_data_t BMA020::getGData (const raw_acc_t &raw) const
{
    g_data_t _g_data;
    dec_acc_t dec = getDecData (raw);
    _g_data.acc_x = static_cast<double_t> (dec.acc_x) * _gfactor;
    _g_data.acc_y = static_cast<double_t> (dec.acc_y) * _gfactor;
    _g_data.acc_z = static_cast<double_t> (dec.acc_z) * _gfactor;
    return _g_data;
}

String BMA020::getStatus()
{
    const uint8_t ms = 10;
    uint8_t ctr_ra = BMA020::BMA020_RANGE_8G;
    uint8_t ctr_bw = BMA020::BMA020_BW_1500HZ;
    bool rok = true;

    for (uint8_t ctr = ctr_ra + 1; ctr > 0; ctr--)
    {
        setRange (static_cast<BMA020::BMA020RANGE> (ctr - 1));
        delay (ms);
        rok &= getRange() == ctr - 1;
    }

    delay (100);
    bool bok = true;

    for (uint8_t ctr = ctr_bw + 1; ctr > 0; ctr--)
    {
        setBandwidth (static_cast<BMA020::BMA020BANDWIDTH> (ctr - 1));
        delay (ms);
        bok &= getBandWidth() == ctr - 1;
    }

    delay (ms);
    String res = String ("on:") + (isOk() ? " OK" : "NOK") + ' ';
    res += String ("rg:") + (rok ? " OK" : "NOK") + ' ';
    res += String ("bw:") + (bok ? " OK" : "NOK") + ' ';
    res += String ("NI:") + (getNewDataInterrupt() ? " ON" : "OFF") + ' ';
    return res;
}

bool BMA020::setupInterruptNewDataMode (uint8_t interruptBMAOutPin, bool enable)
{
    if (_isSetup == false)
    {
        return false;
    }

    pinMode (interruptBMAOutPin, INPUT);

    attachInterrupt (digitalPinToInterrupt (interruptBMAOutPin), ISR_newdata,
                     RISING); // RISING

    return setNewDataInterrupt (enable);
}

BMA020::BMA020() : TwoWire(), _isSetup (false)
{
    isrData = (uint8_t*)malloc (ACCBUFFLEN * sizeof (uint8_t));
    setup();
}

BMA020::BMA020 (uint8_t adr, uint8_t sda, uint8_t scl) : TwoWire(),
    _isSetup (false)
{
    setup (sda, scl, adr);
}

bool BMA020::setup (uint8_t sda, uint8_t scl, uint8_t adr, BMA020RANGE range,
                    BMA020BANDWIDTH bandwidth)
{
    last = 0;
    TwoWire::begin (sda, scl);
    _adr = adr;
    // soft reset
    uint8_t rag = readReg (BMA020REGISTER::E_CTRL_0A);
    rag |= (1 << 1);
    writeReg (BMA020REGISTER::E_CTRL_0A, rag);

    // wait a bit for things to settle
    delay (20);
    bool ok = isOk();

    if (!ok)
    {
        _isSetup = false;
        return ok;
    }

    _isSetup = ok;

    setRange (range);
    setBandwidth (bandwidth);
    // set interrupt routine
    isr_func = std::bind (&BMA020::_isr_callback, this);
    // disable interrupt
    setNewDataInterrupt (false);
    // calculate factor to calculate g rates
    ok &= calculateGFactor();
    _isSetup = ok;
    return _isSetup;
}

bool BMA020::isOk()
{
    return getChipId() == _DEFAULT_CHIPID;
}
