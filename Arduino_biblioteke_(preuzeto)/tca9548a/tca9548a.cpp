#include "tca9548a.h"
#include "Arduino.h"


TCA9548A::TCA9548A(uint8_t address) 
: _address(address) {}

TCA9548A::~TCA9548A(){}


void TCA9548A::begin(TwoWire &inBus)
{
    _i2c_bus = &inBus;
}

void TCA9548A::openOneChannel(uint8_t channel)
{
    if(channel > 7 ) return;
    uint8_t buff = 0x00;    
    buff = 1 << channel;
    _channels |= buff;
    write(_channels);
}

void TCA9548A::closeOneChannel(uint8_t channel)
{
    if(channel > 7 ) return;
    uint8_t buff = 0x00;    
    buff = 1 << channel;    
    _channels ^= buff;
    write(_channels);
}

/**************************************************************************/
/*!
    @brief  Open custom channels using tcaChannel enum and | operator.
    @param  channelBuff tcaChannel enum
    e.g.: openCustomChannels(TCA_CHANNEL_0 | TCA_CHANNEL_4)
*/
/**************************************************************************/

void TCA9548A::openCustomChannels(uint8_t channelBuff)
{
    if(channelBuff > 7 ) return;
    _channels |= channelBuff;
    write(_channels);
}
/**************************************************************************/
/*!
    @brief  Same as openCustomChannels() but for closing.
*/
/**************************************************************************/
void TCA9548A::closeCustomChannels(uint8_t channelBuff)
{
    if(channelBuff > 7 ) return;
    _channels ^= channelBuff;
    write(_channels);
}

void TCA9548A::closeAllChannels()
{
    this->_channels = 0x00;
    write(this->_channels);
}

void TCA9548A::openAllChannels()
{
    this->_channels = 0xFF;
    write(this->_channels);
}

void TCA9548A::write(uint8_t inData)
{
    _i2c_bus->beginTransmission(_address);
    _i2c_bus->write(inData);
    _i2c_bus->endTransmission();
}
