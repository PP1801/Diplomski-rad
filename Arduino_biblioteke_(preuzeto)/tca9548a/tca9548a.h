#ifndef TCA9548A_H
#define TCA9548A_h

#include <Arduino.h>
#include <Wire.h>

#define TCA9548A_DEFAULT_I2C_ADDR 0x70

class TCA9548A
{
    public:
        enum tcaChannel
        {
            TCA_CHANNEL_0   = 0x1,
            TCA_CHANNEL_1   = 0x2,
            TCA_CHANNEL_2   = 0x4,
            TCA_CHANNEL_3   = 0x8,
            TCA_CHANNEL_4   = 0x10,
            TCA_CHANNEL_5   = 0x20,
            TCA_CHANNEL_6   = 0x40,
            TCA_CHANNEL_7   = 0x80,
        };

        TCA9548A(uint8_t address = TCA9548A_DEFAULT_I2C_ADDR);  // Default IC Address
        ~TCA9548A();

        void begin(TwoWire &inBus = Wire); // Default TwoWire Instance
        void openOneChannel(uint8_t channel);
        void closeOneChannel(uint8_t channel);
        void openCustomChannels(uint8_t channelBuff);
        void closeCustomChannels(uint8_t channelBuff);
        void closeAllChannels();
        void openAllChannels();
        
    protected:
    private:
        TwoWire *_i2c_bus;
        uint8_t _address;
        uint8_t _channels;

        void write(uint8_t inData);
};

#endif