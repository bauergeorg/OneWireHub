// 0x1D  4kb 1-Wire RAM with Counter

#ifndef ONEWIRE_DS2423_H
#define ONEWIRE_DS2423_H

class DS2423 : public OneWireItem
{
private:
    static constexpr bool    dbg_sensor  = 0; // give debug messages for this sensor

public:
    static constexpr uint8_t family_code = 0x1D;

    DS2423(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7);

    bool duty(OneWireHub *hub);
};

#endif