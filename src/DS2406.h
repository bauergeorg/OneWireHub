// 1Kb 1-Wire OTP EPROM with dual channel addressable switch
// no not work fine!
// note: datasheet is fuzzy, but device is similar to ds2413 and ds2431
// native bus-features: Overdrive capable

#ifndef ONEWIRE_DS2406_H
#define ONEWIRE_DS2406_H

#include "OneWireItem.h"

class DS2406 : public OneWireItem
{
private:

    // Memory Section
    static constexpr uint8_t  MEM_SIZE          { 144 };

    static constexpr uint8_t  PAGE_SIZE         { 32 };
    static constexpr uint8_t  PAGE_COUNT        { MEM_SIZE / PAGE_SIZE };
    static constexpr uint8_t  PAGE_MASK         { 0b00011111 };

    static constexpr uint8_t  SCRATCHPAD_SIZE   { 8 };
    static constexpr uint8_t  SCRATCHPAD_MASK   { 0b00000111 };

    static constexpr uint8_t  REG_ES_PF_MASK    { 0b00100000 }; // partial byte flag
    static constexpr uint8_t  REG_ES_ZERO_MASK  { 0b01011000 }; // reads always zero
    static constexpr uint8_t  REG_ES_AA_MASK    { 0b10000000 }; // authorization accepted (data copied to target memory)

    static constexpr uint8_t  WP_MODE           { 0x55 }; // write protect mode
    static constexpr uint8_t  EP_MODE           { 0xAA }; // eprom mode

    uint8_t memory[MEM_SIZE];

    // Pin State Section
    bool pin_state[2];  // sensed input for A and B
    bool pin_latch[2];  // PIO can be set to input (0) or output-to-zero (1)


    uint8_t scratchpad[SCRATCHPAD_SIZE];
    uint8_t page_protection;
    uint8_t page_eprom_mode;

    bool    updatePageStatus(void);
    void    clearScratchpad(void);

public:

    static constexpr uint8_t family_code        { 0x12 };

    DS2406(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7);

    void    duty(OneWireHub * hub) final;

    // Memory Section
    void    clearMemory(void);

    bool    writeMemory(const uint8_t* source, uint8_t length, uint8_t position = 0);
    bool    readMemory(uint8_t* destination, uint16_t length, uint16_t position = 0) const;

    void    setPageProtection(uint8_t position);
    bool    getPageProtection(uint8_t position) const;

    void    setPageEpromMode(uint8_t position);
    bool    getPageEpromMode(uint8_t position) const;

    // Pin State Section
    bool    setPinState(const uint8_t a_or_b, const bool value)
    {
        if (value && pin_latch[a_or_b & 1]) return false; // can't set 1 because pin is latched
        pin_state[a_or_b & 1] = value;
        return true;
    }

    bool    getPinState(const uint8_t a_or_b) const
    {
        return pin_state[a_or_b & 1];
    }

    void    setPinLatch(const uint8_t a_or_b, const bool value) // latching a pin will pull it down (state=zero)
    {
        pin_latch[a_or_b & 1] = value;
        if (value) setPinState(a_or_b, false);
    }

    bool    getPinLatch(const uint8_t a_or_b) const
    {
        return pin_latch[a_or_b & 1];
    }

};

#endif
