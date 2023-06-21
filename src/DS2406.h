// 1Kb 1-Wire OTP EPROM with dual channel addressable switch
// no not work fine!
// note: datasheet is fuzzy, but device is similar to ds2413 and ds2431
// native bus-features: Overdrive capable

#ifndef ONEWIRE_DS2406_H
#define ONEWIRE_DS2406_H

#include "OneWireItem.h"

constexpr uint8_t value_xFF { static_cast<uint8_t>(0xFF) };
constexpr uint8_t value_x00 { static_cast<uint8_t>(0x00) };

class DS2406 : public OneWireItem
{
private:

    // Memory Section
    static constexpr uint8_t  MEM_SIZE          { 128 }; // in byte, TODOgba: 144

    static constexpr uint8_t  PAGE_SIZE         { 32 };  // in byte
    static constexpr uint8_t  PAGE_COUNT        { MEM_SIZE / PAGE_SIZE }; // at the moment = 4
    static constexpr uint8_t  PAGE_MASK         { PAGE_SIZE - 1 };

    static constexpr uint8_t  SCRATCHPAD_SIZE   { 8 };
    static constexpr uint8_t  SCRATCHPAD_MASK   { 0b00000111 };

    static constexpr uint8_t  REG_ES_PF_MASK    { 0b00100000 }; // partial byte flag
    static constexpr uint8_t  REG_ES_ZERO_MASK  { 0b01011000 }; // reads always zero
    static constexpr uint8_t  REG_ES_AA_MASK    { 0b10000000 }; // authorization accepted (data copied to target memory)

    uint8_t memory[MEM_SIZE]; // 4 pages of 32 bytes
    uint8_t scratchpad[SCRATCHPAD_SIZE];

    // Pin State Section
    bool pin_state[2];  // sensed input for A and B
    bool pin_latch[2];  // PIO can be set to input (0) or output-to-zero (1)

    // Status Section (copied from DS2502)
    static constexpr uint8_t    STATUS_SIZE         { 8 }; // in bytes
    
    static constexpr uint8_t    STATUS_WP_PAGES     { 0x00 }; // 1 byte -> Page write protection (bit 0-3) and page used status (bit 4-7)
    static constexpr uint8_t    STATUS_PG_REDIR     { 0x01 }; // 4 byte -> Page redirection
    static constexpr uint8_t    STATUS_UNDEF_B1     { 0x05 }; // 2 byte -> reserved / undefined
    static constexpr uint8_t    STATUS_DEVICE       { 0x07 }; // 1 byte -> valid device settings 

    uint8_t  status[STATUS_SIZE]; // eprom status bytes

    void    clearScratchpad(void);
    uint8_t  translateRedirection(uint8_t source_address) const; // react to redirection in status and not available memory

public:

    static constexpr uint8_t family_code        { 0x12 };

    DS2406(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7);

    void    duty(OneWireHub * hub) final;

    // Memory Section
    void    clearMemory(void);

    bool    writeMemory(const uint8_t* source, uint8_t length, uint8_t position = 0);
    bool    readMemory(uint8_t* destination, uint16_t length, uint16_t position = 0) const;

    // Status Section
    void    clearStatus(void);

    uint8_t writeStatus(uint16_t address, uint8_t value);
    uint8_t readStatus(uint16_t address) const;
    
    void    setPageProtection(uint8_t page);
    bool    getPageProtection(uint8_t page) const; 

    void    setPageUsed(uint8_t page);
    bool    getPageUsed(uint8_t page) const;

    bool    setPageRedirection(uint8_t page_source, uint8_t page_destin);
    uint8_t getPageRedirection(uint8_t page) const;


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
