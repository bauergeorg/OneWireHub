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
    // STATUS_WP_PAGES byte
    //  Bit 0...3: Write protect
    //  Bit 4..7: Bit mask
    static constexpr uint8_t    STATUS_PG_REDIR     { 0x01 }; // 4 byte -> Page redirection
    // STATUS_PG_REDIR byte(s)
    //  Bit 0...2: Redirection
    //  Bit 3...7: static 1
    static constexpr uint8_t    STATUS_UNDEF_B1     { 0x05 }; // 2 byte -> reserved / undefined
    static constexpr uint8_t    STATUS_DEVICE       { 0x07 }; // 1 byte -> valid device settings 
    // STATUS_DEVICE byte
    //  Bit 0: CSS0 Polarity
    //  Bit 1: CSS1 Source Select
    //  Bit 2: CSS2 Source Select
    //  Bit 3: CSS3 Channel Select
    //  Bit 4: CSS4 Channel Select
    //  Bit 5: PIO-A Channel Flip-Flop
    //  Bit 6: PIO-B Channel Flip-Flop
    //  Bit 7: Suppy Indication (read only)

    uint8_t  status[STATUS_SIZE]; // eprom status bytes

    // Channel Control Section
    static constexpr uint8_t    CONTROL_SIZE         { 2 }; // in bytes

    static constexpr uint8_t    CONTROL_1     { 0x00 }; // 1 byte
    // Channel Control Byte 1
    //  Bit 0: CRC0
    //  Bit 1: CRC1
    //  Bit 2: CHS0
    //  Bit 3: CHS1
    //  Bit 4: IC
    //  Bit 5: TOG
    //  Bit 6: IM
    //  Bit 7: ALR
    static constexpr uint8_t    CONTROL_2     { 0x01 }; // 1 byte
    // Channel Control Byte 2: always 0xFF
    // The bit assignments of Channel Control Byte 2 are reserved for future development.
    // The bus master should always send FFh for the second Channel Control Byte. 

    uint8_t  control[CONTROL_SIZE]; // channel control bytes

    // Channel Info Section
    static constexpr uint8_t    INFO_SIZE         { 1 }; // in bytes

    static constexpr uint8_t    INFO     { 0x00 }; // 1 byte
    // Channel Info Byte
    //  Bit 0: PIO-A Channel Flip-Flop Q
    //  Bit 1: PIO-B Channel Flip-Flop Q
    //  Bit 2: PIO A Sensed Level 
    //  Bit 3: PIO B Sensed Level 
    //  Bit 4: PIO-A Activity Latch
    //  Bit 5: PIO-B Activity Latch
    //  Bit 6: Number of Channels: 0 = channel A only
    //  Bit 7: Supply Indication: 0 = no supply

    uint8_t  info[INFO_SIZE]; // channel info byte

    void    clearScratchpad(void);
    uint8_t  translateRedirection(uint8_t source_address) const; // react to redirection in status and not available memory

public:

    static constexpr uint8_t family_code        { 0x12 };
    static constexpr uint8_t chip_type          { 0 };      // Chip type = 0x00 for the DS2406+, 3 pin package TO-92 (No Vcc, only one switch: PIO-A) (default)
                                                            // Chip type = 0x01 for the DS2406P+, 6 pin package TSOC (Vcc Pin, two switches: PIO-A & PIO-B)

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

    // Pin Section
    /*
    void    setPinState(uint8_t pinNumber, bool value);
    bool    getPinState(uint8_t pinNumber) const;
    uint8_t getPinState(void) const;

    void    setPinActivity(uint8_t pinNumber, bool value);
    bool    getPinActivity(uint8_t pinNumber) const;
    uint8_t getPinActivity(void) const;
    */

    // Supply Indictaion Section
    void DS2406::setSupplyIndication(bool value);
    void DS2406::clearSupplyIndication(void);
    uint8_t DS2406::getSupplyIndication(void) const;

    // Channel Control Section
    void DS2406::writeChannelControl(uint8_t value);
    void DS2406::clearChannelControl(void);
    uint8_t DS2406::readChannelControl(void) const;

    // Channel Info Section
    void DS2406::writeChannelInfo(uint8_t value);
    void DS2406::clearChannelInfo(void);
    uint8_t DS2406::readChannelInfo(void) const;

};

#endif
