#include "DS2406.h"

DS2406::DS2406(uint8_t ID1, uint8_t ID2, uint8_t ID3, uint8_t ID4, uint8_t ID5, uint8_t ID6, uint8_t ID7) : OneWireItem(ID1, ID2, ID3, ID4, ID5, ID6, ID7)
{
    
    // Memory Sections
    static_assert(sizeof(scratchpad) < 256, "Implementation does not cover the whole address-space");
    static_assert(sizeof(memory) < 256,  "Implementation does not cover the whole address-space");

    // define default value DS2406+
    chip_type = 0;
    channel_size = 1;

    clearStatus();
    clearMemory();

    clearControl();
    clearInfo();

    clearScratchpad();

}

void DS2406::duty(OneWireHub * const hub)
{
    static uint16_t   reg_TA         { 0 }; // contains TA1, TA2
    uint16_t          reg_RA         { 0 }; // target address
    uint16_t          crc            { 0 };
    uint8_t           cmd, data;            // command, data, crc

    // always receives cmd and TA
    if (hub->recv(&cmd,1,crc))  return;
    if (hub->recv(reinterpret_cast<uint8_t *>(&reg_TA),2,crc))  return;

    switch (cmd)
    {
        case 0xF0:      // READ MEMORY // copied from DS2506

            while (reg_TA <= MEM_SIZE)
            {
                const uint16_t destin_TA = translateRedirection(reg_TA);
                const uint8_t  length    = PAGE_SIZE - uint8_t(reg_TA & PAGE_MASK);

                if (destin_TA < MEM_SIZE)
                {
                    if (hub->send(&memory[destin_TA],length,crc)) return;
                }
                else // fake data
                {
                    data  = 0x00;
                    uint8_t counter = length;
                    while (counter-- > 0)
                    {
                        if (hub->send(&data, 1, crc)) return;
                    }
                }

                reg_TA += length;
            }
            crc = ~crc; // normally crc16 is sent ~inverted
            hub->send(reinterpret_cast<uint8_t *>(&crc),2);
            break; // datasheet says we should return 1s, till reset, nothing to do here

        case 0xA5:      // EXTENDED READ MEMORY (with redirection-information) // copied from DS2506

            while (reg_TA <= MEM_SIZE)
            {
                const uint8_t  source_page = static_cast<uint8_t>(reg_TA >> 5);
                const uint8_t  destin_page = getPageRedirection(source_page);
                if (hub->send(&destin_page,1,crc))                        return;
                crc = ~crc; // normally crc16 is sent ~inverted
                hub->send(reinterpret_cast<uint8_t *>(&crc),2); // send crc of (cmd,TA,destin_page) at first, then only crc of (destin_page)
                crc=0;
                const uint16_t destin_TA   = translateRedirection(reg_TA);
                const uint8_t  length      = PAGE_SIZE - uint8_t(reg_TA & PAGE_MASK);

                if (destin_TA < MEM_SIZE)
                {
                    if (hub->send(&memory[destin_TA],length,crc)) return;
                }
                else // fake data
                {
                    data  = 0x00;
                    uint8_t counter = length;
                    while (counter-- != 0)
                    {
                        if (hub->send(&data, 1, crc)) return;
                    }
                }

                crc = ~crc; // normally crc16 is sent ~inverted
                if (hub->send(reinterpret_cast<uint8_t *>(&crc),2)) break;
                reg_TA += length;
                crc=0;
            }
            break; // datasheet says we should return 1s, till reset, nothing to do here

        case 0x0F:      // WRITE MEMORY // copied from DS2506

            while (reg_TA < MEM_SIZE) // check for valid address
            {
                if (hub->recv(&data,1,crc)) break;

                crc = ~crc; // normally crc16 is sent ~inverted
                if (hub->send(reinterpret_cast<uint8_t *>(&crc), 2)) break;
                // master issues now a 480us 12V-Programming Pulse -> advantage for us, enough time to handle addressMapping

                reg_RA = translateRedirection(reg_TA);
                const uint8_t  page = static_cast<uint8_t>(reg_RA>>5);
                if (getPageProtection(page))
                {
                    const uint8_t mem_zero = 0x00;
                    if (hub->send(&mem_zero)) break;
                }
                else
                {
                    memory[reg_RA] &= data; // like EEPROM-Mode
                    setPageUsed(page);
                    if (hub->send(&memory[reg_RA])) break;
                }
                crc = ++reg_TA; // prepare new loop
            }
            break;

        case 0x55:      // WRITE STATUS // copied from DS2506

            while (reg_TA < STATUS_SIZE) // check for valid address
            {
                if (hub->recv(&data,1,crc)) break;

                crc = ~crc; // normally crc16 is sent ~inverted
                if (hub->send(reinterpret_cast<uint8_t *>(&crc), 2)) break;
                // master issues now a 480us 12V-Programming Pulse

                data = writeStatus(reg_TA, data);
                if (hub->send(&data)) break;
                crc = ++reg_TA; // prepare new loop
            }
            break;

        case 0xAA:      // READ STATUS // copied from DS2506

            while (reg_TA < STATUS_SIZE) // check for valid address
            {
                reg_RA = reg_TA & uint8_t(7);
                while (reg_RA < 8)
                {
                    data = readStatus(reg_TA); // read byte by byte
                    if (hub->send(&data, 1, crc)) return;
                    reg_RA++;
                    reg_TA++;
                }
                crc = ~crc; // normally crc16 is sent ~inverted
                hub->send(reinterpret_cast<uint8_t *>(&crc),2);
                crc = 0;
            }
            break;

        case 0xF5:      // CHANNEL-ACCESS

            static uint8_t channel_control_byte_1 = (reg_TA >> 8) & uint8_t(0xFF);
            static uint8_t channel_control_byte_2 = reg_TA & uint8_t(0xFF);

            
            // channel_control_byte_2 should be 0xFF
            writeControl(channel_control_byte_1);
            data = readInfo();
            if (hub->send(&data, 1, crc)) return;

            static uint8_t info;
            static bool toggle;
            // if IM is set in channel control byte: write mode (toggle = 0)
            if (channel_control_byte_1 & 0x40) {
                toggle = 0;
                if (hub->recv(&info, 1, crc))  return;
                writeInfo(info);
            }
            // if IM is not set in channel control byte: read mode (toggle = 1)
            else{
                toggle = 1;
                info = readInfo();
                if (hub->send(&info, 1, crc)); return;
            }

            // if CRC enabled
            if (channel_control_byte_1 & 0x3) {
                crc = ~crc; // normally crc16 is sent ~inverted
                if (hub->send(reinterpret_cast<uint8_t *>(&crc),2)) return;
                crc = 0;
            }

            break;


            /*
            while (true)
            {
                static uint16_t crc2 = crc;
                if (hub->send(memory,4,crc)) return;
                crc = ~crc; // most important step, easy to miss....
                if (hub->send(reinterpret_cast<uint8_t *>(&crc),2)) return;
                crc = crc2;
            }*/

        /*
        case 0x0F:      // WRITE SCRATCHPAD COMMAND

            if (hub->recv(reinterpret_cast<uint8_t *>(&reg_TA),2,crc))  return;
            reg_ES = uint8_t(reg_TA) & SCRATCHPAD_MASK;
            page_offset = reg_ES;

            // receive up to 8 bytes of data
            for (; reg_ES < SCRATCHPAD_SIZE; ++reg_ES)
            {
                if (hub->recv(&scratchpad[reg_ES], 1, crc))
                {
                    if (hub->getError() == Error::AWAIT_TIMESLOT_TIMEOUT_HIGH) reg_ES |= REG_ES_PF_MASK;
                    break;
                }
            }
            reg_ES--;
            reg_ES &= SCRATCHPAD_MASK;

            if (hub->getError() == Error::NO_ERROR)  // try to send crc if wanted
            {
                crc = ~crc; // normally crc16 is sent ~inverted
                hub->send(reinterpret_cast<uint8_t *>(&crc), 2);
            }

            if (reg_TA < (4*PAGE_SIZE)) // check if page is protected or in eprom-mode
            {
                const uint8_t position = uint8_t(reg_TA) & ~SCRATCHPAD_MASK;
                if (getPageProtection(reinterpret_cast<uint8_t *>(&reg_TA)[0]))       // protected: load memory-segment to scratchpad
                {
                    for (uint8_t i = 0; i < SCRATCHPAD_SIZE; ++i) scratchpad[i] = memory[position + i];
                }
                else if (getPageEpromMode(reinterpret_cast<uint8_t *>(&reg_TA)[0]))   // eprom: logical AND of memory and data
                {
                    for (uint8_t i = page_offset; i <= reg_ES; ++i) scratchpad[i] &= memory[position + i];
                }
            }
            break;

        case 0xAA:      // READ SCRATCHPAD COMMAND

            if (hub->send(reinterpret_cast<uint8_t *>(&reg_TA),2,crc))  return;
            if (hub->send(&reg_ES,1,crc)) return;

            {   // send Scratchpad content
                const uint8_t start  = uint8_t(reg_TA) & SCRATCHPAD_MASK;
                const uint8_t length = (reg_ES & SCRATCHPAD_MASK)+ uint8_t(1) - start; // based on ds2413: difference to ds2433
                if (hub->send(&scratchpad[start],length,crc))   return;
            }

            crc = ~crc;
            if (hub->send(reinterpret_cast<uint8_t *>(&crc),2)) return;
            break; // send 1s when read is complete, is passive, so do nothing

        case 0x55:      // COPY SCRATCHPAD COMMAND

            if (hub->recv(&data))                                  return;
            if (data != reinterpret_cast<uint8_t *>(&reg_TA)[0])   break;
            if (hub->recv(&data))                                  return;
            if (data != reinterpret_cast<uint8_t *>(&reg_TA)[1])   break;
            if (hub->recv(&data))                                  return;
            if (data != reg_ES)                                    return; // Auth code must match

            if (getPageProtection(uint8_t(reg_TA)))                break; // stop if page is protected (WriteMemory also checks this)
            if ((reg_ES & REG_ES_PF_MASK) != 0)                    break; // stop if error occurred earlier

            reg_ES |= REG_ES_AA_MASK; // compare was successful

            reg_TA &= ~uint16_t(SCRATCHPAD_MASK);

            // Write Scratchpad to memory, writing takes about 10ms
            writeMemory(scratchpad, SCRATCHPAD_SIZE, reinterpret_cast<uint8_t *>(&reg_TA)[0]); // checks if copy protected

            noInterrupts();

            do
            {
                hub->clearError();

                hub->sendBit(true); // send passive 1s

            }
            while   (hub->getError() == Error::AWAIT_TIMESLOT_TIMEOUT_HIGH); // wait for timeslots

            interrupts();

            while (!hub->send(&ALTERNATING_10)); //  alternating 1 & 0 after copy is complete
            break;

        case 0xF0:      // READ MEMORY COMMAND

            if (hub->recv(reinterpret_cast<uint8_t *>(&reg_TA),2))  return;
            if (reg_TA >= MEM_SIZE) return;
            if (hub->send(&memory[reg_TA],MEM_SIZE - uint8_t(reg_TA),crc)) return;
            break; // send 1s when read is complete, is passive, so do nothing here
        */
        default:

            hub->raiseSlaveError(cmd);
    }
}

bool DS2406::setChipType(uint8_t value)
// Chip type = 0x00 for the DS2406+, 3 pin package TO-92 (No Vcc, only one switch: PIO-A) (default)
// Chip type = 0x01 for the DS2406P+, 6 pin package TSOC (Vcc Pin, two switches: PIO-A & PIO-B)
{
    if (value > 0x01) return false;
    
    chip_type = value;
    if (chip_type == 0)  channel_size = 1;
    else                 channel_size = 2;

    clearSupplyIndication();
    clearControl();
    clearInfo();

    return true;
}


// Memory Section

void DS2406::clearScratchpad(void) // copied from 
{
    memset(scratchpad, static_cast<uint8_t>(0x00), SCRATCHPAD_SIZE);    // ????
}

void DS2406::clearMemory(void) // copied from DS2506
{
    memset(memory, value_xFF, MEM_SIZE);
}

bool DS2406::writeMemory(const uint8_t* const source, const uint8_t length, const uint8_t position) // copied from DS2502/DS2506
{
    if (position >= MEM_SIZE) return false;
    const uint16_t _length = (position + length >= MEM_SIZE) ? (MEM_SIZE - position) : length;
    memcpy(&memory[position],source,_length);

    const uint8_t page_start = static_cast<uint8_t>(position >> 5);
    const uint8_t page_stop  = static_cast<uint8_t>((position + _length) >> 5);
    for (uint8_t page = page_start; page <= page_stop; page++) setPageUsed(page);

    return (_length==length);
}

bool DS2406::readMemory(uint8_t* const destination, const uint16_t length, const uint16_t position) const // copied from DS2431/DS2502/DS2506
{
    if (position >= MEM_SIZE) return false;
    const uint16_t _length = (position + length >= MEM_SIZE) ? (MEM_SIZE - position) : length;
    memcpy(destination,&memory[position],_length);
    return (_length==length);
}

// Status Section
void DS2406::clearStatus(void) // copied from DS2506
{
    //memset(status, value_xFF, STATUS_SIZE);
    status[STATUS_WP_PAGES]   = value_xFF;
    status[STATUS_PG_REDIR]   = value_xFF;
    status[STATUS_PG_REDIR+1] = value_xFF;
    status[STATUS_PG_REDIR+2] = value_xFF;
    status[STATUS_PG_REDIR+3] = value_xFF;
    status[STATUS_UNDEF_B1]   = value_x00;
    status[STATUS_UNDEF_B1+1] = value_x00;
    status[STATUS_DEVICE]     = value_xFF;

    // clear supply voltage indicator (for all chips) at STATUS_DEVICE section
    clearSupplyIndication();

}

uint8_t DS2406::writeStatus(const uint16_t address, const uint8_t value) // copied from DS2506
{
    if (address < STATUS_UNDEF_B1)  status[address] &= value; // writing is allowed only here
    return status[address];
}

uint8_t DS2406::readStatus(const uint16_t address) const // copied from DS2506
{
    if (address >= STATUS_SIZE)     return 0xFF;
    return status[address];
}

void DS2406::setPageProtection(const uint8_t page) // copied from DS2502
{
    if (page < PAGE_COUNT)          status[STATUS_WP_PAGES] &= ~(uint8_t(1<<page));
}

bool DS2406::getPageProtection(const uint8_t page) const // copied from DS2502
{
    if (page >= PAGE_COUNT) return true;
    return ((status[STATUS_WP_PAGES] & uint8_t(1<<page)) == 0);
}

void DS2406::setPageUsed(const uint8_t page) // copied from DS2502
{
    if (page < PAGE_COUNT)  status[STATUS_WP_PAGES] &= ~(uint8_t(1<<(page+4)));
}

bool DS2406::getPageUsed(const uint8_t page) const // copied from DS2502
{
    if (page >= PAGE_COUNT) return true;
    return ((status[STATUS_WP_PAGES] & uint8_t(1<<(page+4))) == 0);
}

bool DS2406::setPageRedirection(const uint8_t page_source, const uint8_t page_destin) // copied from DS2502
{
    if (page_source >= PAGE_COUNT)  return false; // really available
    if (page_destin >= PAGE_COUNT)  return false; // virtual mem of the device

    status[page_source + STATUS_PG_REDIR] = (page_destin == page_source) ? uint8_t(0xFF) : ~page_destin; // datasheet dictates this, so no page can be redirected to page 0
    
    return true;
}

uint8_t DS2406::getPageRedirection(const uint8_t page) const // copied from DS2502
{
    if (page >= PAGE_COUNT) return 0x00;
    return ~(status[page + STATUS_PG_REDIR]); // TODO: maybe invert this in ReadStatus and safe some Operations? Redirection is critical and often done
}

// copied from DS2502 and DS2506
uint8_t DS2406::translateRedirection(const uint8_t source_address) const // TODO: extended read mem description implies that redirection is recursive
{
    const uint8_t  source_page    = static_cast<uint8_t >(source_address >> 5);
    const uint8_t  destin_page    = getPageRedirection(source_page);
    if (destin_page == 0x00)        return source_address;
    const uint8_t destin_address  = (source_address & PAGE_MASK) | (destin_page << 5);
    return destin_address;
}

// copied from DS2408 and changed
void DS2406::setPinState(const uint8_t pinNumber, const bool value)
{
    uint8_t pio_state = (info[INFO] & 0x0C) >> 2;

    if (pinNumber >= channel_size) return false;

    if(value)   pio_state |= 1 << pinNumber;
    else        pio_state &= ~(1 << pinNumber);

    // look for changes in the activity latches
    info[INFO] |= (pio_state ^ getPinState()) << 4; // activity
    info[INFO] = pio_state << 2;                    // sensed level
    info[INFO] = pio_state;                         // output
}

// copied from DS2408 and changed
bool DS2406::getPinState(const uint8_t pinNumber) const
{
    if (pinNumber >= channel_size) return false;
    
    return static_cast<bool>(info[INFO] & ( 1 << (pinNumber+2) ));
}

// copied from DS2408 and changed
uint8_t DS2406::getPinState(void) const
{
    return (info[INFO] & 0x0C) >> 2;
}

// copied from DS2408 and changed
bool DS2406::setPinActivity(const uint8_t pinNumber, const bool value)
{
    
    if (pinNumber >= channel_size) return false;

    if (value)  info[INFO] |=  (1 << (pinNumber+4) );
    else        info[INFO] &= ~(1 << (pinNumber+4) );

    return true;
}

// bool DS2406::setPinActivity(const uint8_t value)
// {
    
//     if (value > 0x03) return false;

//     info[INFO] = (value & 0x03) << 4 ;

//     return true;
// }

bool DS2406::clearPinActivity(const uint8_t pinNumber)
{

    if (pinNumber >= channel_size) return false;

    info[INFO] &= ~(1 << (pinNumber+4) );

    return true;
}

bool DS2406::clearPinActivity(void)
{

    info[INFO] &= ~(0x03 << 4 );

    return true;
}


// copied from DS2408 and changed
bool DS2406::getPinActivity(const uint8_t pinNumber) const
{
    if (pinNumber >= channel_size) return false;
    
    return static_cast<bool>(info[INFO] & ( 1 << (pinNumber+4) ));
}

// copied from DS2408 and changed
uint8_t DS2406::getPinActivity(void) const
{
    return (info[INFO] & 0x30) >> 4;
}

// Supply Indictaion Section
void DS2406::setSupplyIndication(bool value)
{
    // in case of DS2406P+ chip type it is possible to set supply voltage
    if(chip_type == 0x01) {
        if(value)   status[STATUS_DEVICE] |= 1 << 7;
        else        status[STATUS_DEVICE] &= ~(1 << 7);
    }
}

void DS2406::clearSupplyIndication(void)
{
    status[STATUS_DEVICE] &= ~(1 << 7);
}

uint8_t DS2406::getSupplyIndication(void) const
{
    return static_cast<bool>(status[STATUS_DEVICE] & ( 0x80 ) >> 7);
}


// Channel Control Section

void DS2406::writeControl(const uint8_t value)
{
    control[CONTROL_1] = value;

    // is ALR bit set: 
    if (value & 0x80) {
        // clear pin activity
        clearPinActivity(0x00);
        clearPinActivity(0x01);
    }
}

void DS2406::clearControl(void)
{
    // in case of DS2406+ chip type:
    if(chip_type == 0x00) {
        // clear IC bit
        control[CONTROL_1] &= ~(1 << 4);
    }
}

uint8_t DS2406::readControl(void) const
{
    return control[CONTROL_1];
}

// Channel Info Section

void DS2406::writeInfo(const uint8_t value)
{
    info[INFO] = value;
}

void DS2406::clearInfo(void)
{
    // in case of DS2406+ chip type:
    if(chip_type == 0x00) {
        // clear number of channels bit
        info[INFO] &= ~(1 << 6);
        // clear supply indication
        info[INFO] &= ~(1 << 6);
    }
}

uint8_t DS2406::readInfo(void) const
{
    return info[INFO];
}