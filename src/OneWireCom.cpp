#include "OneWireCom.h"


OneWireCom::OneWireCom(OneWireItem& ds24,  Stream& serial)
{
        m_chip= &ds24;
        m_serial=&serial;
        state = 0;
}


void OneWireCom::communicate()
{
    
    if(m_serial->available()>0){  
        int data = 0;
        int start = 0;
        int length = 0;
        if(state == 0){
            data = Serial.read();
            if(data != 0){
                state = 2;
                Serial.write(data+1);
            }
        }else if(state == 2){
            data = Serial.read();
            start = Serial.read();
            length = Serial.read();
            if((start+length)>128){
                Serial.write(110);
            }
            if(data == 240){ // 0xF0
                // Read Momory
                uint8_t readmemory[4*16];
                for(int i = start; i < (start+length); i++){
                    m_chip->getMemory(i);
                }
                state = 0;
            }else if(data == 250){  
                // DS2431 Read Memory
                uint8_t readmemory[4*16];
                bool tmp = m_chip->readMemory(readmemory, 128,0);
                if(tmp == true){
                    Serial.write(readmemory, 144);
                }
                state = 0;
            }else if(data == 175){  // 0xAF to Clear Memory
                // Clear Memory
                m_chip->clearMemory();
                //m_chip->getMemory(0);
                Serial.write(170);
                state = 0;
            }else if(data == 180){  // 0xAF to Clear Memory
                // Clear Memory
                m_chip->clearMem();
                Serial.write(170);
                state = 0;
            }else if(data == 100){
                // send info byte
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 200){
                // set activity in info byte
                uint8_t newInfoValue = 0b01111111;
                m_chip->writeInfo(newInfoValue);
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 125){
                // set activity in info byte
                Serial.write(m_chip->readStatus(0x07));
                state = 0;
            }else if(data == 130){
                // set PIOA State
                if(start == 1){
                    // set state of PIN A is 1
                    m_chip->setPinState(0,true,1);
                }else{                    
                    // set state of PIN A is 0
                    m_chip->setPinState(0,false,1);
                }
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 140){
                // set PIOA State
                if(start == 1){
                    // set state of PIN A is 1
                    m_chip->setPinState(1,true,1);
                }else{                    
                    // set state of PIN A is 0
                    m_chip->setPinState(1,false,1);
                }
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 150){
                // set PIOA Level
                if(start == 1){
                    // set level of PIN A is 1
                    m_chip->setPinLevel(0,false);
                }else{                    
                    // set level of PIN A is 0
                    m_chip->setPinLevel(0,true);
                }
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 160){
                // set PIOA Level
                if(start == 1){
                    // set level of PIN B is 1
                    m_chip->setPinLevel(1,true);
                }else{                    
                    // set level of PIN B is 1
                    m_chip->setPinLevel(1,false);
                }
                Serial.write(m_chip->readInfo());
                state = 0;
            }else{
                Serial.write(255);
                Serial.println("error: command not recongnized");
                state = 0;
            }
        }
        
    }


}
OneWireCom::~OneWireCom()
{
}
