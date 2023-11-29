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
        if(state == 0){
            data = Serial.read();
            if(data != 0){
                state = 2;
                Serial.write(data+1);
            }
        }else if(state == 2){
            data = Serial.read();
            
            if(data == 240){ // 0xF0
                // Read Momory
                uint8_t readmemory[4*16];
                //bool tmp = m_chip->readMemory(readmemory, 128,0);
                for(int i = 0; i < 144; i++){
                    m_chip->getMemory(i);
                }
                state = 0;
            }else if(data == 175){  // 0xAF to Clear Memory
                // Clear Memory
                m_chip->clearMemory();
                Serial.write(170);
                state = 0;
            }else if(data == 20){
                // return the status info byte
                Serial.write(m_chip->readInfo());
                state = 0;
            }else if(data == 30){
                // PIOA auf 0 überprüfen 
                if((m_chip->readInfo() & 0x10) > 0){
                    Serial.write(55);
                }else{
                    Serial.write(110);
                }
                state = 0;
            }else if(data == 40){
                // PIOB auf 0 überprüfen
                if((m_chip->readInfo() & 0x20) > 0){
                    Serial.write(55);
                }else{
                    Serial.write(110);
                }
                state = 0;
            }else{
                Serial.write(0);
            }
            Serial.print(10);
            
        }
    }

}

OneWireCom::~OneWireCom()
{
}
