#include "OneWireCom.h"


OneWireCom::OneWireCom(OneWireItem& ds24,  HardwareSerial& serial)
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
            if(data == 1){
                state = 1;
                Serial.write(data+1);
            }
        }else if(state == 1){
            data = Serial.read();
            data = data + 1;
            Serial.write(data);
            state = 2;
        }else if(state == 2){
            data = Serial.read();
            if(data == 240){ // 0xF0
                // Read Momory
                uint8_t readmemory[4*16];
                // m_serial->print("read menory...\n");
                bool tmp = m_chip->readMemory(readmemory, 128,0);
                for(int i = 0; i < 128; i++){
                    Serial.write(readmemory[i]);
                }
                // Vllt mehr mals einen leeren String 
                // und dann eine 55 schicken

                state = 0;
            }else if(data == 175){  // 0xAF to Clear Memory
                // Clear Memory
                if(data == 175){
                    m_chip->clearMemory();
                }
                Serial.write(170);
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
            }

            
        }
    }

}

OneWireCom::~OneWireCom()
{
}
