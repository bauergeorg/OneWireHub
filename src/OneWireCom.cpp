#include "OneWireCom.h"


OneWireCom::OneWireCom(OneWireItem& ds24,  HardwareSerial& serial)
{
        m_chip= &ds24;
        m_serial=&serial;
}

void OneWireCom::communicate()
{
    if(m_serial->available()>0){

        const int maxLength=500;
        byte inputbuffer[maxLength];

        int readlength = m_serial->readBytesUntil('\n', inputbuffer, maxLength - 1);    //readBytesUntil(...) Returns the number of characters to be read including spaces
        inputbuffer[readlength] = '\0';

        int cmdLength=((readlength+12)/3);

        uint8_t cmdbuffer[cmdLength];
        char* token;
        token = strtok(inputbuffer, " ");
        for (int i = 0; i < readlength; i++) {
          cmdbuffer[i] = strtoul(token, NULL, 16);
          token = strtok(NULL, " ");
        }
        // m_serial->print("command: ");
        //
        // for (int j=0; j < cmdLength; j++){
        //
        //     if (cmdbuffer[i]<0x10) {
        //       m_serial->print("0");
        //     }
        //     m_serial->print(cmdbuffer[j], HEX);
        //     m_serial->print(" ");
        //
        // }
        // m_serial->println();

        if (cmdbuffer[0]==00) {

              // m_serial->print("write command received\n");

              const uint8_t pos=cmdbuffer[1];
              // m_serial->print("Start position: ");
              // m_serial->println(pos,HEX);
              const uint8_t length=cmdbuffer[2];
              // m_serial->print("length: ");
              // m_serial->println(length,HEX);
              uint8_t memory[length];
              m_serial->print("content: ");
              for(int i = 0; i < length ; i++){
                  memory[i]=cmdbuffer[i+3];
                  m_serial->print( memory[i], HEX);
                  m_serial->print(" ");

              }
              m_serial->println();

              // m_serial->print("write menory...\n");
              m_chip->writeMemory(memory,length,pos);
              // m_serial->print("done!\n");

              // uint8_t mem[length];
              // int compareCount=0;
              // m_chip->readMemory(mem,length,pos);
              // for (size_t i = 0; i < length; i++) {
              //   if (mem[i]==memory[i]) {
              //     compareCount++;
              //   }
              // }
              // if (compareCount==length) {
              //   m_serial->print("done!\n");
              // }
              // else{
              //   m_serial->print("something went wrong while writing, please check the command!\n");
              // }
        }

        else if (cmdbuffer[0]==01) {

              // m_serial->print("read command received\n");
              uint8_t pos=cmdbuffer[1];
              // m_serial->print("Start position: ");
              // m_serial->println(pos, HEX);
              uint8_t length=cmdbuffer[2];
              // m_serial->print("length: ");
              // m_serial->println(length, HEX);
              uint8_t readmemory[length];

              // m_serial->print("read menory...\n");
              m_chip->readMemory(readmemory, length, pos);

              // int count=0;
              for (int j=0; j<length; j++){

                  if (readmemory[j]<0x10) {
                    m_serial->print("0");
                  }
                  m_serial->print(readmemory[j], HEX);
                  m_serial->print(" ");
                    //  if ((len>0) & ((len%32)==0)) {m_serial->print("\n");}
                  // count++;
                  }
              m_serial->println();
              // m_serial->print("done!\n");

              // if (count==length) {
              //   m_serial->print("done!\n");
              // }
              // else {
              //     m_serial->print("something went wrong while reading, please check the command!\n");
              // }


        }
        else if (cmdbuffer[0]==02) {
              //
              // m_serial->print("clear command received\n");
              // m_serial->print("clear menory...\n");
              m_chip->clearMemory();
              m_serial->print("success");
        }

        else{
              m_serial->print("the command given is incorrect!\n");
              m_serial->print("*To read the memory content, please enter the command '01', followed by the start position and the length of the content to be read, separated by spaces.\n");
              m_serial->print("*To write the memory, please enter the command '00', followed by the start position, the length and the content you want to write, separated by spaces.\n");
              m_serial->print("*To clear the memory, please enter '02'..\n");

              m_serial->print("command: ");
             for (int i = 0; i < cmdLength; i++) {
                if(cmdbuffer[i]<0x10) m_serial->print("0");
                m_serial->print(cmdbuffer[i],HEX);
                m_serial->print(" ");
              }
            m_serial->println();
        }

  }

}

OneWireCom::~OneWireCom()
{
}
