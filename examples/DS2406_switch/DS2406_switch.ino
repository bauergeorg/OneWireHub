/*
 * Example code that emulates an One-Wire Dual Switch with 1K-bit memory (DS2406)
 * 
 * Tested with the Maxim DS9481R-3C7+ USB-to-1-Wire adapter and OneWireViewer
 *
 * Used Library: OneWireHub
 * - the original OneWireHub library does not support the DS2406 chip
 *  to emulate the DS2406 with the OneWire library, the original repository https://github.com/orgua/OneWireHub was used 
 *  to https://github.com/bauergeorg/OneWireHub and the folder src was extended with the files DS2406.h and DS2406.cpp extended.
 *  + the original OneWireHub can be downloaded in the library manager
 *  + the extended OneWireHub library (with support for DS2406) can be used by creating the repository 
 *   https://github.com/bauergeorg/OneWireHub clone, and saving the folders in the [libraries] folder of Arduino.     
 *
 * Usage:
 *  Connection of the DS2406_Slave (arduino nano) to the master device:
 *  - Connect OneWire pin from arduino nano (can be changed in code below, here pin <2>) with master device.
 *  - Connect ground of your arduino with ground of master device.
 * 
 * Description:
 *  The following code simulates a DS2406 chip as OneWire slave and writes data into the main memory. 
 * 
 * PIOs: 
 *  You can select bewtween DS2406+ (TO-92 package) and DS2406P+ (TSOC package). Check out function 'ds2406.setChipType(xyz)':
 *   0x00 for the DS2406+, 3 pin package TO-92 (No Vcc, only one switch: PIO-A) (default)
 *   0x01 for the DS2406P+, 6 pin package TSOC (Vcc Pin, two switches: PIO-A & PIO-B)
 * 
 *  You have the option to simulate a pull-up resistor at the output of the PIO's. Pleas check out the pull-up funktion.
 *
 */

#include <OneWireHub.h>
#include <DS2406.h>

constexpr uint8_t pin_onewire  { 2 };

auto hub = OneWireHub(pin_onewire);
auto ds2406 = DS2406(DS2406::family_code, 0xD5, 0x59, 0xE9, 0x00, 0x00, 0x00);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("OneWire-Hub DS2406");
  Serial.println("Write Hex Data to the first DS2406 (DS2406_1)");

  Serial.println("Write Hex Data to page 0");
  constexpr uint8_t memory[] = {0xD1, 0xA8, 0x42, 0xB6, 0xC7, 0x70, 0x58, 0x0A, 0xDA, 0xE2, 0x7E, 0x12, 0x83, 0x86, 0x9A, 0x62,
                                0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x00, 0x00, 0x44, 0x65, 0x66, 0x61, 0x75, 0x6C, 0x74, 0x5F };
  ds2406.writeMemory(memory,sizeof(memory),0x00);

  Serial.println("Write Hex Data to page 1");
  constexpr uint8_t mem_1[] = {0x52, 0x6F, 0x74, 0x6F, 0x72, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x0D,
                               0x44, 0x65, 0x66, 0x61, 0x75, 0x6C, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
  ds2406.writeMemory(mem_1, sizeof(mem_1), 1*32);

  Serial.println("Write Hex Data to page 2");
  constexpr uint8_t mem_2[] = {0x44, 0x65, 0x66, 0x61, 0x75, 0x6C, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x88, 0x13, 0x10, 0x27, 0xE8, 0x03, 0x64, 0x00, 0xC8, 0x00};
  ds2406.writeMemory(mem_2, sizeof(mem_2), 2*32);

  Serial.println("Write Hex Data to page 3");
  constexpr uint8_t mem_3[] = { 0x0A, 0x00, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF,
                                0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  ds2406.writeMemory(mem_3, sizeof(mem_3), 3*32);  

  // DS2406.clearMemory(); // begin fresh after doing some work

  // Setup OneWire
  hub.attach(ds2406);
  Serial.println("config done");

  ds2406.setChipType(0x01);
  ds2406.setSupplyIndication(true);

  // something to play with
  // ds2406.setPinState(0, true);
  // ds2406.setPinActivity(0, true);
  // ds2406.clearPinActivity();
  // ds2406.setPinState(0, false);
  // ds2406.setPullUpResistor(false);

}

void loop() {
  // put your main code here, to run repeatedly:
   hub.poll();
}
