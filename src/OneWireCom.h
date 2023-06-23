#ifndef ONE_WIRE_COM_H
#define ONE_WIRE_COM_H

#include "DS2431.h"
#include "DS2406.h"
#include "OneWireItem.h"
#include <Arduino.h>
using namespace std;


class OneWireCom
{
private:

    OneWireItem* m_chip{nullptr};
    HardwareSerial* m_serial;

public:
    OneWireCom(OneWireItem& ds24,  HardwareSerial& serial);
    void communicate();
    ~OneWireCom();

};
#endif
