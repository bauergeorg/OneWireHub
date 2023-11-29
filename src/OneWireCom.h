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
    Stream* m_serial;
    int state;

    
public:
    OneWireCom(OneWireItem& ds24);
    OneWireCom(OneWireItem& ds24,  Stream& serial, int x);
    OneWireCom(OneWireItem& ds24,  Stream& serial);
    
    
    void communicate();
    ~OneWireCom();

};
#endif
