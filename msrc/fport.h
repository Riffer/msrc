#ifndef FPORT_H
#define FPORT_H

#include <Arduino.h>
#include "softserial.h"
#include "hardserial.h"
#include "sensor.h"
#include "constants.h"

#include "escHW3.h"
#include "escHW4.h"
#include "escPWM.h"
#include "escCastle.h"
#include "escKontronik.h"
#include "escApdF.h"
#include "escApdHV.h"
#include "voltage.h"
#include "current.h"
#include "ntc.h"
#include "pressure.h"
#include "bmp280.h"
#include "ms5611.h"
#include "bn220.h"
#include "configeeprom.h"
#include "pwmout.h"
#include "smartport.h"

#define FPORT_WAIT 0
#define FPORT_SEND 1

#define FPORT_TIMEOUT 500
#define FPORT_PACKET_LENGHT 12

class FPort : public Smartport
{
private:

public:
    FPort(AbstractSerial &serial);
    ~FPort();
    void begin();
    void updateFPort();
    void sendPacket(uint8_t packetId);
};

#endif