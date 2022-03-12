#ifndef CRFS_H
#define CRFS_H

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

#define CRFS_PACKET_LENGTH 26
#define CRFS_TIMEOUT 500


class Crfs
{
private:
    AbstractSerial &serial_;
    bool isGpsEnabled = false;
    bool isBatteryEnabled = false;

public:
    Crfs(AbstractSerial &serial);
    ~Crfs();
    void begin();
    void update();
    void setConfig(Config &config);
    void sendPacket(uint8_t packetId);
    uint8_t crc(uint8_t *crc, uint8_t crc_lenght);
    uint8_t update_crc(uint8_t crc, uint8_t crc_seed);
};

#endif