#ifndef HOTT_H
#define HOTT_H

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

#define HOTT_START_BYTE 0x7C
#define HOTT_STOP_BYTE 0x7D

#define HOTT_GAM_ID 0x8D
#define HOTT_VARIO_ID 0x89
#define HOTT_GPS_ID 0x8A
#define HOTT_ESC_ID 0X8C

#define HOTT_TEXT_REQUEST_ID 0x7F
#define HOTT_BINARY_REQUEST_ID 0x80

#define HOTT_PACKET_LENGTH 2
#define HOTT_TIMEOUT 1000

struct HottGam
{
    float *voltage1P = NULL;
    float *voltage2P = NULL;
    float *temperature1P = NULL;
    float *temperature2P = NULL;
    float *currentP = NULL;
    float *consumptionP = NULL;
    float *speedP = NULL;
    float *altP = NULL;
    float *varioP = NULL;
};

struct HottGps
{
    float *cogP = NULL;
    float *spdP = NULL;
    float *latP = NULL;
    float *lonP = NULL;
    float *distP = NULL;
    float *altP = NULL;
    float *varioP = NULL;
    float *satP = NULL;
    float *timeP = NULL;
};

struct HottEsc
{
    float *consumptionP = NULL;
    float *tempP = NULL;
    float *rpmP = NULL;
    float *spdP = NULL;
    float *voltageP = NULL;
    float *currentP = NULL;
    float *becVoltageP = NULL;
    float *becCurrentP = NULL;
    float *tempBecP = NULL;
    float *motorTempP = NULL;
};

class Hott
{
private:
    struct DeviceElement
    {
        AbstractDevice *deviceP;
        DeviceElement *nextP;
    };
    DeviceElement *deviceElementP = NULL;
    AbstractSerial &serial_;
    bool isGamEnabled = false;
    bool isGpsEnabled = false;
    bool isEscEnabled = false;
    bool isVarioEnabled = false;
    HottGam hottGam;
    HottGps hottGps;
    HottEsc hottEsc;
    void setConfig(Config &config);
    void sendGam();
    void sendGps();
    void sendEsc();
    void write8(uint8_t value, uint8_t &crc);
    void write16(uint16_t value, uint8_t &crc);
    void writeCrc(uint8_t value);
    void addDevice(AbstractDevice *deviceP);

public:
    Hott(AbstractSerial &serial);
    ~Hott();
    void begin();
    void update();
};

#endif