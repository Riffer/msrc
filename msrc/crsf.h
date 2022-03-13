#ifndef CRSF_H
#define CRSF_H

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

#define CRSF_FRAMETYPE_GPS 0x02
#define CRSF_FRAMETYPE_BATTERY_SENSOR 0x08
#define CRSF_FRAMETYPE_VARIO_SENSOR 0x07

#define CRSF_ADDRESS_BROADCAST 0x00
#define CRSF_ADDRESS_CURRENT_SENSOR 0xC0
#define CRSF_ADDRESS_GPS 0xC2
#define CRSF_ADDRESS_VARIO 0xC2

#define CRSF_FRAME_GPS_PAYLOAD_SIZE 15
#define CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE 8
#define CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE 2
#define CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE 22 // 11 bits per channel * 16 channels = 22 bytes.

#define CRSF_FRAME_LENGTH_ADDRESS 1      // length of ADDRESS field
#define CRSF_FRAME_LENGTH_FRAMELENGTH 1  // length of FRAMELENGTH field
#define CRSF_FRAME_LENGTH_TYPE 1         // length of TYPE field
#define CRSF_FRAME_LENGTH_CRC 1          // length of CRC field
#define CRSF_FRAME_LENGTH_TYPE_CRC 2,    // length of TYPE and CRC fields combined
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC 4 // length of Extended Dest/Origin, TYPE and CRC fields combined
#define CRSF_FRAME_LENGTH_NON_PAYLOAD 4  // combined length of all fields except payload

#define CRSF_PACKET_LENGTH 26
#define CRSF_TIMEOUT 500

#define CRSF_WAIT 0
#define CRSF_SEND 1

struct CsrfGps
{
    AbstractDevice *deviceP = NULL;
    float *latP;
    float *lonP;
    float *spdP;
    float *cogP;
    float *altP;
    float *satP;
};

struct CsrfBattery
{
    AbstractDevice *deviceVoltageP = NULL;
    AbstractDevice *deviceCurrentP = NULL;
    float *voltageP;
    float *currentP;
    float *consumptionP;
};

struct CsrfVario
{
    AbstractDevice *deviceP = NULL;
    float *vSpdP;
};

class Crsf
{
private:
    AbstractSerial &serial_;
    bool isGpsEnabled = false;
    bool isBatteryEnabled = false;
    bool isVarioEnabled = false;
    CsrfGps csrfGps;
    CsrfBattery csrfBattery;
    CsrfVario csrfVario;
    void setConfig(Config &config);
    void sendPacket();
    void sendGps();
    void sendBattery();
    void sendVario();
    uint8_t *sendU8(uint8_t value, uint8_t *buffer);
    uint8_t *sendU16(uint16_t value, uint8_t *buffer);
    uint8_t *sendS16(int16_t value, uint8_t *buffer);
    uint8_t *sendU24(uint32_t value, uint8_t *buffer);
    uint8_t *sendS32(int32_t value, uint8_t *buffer);
    uint8_t getCrc(uint8_t *buffer, uint8_t size);
    uint8_t update_crc(uint8_t crc, uint8_t crc_seed);

public:
    Crsf(AbstractSerial &serial);
    ~Crsf();
    void begin();
    void update();
};

#endif