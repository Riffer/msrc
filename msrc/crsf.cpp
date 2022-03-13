#include "crsf.h"

Crsf::Crsf(AbstractSerial &serial) : serial_(serial)
{
}

Crsf::~Crsf()
{
}

void Crsf::begin()
{
    serial_.begin(420000, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(CRSF_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

uint8_t *Crsf::sendU8(uint8_t value, uint8_t *buffer)
{
    *buffer = value;
    return buffer + 1;
}

uint8_t *Crsf::sendU16(uint16_t value, uint8_t *buffer)
{
    uint16_t swapped = __builtin_bswap16(value);
    memcpy(buffer, &swapped, 2);
    return buffer + 2;
}

uint8_t *Crsf::sendS16(int16_t value, uint8_t *buffer)
{
    int16_t swapped = __builtin_bswap16(value);
    memcpy(buffer, &swapped, 2);
    return buffer + 2;
}

uint8_t *Crsf::sendU24(uint32_t value, uint8_t *buffer)
{
    uint32_t swapped = __builtin_bswap32(value << 8);
    memcpy(buffer, &swapped, 3);
    return buffer + 3;
}

uint8_t *Crsf::sendS32(int32_t value, uint8_t *buffer)
{
    int32_t swapped = __builtin_bswap32(value);
    memcpy(buffer, &swapped, 4);
    return buffer + 4;
}

void Crsf::sendGps()
{
    uint8_t buffer[CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE];
    uint8_t *bufferP = &buffer[0];
    bufferP = sendU8(CRSF_ADDRESS_GPS, bufferP);
    bufferP = sendU8(CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE, bufferP);
    bufferP = sendU8(CRSF_FRAMETYPE_GPS, bufferP);
    bufferP = sendS32(*csrfGps.latP / 60 * 1E7, bufferP);
    bufferP = sendS32(*csrfGps.lonP / 60 * 1E7, bufferP);
    bufferP = sendU16(*csrfGps.spdP * 1.852 * 10, bufferP);
    bufferP = sendU16(*csrfGps.cogP * 100, bufferP);
    bufferP = sendU16(*csrfGps.altP + 1000, bufferP);
    bufferP = sendU8(*csrfGps.satP, bufferP);
    uint8_t crcValue = getCrc(buffer + 2, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE - 3);
    sendU8(crcValue, bufferP);
    serial_.writeBytes(buffer, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE);
#ifdef DEBUG
    DEBUG_PRINT(">");
    for (uint8_t i = 0; i < CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_GPS_PAYLOAD_SIZE; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
#endif
}

void Crsf::sendBattery()
{
    uint8_t buffer[CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE];
    uint8_t *bufferP = &buffer[0];
    bufferP = sendU8(CRSF_ADDRESS_CURRENT_SENSOR, bufferP);
    bufferP = sendU8(CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE, bufferP);
    bufferP = sendU8(CRSF_FRAMETYPE_BATTERY_SENSOR, bufferP);
    bufferP = sendU16(*csrfBattery.voltageP * 100, bufferP);
    bufferP = sendU16(*csrfBattery.currentP * 100, bufferP);
    bufferP = sendU24(*csrfBattery.consumptionP, bufferP);
    bufferP = sendU8(0, bufferP);
    uint8_t crcValue = getCrc(buffer + 2, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE - 3);
    sendU8(crcValue, bufferP);
    serial_.writeBytes(buffer, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE);
#ifdef DEBUG
    DEBUG_PRINT(">");
    for (uint8_t i = 0; i < CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
#endif
}

void Crsf::sendVario()
{
    uint8_t buffer[CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE];
    uint8_t *bufferP = &buffer[0];
    bufferP = sendU8(CRSF_ADDRESS_VARIO, bufferP);
    bufferP = sendU8(CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE, bufferP);
    bufferP = sendU8(CRSF_FRAMETYPE_VARIO_SENSOR, bufferP);
    bufferP = sendS16(*csrfVario.vSpdP * 10, bufferP);
    uint8_t crcValue = getCrc(buffer + 2, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE - 3);
    sendU8(crcValue, bufferP);
    serial_.writeBytes(buffer, CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE);
#ifdef DEBUG
    DEBUG_PRINT(">");
    for (uint8_t i = 0; i < CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_VARIO_SENSOR_PAYLOAD_SIZE; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
#endif
}

void Crsf::sendPacket()
{
    static uint8_t packetCount = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    if (isGpsEnabled && packetCount % 3 == 0)
        sendGps();
    else if (isBatteryEnabled && packetCount % 3 == 1)
        sendBattery();
    else if (isVarioEnabled && packetCount % 3 == 2)
        sendVario();
    digitalWrite(LED_BUILTIN, LOW);
    packetCount++;
}

void Crsf::update()
{
    uint8_t status = CRSF_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)(millis() - ts) > 500)
    {
        if (!mute)
        {
            status = CRSF_SEND;
        }
        mute = !mute;
        ts = millis();
#else
    uint8_t length = serial_.availableTimeout();
    if (length)
    {
        uint8_t buff[length];
        serial_.readBytes(buff, length);
#ifdef DEBUG_PACKET
        DEBUG_PRINT("<");
        for (uint8_t i = 0; i < length; i++)
        {
            DEBUG_PRINT_HEX(buff[i]);
            DEBUG_PRINT(" ");
            delayMicroseconds(100);
        }
        DEBUG_PRINTLN();
#endif
        if (length == CRSF_FRAME_LENGTH_NON_PAYLOAD + CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE)
        {
            if (!mute)
            {
                status = CRSF_SEND;
            }
            mute = !mute;
        }
#endif
    }
    if (status == CRSF_SEND)
    {
        if (1) //(serial_.timestamp() < 1500)
            sendPacket();
#ifdef DEBUG
        else
        {
            DEBUG_PRINT("KO");
            DEBUG_PRINTLN();
        }
#endif
    }
    

    static uint8_t sensorCont = 0;
    if (isGpsEnabled && sensorCont % 3 == 0)
        csrfGps.deviceP->update();
    if (isBatteryEnabled && sensorCont % 3 == 1)
    {
        if (csrfBattery.deviceVoltageP) csrfBattery.deviceVoltageP->update();
        if (csrfBattery.deviceCurrentP) csrfBattery.deviceCurrentP->update();
    }
    if (isVarioEnabled && sensorCont % 3 == 2)
        csrfVario.deviceP->update();
    sensorCont++;
}

void Crsf::setConfig(Config &config)
{
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        csrfBattery.deviceVoltageP = esc;
        csrfBattery.deviceCurrentP = esc;
        csrfBattery.voltageP = esc->voltageP();
        csrfBattery.currentP = esc->currentP();
        csrfBattery.consumptionP = esc->consumptionP();
        isBatteryEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        csrfBattery.deviceVoltageP = esc;
        csrfBattery.deviceCurrentP = esc;
        csrfBattery.voltageP = esc->voltageP();
        csrfBattery.currentP = esc->currentP();
        csrfBattery.consumptionP = esc->consumptionP();
        isBatteryEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        csrfBattery.deviceVoltageP = esc;
        csrfBattery.deviceCurrentP = esc;
        csrfBattery.voltageP = esc->voltageP();
        csrfBattery.currentP = esc->currentP();
        csrfBattery.consumptionP = esc->consumptionP();
        isBatteryEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        csrfBattery.deviceVoltageP = esc;
        csrfBattery.deviceCurrentP = esc;
        csrfBattery.voltageP = esc->voltageP();
        csrfBattery.currentP = esc->currentP();
        csrfBattery.consumptionP = esc->consumptionP();
        isBatteryEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        csrfBattery.deviceVoltageP = esc;
        csrfBattery.deviceCurrentP = esc;
        csrfBattery.voltageP = esc->voltageP();
        csrfBattery.currentP = esc->currentP();
        csrfBattery.consumptionP = esc->consumptionP();
        isBatteryEnabled = true;
    }
    if (config.gps == true)
    {
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        csrfGps.deviceP = gps;
        csrfGps.latP = gps->latP();
        csrfGps.lonP = gps->lonP();
        csrfGps.spdP = gps->spdP();
        csrfGps.cogP = gps->cogP();
        csrfGps.altP = gps->altP();
        csrfGps.satP = gps->satP();
        isGpsEnabled = true;
        csrfVario.deviceP = gps;
        csrfVario.vSpdP = gps->varioP();
        isVarioEnabled = true;
    }
    if (config.voltage1 == true)
    {
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        csrfBattery.deviceVoltageP = voltage;
        csrfBattery.voltageP = voltage->valueP();
        isBatteryEnabled = true;
    }
    if (config.current == true)
    {
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        csrfBattery.deviceCurrentP = current;
        csrfBattery.currentP = current->valueP();
        csrfBattery.consumptionP = current->consumptionP();
        isBatteryEnabled = true;
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        csrfVario.deviceP = bmp;
        csrfVario.vSpdP = bmp->varioP();
        isVarioEnabled = true;
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        csrfVario.deviceP = bmp;
        csrfVario.vSpdP = bmp->varioP();
        isVarioEnabled = true;
    }
}

uint8_t Crsf::getCrc(uint8_t *buffer, uint8_t size)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < size; i++)
    {
        crc = update_crc(crc, *(buffer + i));
    }
    return crc;
}

uint8_t Crsf::update_crc(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii)
    {
        if (crc & 0x80)
            crc = (crc << 1) ^ 0xD5;
        else
            crc = crc << 1;
    }
    return crc;
}