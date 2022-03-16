#include "hott.h"

Hott::Hott(AbstractSerial &serial) : serial_(serial)
{
}

Hott::~Hott()
{
}

void Hott::begin()
{
    serial_.begin(19200, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(HOTT_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Hott::writeCrc(uint8_t value)
{
    serial_.write(value);
}

void Hott::write8(uint8_t value, uint8_t &crc)
{
    crc += value;
    serial_.write(value);
#ifdef DEBUG
    DEBUG_PRINT_HEX(value);
    DEBUG_PRINT(" ");
#endif
}

void Hott::write16(uint16_t value, uint8_t &crc)
{
    write8(value, crc);
    write8(value >> 8, crc);
}

void Hott::sendGam()
{
    uint8_t crc = 0;
#ifdef DEBUG
    DEBUG_PRINT(">");
#endif
    write8(HOTT_START_BYTE, crc);
    write8(HOTT_GAM_ID, crc);
    write8(0, crc);
    write8(0xD0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write16(*hottGam.voltage1P * 10, crc);
    write16(*hottGam.voltage2P * 10, crc);
    write8(*hottGam.temperature1P + 20, crc);
    write8(*hottGam.temperature2P + 20, crc);
    write8(0, crc);
    write16(0, crc);
    write16(0, crc);
    write16(*hottGam.altP, crc);
    write16(*hottGam.varioP, crc);
    write8(0, crc);
    write16(*hottGam.currentP * 10, crc);
    write16(0, crc);
    write16(*hottGam.consumptionP / 10, crc);
    write16(*hottGam.speedP, crc);
    write8(0, crc);
    write8(0, crc);
    write16(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(HOTT_STOP_BYTE, crc);
    writeCrc(crc);
#ifdef DEBUG
    DEBUG_PRINTLN();
#endif
}

void Hott::sendGps()
{
    uint8_t crc = 0;
    static int16_t groundAlt = 0;
#ifdef DEBUG
    DEBUG_PRINT(">");
#endif
    write8(HOTT_START_BYTE, crc);
    write8(HOTT_GPS_ID, crc);
    write8(0, crc);
    write8(0xA0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(*hottGps.cogP / 2, crc);
    write16(*hottGps.spdP * 1.852, crc);
    uint8_t degrees, minutes, seconds, hours;
    float coordinate = *hottGps.latP;
    if (coordinate > 0)
        write8(0, crc);
    else
    {
        write8(1, crc);
        coordinate *= -1;
    }
    degrees = coordinate / 60;
    minutes = coordinate - degrees * 60;
    write16(degrees * 100 + minutes, crc);
    write16((coordinate - degrees * 60 - minutes) * 60 * 100, crc);
    coordinate = *hottGps.lonP;
    if (coordinate > 0)
        write8(0, crc);
    else
    {
        write8(1, crc);
        coordinate *= -1;
    }
    degrees = coordinate / 60;
    minutes = coordinate - degrees * 60;
    write16(degrees * 100 + minutes, crc);
    write16((coordinate - degrees * 60 - minutes) * 60 * 100, crc);
    write16(*hottGps.distP, crc);
    if (millis() > 5000 && groundAlt == 0)
        groundAlt = *hottGps.altP;
    write16(*hottGps.altP - groundAlt + 500, crc);
    write16((*hottGps.varioP + 300) * 100, crc);
    write8(0, crc);
    write8(*hottGps.satP, crc);
    write8(3, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    hours = *hottGps.timeP / 10000;
    minutes = (*hottGps.timeP - hours * 10000.0) / 100;
    seconds = *hottGps.timeP - hours * 10000.0 - minutes * 100;
    write8(hours, crc);
    write8(minutes, crc);
    write8(seconds, crc);
    write8(0, crc);
    write16(*hottGps.altP, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(HOTT_STOP_BYTE, crc);
    writeCrc(crc);
#ifdef DEBUG
    DEBUG_PRINTLN();
#endif
}

void Hott::sendEsc()
{
    static uint16_t voltMin = 0xFFFF;
    static uint16_t becVoltMin = 0xFFFF;
    static uint16_t tempMax = 0;
    static uint16_t becTempMax = 0;
    static uint16_t currMax = 0;
    static uint16_t rpmMax = 0;
    static uint16_t becCurrMax = 0;
    uint8_t crc = 0;
#ifdef DEBUG
    DEBUG_PRINT(">");
#endif
    write8(HOTT_START_BYTE, crc);
    write8(HOTT_ESC_ID, crc);
    write8(0, crc);
    write8(0xC0, crc);
    write8(0, crc);
    write8(0, crc);
    write16(*hottEsc.voltageP, crc);
    if (*hottEsc.voltageP < voltMin)
        voltMin = *hottEsc.voltageP;
    write16(voltMin, crc);
    write16(*hottEsc.consumptionP / 10, crc);
    write8(*hottEsc.tempP, crc);
    if (*hottEsc.tempP > tempMax)
        tempMax = *hottEsc.tempP;
    write8(tempMax, crc);
    write16(*hottEsc.currentP * 10, crc);
    if (*hottEsc.currentP * 10 > currMax)
        currMax = *hottEsc.currentP * 10;
    write16(currMax, crc);
    write16(*hottEsc.rpmP / 10, crc);
    if (*hottEsc.rpmP / 10 > rpmMax)
        rpmMax = *hottEsc.rpmP / 10;
    write16(rpmMax, crc);
    write8(0, crc);
    write16(0, crc);
    write16(0, crc);
    write8(*hottEsc.becVoltageP, crc);
    if (*hottEsc.becVoltageP < becVoltMin)
        becVoltMin = *hottEsc.becVoltageP;
    write8(becVoltMin, crc);
    write8(*hottEsc.becCurrentP, crc);
    if (*hottEsc.becCurrentP > becCurrMax)
        becCurrMax = *hottEsc.becCurrentP;
    write16(becCurrMax, crc);
    write8(0, crc);
    write8(*hottEsc.tempBecP, crc);
    if (*hottEsc.tempBecP > becTempMax)
        becTempMax = *hottEsc.tempP;
    write8(becTempMax, crc);
    write8(0, crc);
    write8(0, crc);
    write16(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(0, crc);
    write8(HOTT_STOP_BYTE, crc);
    writeCrc(crc);
#ifdef DEBUG
    DEBUG_PRINTLN();
#endif
}

void Hott::update()
{
#if defined(SIM_RX)
    static uint16_t ts = 0;
    if ((uint16_t)(millis() - ts) > 500)
    {
        uint8_t length = 2;
        static uint8_t buff[2] = {HOTT_BINARY_REQUEST_ID, HOTT_ESC_ID};
        if (buff[1] == HOTT_ESC_ID)
            buff[1] = HOTT_GAM_ID;
        else if (buff[1] == HOTT_GAM_ID)
            buff[1] = HOTT_GPS_ID;
        else if (buff[1] == HOTT_GPS_ID)
            buff[1] = HOTT_ESC_ID;
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
        }
        DEBUG_PRINTLN();
#endif
#endif
        if (length == HOTT_PACKET_LENGTH)
        {
            if (buff[0] == HOTT_BINARY_REQUEST_ID)
            {
                if (buff[1] == HOTT_GAM_ID && isGamEnabled)
                    sendGam();
                if (buff[1] == HOTT_GPS_ID && isGpsEnabled)
                    sendGps();
                if (buff[1] == HOTT_ESC_ID && isEscEnabled)
                    sendEsc();
            }
        }
    }
    // update devices
    if (deviceElementP)
    {
        deviceElementP->deviceP->update();
        deviceElementP = deviceElementP->nextP;
    }
}

void Hott::addDevice(AbstractDevice *deviceP)
{
    DeviceElement *newDeviceElementP;
    newDeviceElementP = new DeviceElement;
    newDeviceElementP->deviceP = deviceP;
    if (deviceElementP == NULL)
    {
        deviceElementP = newDeviceElementP;
        newDeviceElementP->nextP = newDeviceElementP;
    }
    else
    {
        newDeviceElementP->nextP = deviceElementP->nextP;
        deviceElementP->nextP = newDeviceElementP;
        deviceElementP = newDeviceElementP;
    }
}

void Hott::setConfig(Config &config)
{
    if (ESC_PROTOCOL == PROTOCOL_PWM)
    {
        EscPWM *esc;
        esc = new EscPWM(ALPHA(config.average.rpm));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V3)
    {
        EscHW3 *esc;
        esc = new EscHW3(ESC_SERIAL, ALPHA(config.average.rpm));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        hottEsc.voltageP = esc->voltageP();
        hottEsc.currentP = esc->currentP();
        hottEsc.tempP = esc->tempFetP();
        hottEsc.tempBecP = esc->tempBecP();
        hottEsc.consumptionP = esc->consumptionP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        hottEsc.voltageP = esc->voltageP();
        hottEsc.currentP = esc->currentP();
        hottEsc.consumptionP = esc->consumptionP();
        hottEsc.becVoltageP = esc->becVoltageP();
        hottEsc.becCurrentP = esc->becCurrentP();
        hottEsc.tempP = esc->temperatureP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        hottEsc.voltageP = esc->voltageP();
        hottEsc.currentP = esc->currentP();
        hottEsc.consumptionP = esc->consumptionP();
        hottEsc.becVoltageP = esc->becVoltageP();
        hottEsc.becCurrentP = esc->becCurrentP();
        hottEsc.tempP = esc->tempFetP();
        hottEsc.tempBecP = esc->tempBecP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        hottEsc.voltageP = esc->voltageP();
        hottEsc.currentP = esc->currentP();
        hottEsc.consumptionP = esc->consumptionP();
        hottEsc.tempP = esc->tempP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        hottEsc.rpmP = esc->rpmP();
        hottEsc.voltageP = esc->voltageP();
        hottEsc.currentP = esc->currentP();
        hottEsc.consumptionP = esc->consumptionP();
        hottEsc.tempP = esc->tempP();
        addDevice(esc);
        isEscEnabled = true;
    }
    if (config.gps == true)
    {
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        hottGps.latP = gps->latP();
        hottGps.lonP = gps->lonP();
        hottGps.spdP = gps->spdP();
        hottGps.cogP = gps->cogP();
        hottGps.altP = gps->altP();
        hottGps.satP = gps->satP();
        hottGps.varioP = gps->varioP();
        hottGps.distP = gps->distP();
        hottGps.satP = gps->satP();
        hottGps.timeP = gps->timeP();
        addDevice(gps);
        isGpsEnabled = true;
    }
    if (config.airspeed == true)
    {
        Pressure *pressure;
        pressure = new Pressure(PIN_PRESSURE, ALPHA(config.average.volt));
        hottGam.speedP = pressure->valueP();
        addDevice(pressure);
        isGamEnabled = true;
    }
    if (config.voltage1 == true)
    {
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        hottGam.voltage1P = voltage->valueP();
        addDevice(voltage);
        isGamEnabled = true;
    }
    if (config.voltage2 == true)
    {
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE2, ALPHA(config.average.volt), VOLTAGE2_MULTIPLIER);
        hottGam.voltage2P = voltage->valueP();
        addDevice(voltage);
        isGamEnabled = true;
    }
    if (config.ntc1 == true)
    {
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC1, ALPHA(config.average.temp));
        hottGam.temperature1P = ntc->valueP();
        addDevice(ntc);
        isGamEnabled = true;
    }
    if (config.ntc2 == true)
    {
        Ntc *ntc;
        ntc = new Ntc(PIN_NTC2, ALPHA(config.average.temp));
        hottGam.temperature2P = ntc->valueP();
        addDevice(ntc);
        isGamEnabled = true;
    }
    if (config.current == true)
    {
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        hottGam.currentP = current->valueP();
        hottGam.consumptionP = current->consumptionP();
        addDevice(current);
        isGamEnabled = true;
    }
    if (config.deviceI2C1Type == I2C_BMP280)
    {
        Bmp280 *bmp;
        bmp = new Bmp280(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        hottGam.altP = bmp->altitudeP();
        hottGam.varioP = bmp->varioP();
        addDevice(bmp);
        isGamEnabled = true;
    }
    if (config.deviceI2C1Type == I2C_MS5611)
    {
        MS5611 *bmp;
        bmp = new MS5611(config.deviceI2C1Address, ALPHA(config.average.temp), ALPHA(1));
        bmp->begin();
        hottGam.altP = bmp->altitudeP();
        hottGam.varioP = bmp->varioP();
        addDevice(bmp);
        isGamEnabled = true;
    }
}