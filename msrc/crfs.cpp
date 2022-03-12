#include "crfs.h"

Crfs::Crfs(AbstractSerial &serial) : serial_(serial)
{
}

Crfs::~Crfs()
{
}

void Crfs::begin()
{
    serial_.begin(420000, SERIAL__8N1 | SERIAL__HALF_DUP);
    serial_.setTimeout(CRFS_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void Crfs::sendPacket(uint8_t packetId)
{
    if (isGpsEnabled) sendGps();
    if (isBatteryEnabled) sendBattery();
    
    digitalWrite(LED_BUILTIN, HIGH);
    
#ifdef DEBUG
    DEBUG_PRINT(">");
    for (uint8_t i = 0; i < lengthExBuffer + 8; i++)
    {
        DEBUG_PRINT_HEX(buffer[i]);
        DEBUG_PRINT(" ");
    }
    DEBUG_PRINTLN();
#endif
    digitalWrite(LED_BUILTIN, LOW);
}

void Crfs::update()
{
    uint8_t status = CRFS_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    static uint8_t packetId = 0;
    if ((uint16_t)(millis() - ts) > 100)
    {
        if (!mute)
        {
            status = CRFS_SEND;
            packetId++;
        }
        mute = !mute;
        ts = millis();
    }
#else
    uint8_t packetId;
    uint8_t length = serial_.availableTimeout();
    static uint16_t ts = 0;
    if (length)
    {
        uint8_t buff[length];
        serial_.readBytes(buff, length);
#ifdef DEBUG_PACKET2
        DEBUG_PRINT("<");
        for (uint8_t i = 0; i < length; i++)
        {
            DEBUG_PRINT_HEX(buff[i]);
            DEBUG_PRINT(" ");
            delayMicroseconds(100);
        }
        DEBUG_PRINTLN();
#endif
        uint8_t packet[CRFS_PACKET_LENGHT];
        if (length == CRFS_PACKET_LENGHT)
        {
            if (!mute)
            {
                status = CRFS_SEND;
            }
            mute = !mute;
        }
#endif
    if (status == CRFS_SEND)
    {
        if (serial_.timestamp() < 1500)
            sendPacket(packetId);
#ifdef DEBUG
        else
        {
            DEBUG_PRINT("KO");
            DEBUG_PRINTLN();
        }
#endif
        }
    }
    // update sensor
    static uint8_t cont = 0;
    if (sensorCrfsP[cont])
    {
        sensorCrfsP[cont]->update();
    }
    cont++;
    if (cont == 16 || sensorCrfsP == NULL)
        cont = 0;
}

void Crfs::setConfig(Config &config)
{
    /*
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        SensorCrfs *sensorCrfsP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("RPM");
        sensorCrfsP->setUnit("RPM");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temp FET");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temp BEC");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Cell Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        SensorCrfs *sensorCrfsP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("RPM");
        sensorCrfsP->setUnit("RPM");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->rippleVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Ripple Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->becCurrentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("BEC Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("BEC Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->temperatureP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temperature");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Cell Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        SensorCrfs *sensorCrfsP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("RPM");
        sensorCrfsP->setUnit("RPM");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->becCurrentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("BEC Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("BEC Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temp FET");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temp BEC");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Cell Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        SensorCrfs *sensorCrfsP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("RPM");
        sensorCrfsP->setUnit("RPM");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temperature");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Cell Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        SensorCrfs *sensorCrfsP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("RPM");
        sensorCrfsP->setUnit("RPM");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Current");
        sensorCrfsP->setUnit("A");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->tempP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Temperature");
        sensorCrfsP->setUnit("C");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Cell Voltage");
        sensorCrfsP->setUnit("V");
        sensorCrfsP = new SensorCrfs(CRFS_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrfsP->setSensorId(addSensor(sensorCrfsP));
        sensorCrfsP->setText("Consumption");
        sensorCrfsP->setUnit("mAh");
    }
    */
    
    if (config.gps == true)
    {
        SensorCrfs *sensorCrfsP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        isGpsEnabled = true;

    }
    if (config.voltage1 == true)
    {
        SensorCrfs *sensorCrfsP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        isBatteryEnabled = true;

    }
    if (config.current == true)
    {
        SensorCrfs *sensorCrfsP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        isBatteryEnabled = true;
    }

}

uint8_t Crfs::crc(uint8_t *crc, uint8_t crc_length)
{
    uint8_t crc_up = 0;
    uint8_t c;
    for (c = 0; c < crc_length; c++)
    {
        crc_up = update_crc8(crc[c], crc_up);
    }
    return crc_up;
}

uint8_t Crfs::update_crc(uint8_t crc, uint8_t crc_seed)
{
    uint8_t crc_u;
    uint8_t i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for (i = 0; i < 8; i++)
    {
        crc_u = (crc_u & 0x80) ? 0x07 ^ (crc_u << 1) : (crc_u << 1);
    }
    return crc_u;
}
