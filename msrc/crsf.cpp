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

void Crsf::sendPacket(uint8_t packetId)
{
    static uint8_t packetCount = 0;
    digitalWrite(LED_BUILTIN, HIGH);
    if (isGpsEnabled) sendGps();
    if (isBatteryEnabled) sendBattery();
    if (isVarioEnabled) sendVario();
    packetCount++;
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

void Crsf::update()
{
    uint8_t status = CRSF_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    static uint8_t packetId = 0;
    if ((uint16_t)(millis() - ts) > 100)
    {
        if (!mute)
        {
            status = CRSF_SEND;
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
        uint8_t packet[CRSF_PACKET_LENGHT];
        if (length == CRSF_PACKET_LENGHT)
        {
            if (!mute)
            {
                status = CRSF_SEND;
            }
            mute = !mute;
        }
#endif
    if (status == CRSF_SEND)
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
    if (sensorCrsfP[cont])
    {
        sensorCrsfP[cont]->update();
    }
    cont++;
    if (cont == 16 || sensorCrsfP == NULL)
        cont = 0;
}

void Crsf::setConfig(Config &config)
{
    /*
    if (ESC_PROTOCOL == PROTOCOL_HW_V4)
    {
        SensorCrsf *sensorCrsfP;
        EscHW4 *esc;
        esc = new EscHW4(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp), 0);
        esc->begin();
        PwmOut pwmOut;
        pwmOut.setRpmP(esc->rpmP());
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("RPM");
        sensorCrsfP->setUnit("RPM");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temp FET");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temp BEC");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Cell Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_CASTLE)
    {
        SensorCrsf *sensorCrsfP;
        EscCastle *esc;
        esc = new EscCastle(ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("RPM");
        sensorCrsfP->setUnit("RPM");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->rippleVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Ripple Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->becCurrentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("BEC Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("BEC Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->temperatureP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temperature");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Cell Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_KONTRONIK)
    {
        SensorCrsf *sensorCrsfP;
        EscKontronik *esc;
        esc = new EscKontronik(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("RPM");
        sensorCrsfP->setUnit("RPM");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->becCurrentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("BEC Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->becVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("BEC Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempFetP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temp FET");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempBecP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temp BEC");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Cell Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_F)
    {
        SensorCrsf *sensorCrsfP;
        EscApdF *esc;
        esc = new EscApdF(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("RPM");
        sensorCrsfP->setUnit("RPM");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temperature");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Cell Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
    }
    if (ESC_PROTOCOL == PROTOCOL_APD_HV)
    {
        SensorCrsf *sensorCrsfP;
        EscApdHV *esc;
        esc = new EscApdHV(ESC_SERIAL, ALPHA(config.average.rpm), ALPHA(config.average.volt), ALPHA(config.average.curr), ALPHA(config.average.temp));
        esc->begin();
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT22, 0, esc->rpmP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("RPM");
        sensorCrsfP->setUnit("RPM");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 1, esc->currentP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Current");
        sensorCrsfP->setUnit("A");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->voltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->tempP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Temperature");
        sensorCrsfP->setUnit("C");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 2, esc->cellVoltageP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Cell Voltage");
        sensorCrsfP->setUnit("V");
        sensorCrsfP = new SensorCrsf(CRSF_TYPE_INT14, 0, esc->consumptionP(), esc);
        sensorCrsfP->setSensorId(addSensor(sensorCrsfP));
        sensorCrsfP->setText("Consumption");
        sensorCrsfP->setUnit("mAh");
    }
    */
    
    if (config.gps == true)
    {
        SensorCrsf *sensorCrsfP;
        Bn220 *gps;
        gps = new Bn220(GPS_SERIAL, GPS_BAUD_RATE);
        gps->begin();
        isGpsEnabled = true;

    }
    if (config.voltage1 == true)
    {
        SensorCrsf *sensorCrsfP;
        Voltage *voltage;
        voltage = new Voltage(PIN_VOLTAGE1, ALPHA(config.average.volt), VOLTAGE1_MULTIPLIER);
        isBatteryEnabled = true;

    }
    if (config.current == true)
    {
        SensorCrsf *sensorCrsfP;
        Current *current;
        current = new Current(PIN_CURRENT, ALPHA(config.average.curr), CURRENT_MULTIPLIER);
        isBatteryEnabled = true;
    }

}

uint8_t Crsf::crc(uint8_t *crc, uint8_t crc_length)
{
    uint8_t crc_up = 0;
    uint8_t c;
    for (c = 0; c < crc_length; c++)
    {
        crc_up = update_crc8(crc[c], crc_up);
    }
    return crc_up;
}

uint8_t Crsf::update_crc(uint8_t crc, uint8_t crc_seed)
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
