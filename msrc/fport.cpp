#include "fport.h"

FPort::FPort(AbstractSerial &serial) : Smartport(serial)
{
}

FPort::~FPort()
{
}

void FPort::begin()
{
    serial_.begin(115200, SERIAL__8N1_RXINV_TXINV | SERIAL__HALF_DUP);
    serial_.setTimeout(FPORT_TIMEOUT);
    pinMode(LED_BUILTIN, OUTPUT);
    Config config = {CONFIG_AIRSPEED, CONFIG_GPS, CONFIG_VOLTAGE1, CONFIG_VOLTAGE2, CONFIG_CURRENT, CONFIG_NTC1, CONFIG_NTC2, CONFIG_PWMOUT, {CONFIG_REFRESH_RPM, CONFIG_REFRESH_VOLT, CONFIG_REFRESH_CURR, CONFIG_REFRESH_TEMP}, {CONFIG_AVERAGING_ELEMENTS_RPM, CONFIG_AVERAGING_ELEMENTS_VOLT, CONFIG_AVERAGING_ELEMENTS_CURR, CONFIG_AVERAGING_ELEMENTS_TEMP}, CONFIG_ESC_PROTOCOL, CONFIG_I2C1_TYPE, CONFIG_I2C1_ADDRESS, 0, 0, SENSOR_ID};
    setConfig(config);
}

void FPort::updateFPort()
{
    uint8_t status = FPORT_WAIT;
    static bool mute = true;
#if defined(SIM_RX)
    static uint16_t ts = 0;
    static uint8_t packetId = 0;
    if ((uint16_t)(millis() - ts) > 100)
    {
        if (!mute)
        {
            status = FPORT_SEND;
            packetId++;
        }
        mute = !mute;
        ts = millis();
    }
#else
    uint8_t packetId;
    uint8_t length = serial_.availableTimeout();
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
        uint8_t packet[FPORT_PACKET_LENGHT];
        if (buff[0] == 0x7E && buff[2] == 0x00 && length - buff[1] == FPORT_PACKET_LENGHT)
        {
            memcpy(packet, buff + buff[1], FPORT_PACKET_LENGHT);
#ifdef DEBUG_PACKET
            DEBUG_PRINT("P:");
            for (uint8_t i = 0; i < FPORT_PACKET_LENGHT; i++)
            {
                DEBUG_PRINT_HEX(packet[i]);
                DEBUG_PRINT(" ");
            }
            DEBUG_PRINTLN();
#endif
        }
        else if (length == FPORT_PACKET_LENGHT)
        {
            memcpy(packet, buff, FPORT_PACKET_LENGHT);
        }
        else
        {
            return;
        }
        //if (crc(packet, FPORT_PACKET_LENGHT) == 0)
        {
            if (packet[0] == 0x3D && packet[1] == 0x01 && packet[4] == 0x3A)
            {
                if (!mute)
                {
                    status = FPORT_SEND;
                    packetId = packet[3];
                }
                mute = !mute;
            }
        }
    }
#endif
    if (status == FPORT_SEND)
    {
        if (1) //(serial_.timestamp() < 1500)
        {
            if (packetId == 0x30 || packetId == 0x31)
                packetId = 0x32;
            sendPacket(packetId);
        }
#ifdef DEBUG
        else
        {
            DEBUG_PRINT("KO");
            DEBUG_PRINTLN();
        }
#endif
    }
    // update sensor
    if (sensorP != NULL)
    {
        sensorP->update();
        sensorP = sensorP->nextP;
    }
}

void FPort::sendPacket(uint8_t packetId)
{
    // loop sensors until correct timestamp or 1 sensors cycle
    Sensor *initialSensorP = spSensorP;
    while (((uint16_t)(millis() - spSensorP->timestamp()) <= (uint16_t)spSensorP->refresh() * 100) && spSensorP->nextP != initialSensorP)
    {
        spSensorP = spSensorP->nextP;
    }
    if ((uint16_t)(millis() - spSensorP->timestamp()) >= (uint16_t)spSensorP->refresh() * 100)
    {
        serial_.write(0x08);
        serial_.write(0x81);
        sendData(packetId, spSensorP->dataId(), spSensorP->valueFormatted());
#ifdef DEBUG
        DEBUG_PRINT("D:");
        DEBUG_PRINT_HEX(spSensorP->dataId());
        DEBUG_PRINT(" V:");
        DEBUG_PRINT(spSensorP->valueFormatted());
        spSensorP->valueFormatted(); // toggle type if date/time or lat/lon sensor
        DEBUG_PRINT(" T:");
        DEBUG_PRINT(spSensorP->timestamp());
        DEBUG_PRINTLN();
#endif
        spSensorP->setTimestamp(millis());
        spSensorP = spSensorP->nextP;
    }
    else
    {
        serial_.write(0x08);
        serial_.write(0x81);
        sendVoid();
    }
}
