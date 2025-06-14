#pragma once

#include <Arduino.h>
#include <IBusBM.h>
#include <ArduinoJson.h>

class FlySkyReceiver
{
public:
    HardwareSerial &serial;
    IBusBM *ibus = nullptr;
    hw_timer_t *timerHandle = nullptr;
    bool isPowerOn = false;
    bool transmitter_on = false;

    static constexpr uint8_t MAX_CHANNELS = 10;
    uint16_t channels[MAX_CHANNELS] = {0};

    FlySkyReceiver(HardwareSerial &hwSerial = Serial1)
        : serial(hwSerial)
    {}

    void begin()
    {
        stopIBus();
        pinMode(IBUS_CONTROL_PIN, OUTPUT);
        start();
    }

    void start()
    {
        if (isPowerOn)
        {
            Serial.println(F("IBus already started!"));
            return;
        }
        powerOn();
        serial.begin(115200, SERIAL_8N1, FLY_SKY_IBUS_RX_PIN, -1);
        startIBus();
    }
    
    void stop()
    {
        stopIBus();
        serial.end();
        powerOff();
    }

    void powerOn()
    {
        digitalWrite(IBUS_CONTROL_PIN, HIGH);
        isPowerOn = true;
    }

    void powerOff()
    {
        digitalWrite(IBUS_CONTROL_PIN, LOW);
        isPowerOn = false;
    }

    void startIBus()
    {
        if (timerHandle)
            Serial.println(F("IBus already started!"));
            return; 
        extern IBusBM *IBusBMfirst;
        stopIBus();  // на всякий случай

        ibus = new IBusBM();
        ibus->begin(serial, IBUSBM_NOTIMER);  // мы сами вызываем loop()

        // создаём таймер
        timerHandle = timerBegin(3, F_CPU / 1000000L, true); // timer 3, 1 µs steps
        timerAttachInterrupt(timerHandle, []() {
            if (IBusBMfirst) IBusBMfirst->loop();
        }, true);
        timerAlarmWrite(timerHandle, 1000, true);  // 1000 µs = 1 ms
        timerAlarmEnable(timerHandle);
    }

    void stopIBus()
    {
        if (timerHandle)
        {
            timerEnd(timerHandle);
            timerHandle = nullptr;
        }

        // Обнуляем глобальный указатель
        extern IBusBM *IBusBMfirst;
        IBusBMfirst = nullptr;

        if (ibus)
        {
            delete ibus;
            ibus = nullptr;
        }
    }

    bool isReceiverAlive()
    {
        static uint32_t lastCnt = 0;
        static uint32_t lastCheck = 0;
        uint32_t now = millis();

        if (now - lastCheck > 500)
        {
            lastCheck = now;
            if (ibus && ibus->cnt_rec != lastCnt)
            {
                lastCnt = ibus->cnt_rec;
                return true;
            }
            return false;
        }
        return true;
    }

    uint16_t getChannel(uint8_t ch)
    {
        if (ibus && ch < MAX_CHANNELS)
        {
            channels[ch] = ibus->readChannel(ch);
            if (ch == 2)
                transmitter_on = (channels[ch] >= 1000);
            return channels[ch];
        }
        return 0;
    }

    void updateAllChannels()
    {
        if (!ibus) return;

        for (uint8_t i = 0; i < MAX_CHANNELS; ++i)
        {
            channels[i] = ibus->readChannel(i);
        }
        transmitter_on = (channels[2] >= 1000);
    }

    void toJson(JsonObject &json)
    {
        updateAllChannels();
        JsonArray chArr = json.createNestedArray("channels");
        for (uint8_t i = 0; i < MAX_CHANNELS; ++i)
        {
            chArr.add(channels[i]);
        }

        json["power"] = isPowerOn;
        json["receiver_alive"] = isReceiverAlive();
        json["transmitter_on"] = transmitter_on;
        json["ibus_packets"] = ibus ? ibus->cnt_rec : 0;
        json["ibus_sensor_reads"] = ibus ? ibus->cnt_sensor : 0;
        json["ibus_discover"] = ibus ? ibus->cnt_poll : 0;
    }
};
