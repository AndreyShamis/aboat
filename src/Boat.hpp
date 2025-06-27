// Boat.hpp
#pragma once
#include <Arduino.h>
#include <vector>
#include <ArduinoJson.h>
#include <MPU9250_asukiaaa.h>
// #include <TinyGPSPlus.h>
#include "gnss_manager.hpp"
#include "battery_monitor.hpp"
#include "voltage_current_sensor.hpp"
#include "fly_sky.hpp"
#include "system_status.hpp"
#include "temp_sensors.hpp"
#include "oil_pump.hpp"
#include "rudder.hpp"
#include "motor_control.hpp"
#include "command_parser.hpp"
#include "Adafruit_INA3221.h"
#include <Preferences.h>
#include <time.h>
#include "FS.h"
#include "SPIFFS.h"
#include "lora_comm.hpp"
#include <Adafruit_PWMServoDriver.h>
#include "freertos/queue.h"
#include "PacketClasses.hpp"

using namespace ArduinoJson;

static unsigned long lastChannelPrint = 0;
const unsigned long channelPrintInterval = 2000; // 2.5 секунды
static unsigned long lastRandomRudderTime = 0;
static unsigned long nextRandomRudderDelay = 0;
static unsigned long lastControlUpdate = 0;
const unsigned long controlInterval = 100; // 100 мс = 10 Гц

class Boat : LogInterface
{

public:
    MPU9250_asukiaaa mpu;
    FusionAhrs ahrs;
    BatteryMonitor battery;
    VoltageCurrentSensor sensor;
    Adafruit_INA3221 ina3221;
    Adafruit_INA3221 ina3221_low;
    FlySkyReceiver flysky;
    OilPumpController oilPump;
    TempSensorManager temps;
    RudderController rudder{RUDDER};
    MotorEngineControl engine;
    CommandParser parser;
    GNSSManager gnss;
    bool updateStarted = false;
    LoRaComm *loraComm; // Новый объект для LoRa связи
    Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

    TaskHandle_t sendStatusTaskHandle = nullptr;
    bool statusTaskRunning = false;

    bool waitingForASAAck = false;
    bool asaActive = false;
    unsigned long asaProposalTime = 0;
    unsigned long lastPacketReceived = 0; // обновляется при каждом принятом пакете
    unsigned long asaLastSwitchTime = 0;
    int currentSF = LORA_SF;
    int currentCR = LORA_CODING_RATE;
    float currentBW = LORA_BANDWIDTH;

    void onTelemetry(const PacketTelemetry &tel)
    {
        addLog("Received telemetry fragment: len=" + String(tel.payloadLen));
        // собрать, parse и т.д.
    }

    void onInfoEngine(const PacketInfoEngine &info)
    {
        addLog("InfoEngine packet received");
    }

    void onStatus(const PacketStatus &st)
    {
        addLog("Status packet received");
    }

    void onAck(const PacketAck &ack)
    {
        addLog("ACK for ID=" + String(ack.ackedId));
    }

    void onConfig(const PacketConfig &cfg)
    {
        addLog("Config packet received");
    }

    void onNav(const PacketNav &nav)
    {
        addLog("Nav packet received");
    }

    void onHeartbeat(const PacketHeartbeat &hb)
    {
        addLog("Heartbeat packet received");
    }

    Boat() : sensor(0x48), gnss(Serial2)
    {
        loraComm = new LoRaComm(BOAT_DEVICE_ID, this);
    }
    ~Boat()
    {
        delete loraComm;
    }

    void setup()
    {
        delay(3); // Задержка для отладки
        addLog("RAM: Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        addLog("Flash: Sketch size:" + String(ESP.getSketchSize()) + " bytes / Free: " + String(ESP.getFreeSketchSpace()) + " bytes");

        // addLog("Enabling servo power ,,,,,,,,,,,,,,,");
        // pinMode(SERVO_PWR_PIN, OUTPUT);
        // digitalWrite(SERVO_PWR_PIN, HIGH); // Включаем питание сервоприводов
        battery.setup();
        gnss.begin();

        addLog("Starting I2C bus scan...");
        Wire.begin(I2C_SDA, I2C_SCL);
        addLog("Scanning I2C bus...");

        for (uint8_t address = 1; address < 127; ++address)
        {
            Wire.beginTransmission(address);
            if (Wire.endTransmission() == 0)
            {
                addLog("Found I2C device at 0x" + String(address, HEX));
            }
        }

        mpu.setWire(&Wire);
        mpu.beginAccel();
        mpu.beginGyro();
        mpu.beginMag();
        FusionAhrsInitialise(&ahrs);

        addLog("MPU9250 + Madgwick initialized");

        if (!ina3221.begin(0X41, &Wire)) // Инициализация INA3221 на адресе 0x41
        {
            addLog("-INA3221 not found at address 0x41");
        }
        else
        {
            addLog("+INA3221 found at address 0x41");
            ina3221.setAveragingMode(INA3221_AVG_16_SAMPLES);
            for (uint8_t i = 0; i < 3; i++)
            {
                ina3221.setShuntResistance(i, 0.1);
            }

            // Set a power valid alert to tell us if ALL channels are between the two
            // limits:
            ina3221.setPowerValidLimits(4.0 /* lower limit */, 18.0 /* upper limit */);
        }
        if (!ina3221_low.begin(0X43, &Wire)) // Инициализация INA3221 на адресе 0x43
        {
            addLog("-INA3221 LOW not found at address 0x43");
        }
        else
        {
            addLog("+INA3221v LOW found at address 0x43");
            ina3221_low.setAveragingMode(INA3221_AVG_16_SAMPLES);
            for (uint8_t i = 0; i < 3; i++)
            {
                ina3221.setShuntResistance(i, 0.1);
            }

            // Set a power valid alert to tell us if ALL channels are between the two
            // limits:
            ina3221_low.setPowerValidLimits(3.2 /* lower limit */, 10.0 /* upper limit */);
        }
        SystemStatus::printResetReason();
        SystemStatus::printWakeupReason();
        SystemStatus::printUptime();
        sensor.begin();
        temps.begin();
        addLog("Dallas sensors initialized");
        oilPump.begin();
        addLog("Oil pump initialized");

        rudder.begin();
        rudder.setTrim(0);
        engine.begin(MOTOR_LEFT, MOTOR_RIGHT);
        addLog("Boat finish setup");

        parser.registerCommand("M", [this](const String &arg)
                               {
                                    addLog("[CMD M] Motor power set to: " + arg);
                                    int power = arg.toInt();
                                    engine.apply(power, 1500); });

        parser.registerCommand("E", [this](const String &arg)
                               {
                                    addLog("[CMD E] Enfine power set to: " + arg);
                                        if (arg=="S") {
                                            addLog("[CMD E] Engine stopped");
                                            engine.setState(MotorEngineControl::MOTOR_STOP);
                                        } else if(arg=="F") {
                                            addLog("[CMD E] Engine forward");
                                            engine.setState(MotorEngineControl::MOTOR_FORWARD);
                                        } else if(arg=="R") {
                                            addLog("[CMD E] Engine reverse");
                                            engine.setState(MotorEngineControl::MOTOR_REVERSE);
                                        } else {
                                            addLog("[CMD E] Unknown engine command: " + arg);
                                        } });
        parser.registerCommand("R", [this](const String &arg)
                               {
                                    addLog("[CMD R] Rudder angle set to: " + arg);
                                    int angle = arg.toInt();
                                    rudder.setAngle(angle); });

        parser.registerCommand("P", [this](const String &arg)
                               {
                                    addLog("[CMD R] Oil pump set to: " + arg);
                                    int speed = arg.toInt();
                                    oilPump.setSpeed(speed); });

        addLog("Commands registered: M (motor), R (rudder), P (oil pump)");

        addLog("Boat setup completed.");
        addLog("Free heap: " + String(ESP.getFreeHeap()) + " bytes");
        addLog("Free sketch space: " + String(ESP.getFreeSketchSpace()) + " bytes");
        addLog("Sketch size: " + String(ESP.getSketchSize()) + " bytes");
        addLog("Flash chip size: " + String(ESP.getFlashChipSize()) + " bytes");
        addLog("Flash chip speed: " + String(ESP.getFlashChipSpeed()) + " Hz");
        addLog("Chip model: " + String(ESP.getChipModel()));
        addLog("Chip revision: " + String(ESP.getChipRevision()));
        addLog("Chip cores: " + String(ESP.getChipCores()));
        addLog("CPU frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
        addLog("SDK version: " + String(ESP.getSdkVersion()));
        addLog("MAC address: " + String(ESP.getEfuseMac(), HEX));
        printSPIFFSInfo();
        // listSPIFFSFiles();
        addLog("Starting PWM for servos...");

        bool servoStarted = pwm.begin();
        if (!servoStarted)
        {
            addLog(" - ERROR! failed to start Adafruit_PWMServoDriver.");
        }
        else
        {
            addLog(" + Adafruit_PWMServoDriver succesfully connected to module.");
            pwm.setPWMFreq(50);
        }

        // Инициализация нового класса LoRaComm
        if (!loraComm->begin())
        {
            addLog(" - ERROR! LoRaComm не инициализирован.");
        }
        else
        {
            // loraAdapt = new AdaptiveLoRaManager(loraComm, this);
            // loraAdapt->begin();
            addLog(" + LoRaComm успешно инициализирован.");
        }

        addLog("Strarting setup iBUS FlySky...");
        flysky.begin();
        addLog("Boat initialized successfully.");
    }

    unsigned long lastSync = 0;

    void keep()
    {
        if (updateStarted)
        {
            addLog("Update in progress, skipping keep() cycle.");
            return;
        }
        // if (loraAdapt)
        //     loraAdapt->loop();

        battery.prepareForRead();
        gnss.update();
        // engine.checkMotorStateChange(ch1, ch2, ch3, ch4);
        // engine.apply(ch3, ch4);
        //  rudder.update();      in main
        // oilPump.update(motor1Temp, motor2Temp, radiatorTemp);
        battery.readVoltage();

        static unsigned long lastOilPumpUpdate = 0;
        static unsigned long VuPDATE = 0;

        if (millis() - lastOilPumpUpdate >= BOAT_TMR_OIL_PUMP_UPDATE_TIME)
        {
            int percent = 0;
            lastOilPumpUpdate = millis();
            if (temps.get(MOTOR1) > 35 || temps.get(MOTOR2) > 35)
            {
                percent = 90;
            }
            else
            {
                percent = 0;
            }

            oilPump.setSpeed(percent);
        }

        // Показываем системное время раз в 5 секунд
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > BOAT_TMR_STSTEM_PRINT_TIME)
        {
            lastPrint = millis();
            time_t now = time(nullptr);
            addLog("System time:" + String(now));
        }

        static unsigned long lastStatusJsonTime = 0;
        if (millis() - lastStatusJsonTime > 1200000 && !statusTaskRunning)
        {
            lastStatusJsonTime = millis();
            if (sendStatusTaskHandle == nullptr || eTaskGetState(sendStatusTaskHandle) == eDeleted)
            {
                xTaskCreatePinnedToCore(
                    Boat::sendStatusJsonTaskWrapper,
                    "SendStatus",
                    8192,
                    this,
                    1,
                    &sendStatusTaskHandle,
                    1);
            }
        }

        // раз в 20 сек отправляем “ping” на пульт
        if (millis() - lastPingSent >= 5121)
        {
            PacketBase ping{};
            ping.packetType = CMD_PING;
            ping.packetId = nextPacketId++;
            ping.payloadLen = 0;
            loraComm->sendPacket(ping, MISSION_CONTROL_ID);

            addLog("🔄 Ping → MC");

            lastPingSent = millis();
        }
        // Например, каждые 30 сек, если ASA не активен, и нет ожидания ACK:
        uint8_t senderId;
        PacketBase hdr;
        uint8_t payloadBuf[MAX_LORA_PAYLOAD];

        // пытаемся распарсить
        if (loraComm->parseReceivedPacket(senderId, hdr, payloadBuf))
        {
            float snr = loraComm->getRadio().getSNR();
            float rssi = loraComm->getRadio().getRSSI();
            addLog("profile:" + String(currentProfileIndex) + " .RSSI:" + String(rssi) + "dBm, SNR=" + String(snr, 1) + "dB.  type=" + String((char)hdr.packetType) + " from=" + String(senderId) + " id=" + String(hdr.packetId) + " PacketBase: " + hdr.toString());

            // Диспетчер внутри Boat
            switch (hdr.packetType)
            {

            case CMD_COMMAND_STRING:
            {
                PacketCommand cmd{};
                cmd.packetType = hdr.packetType;
                cmd.packetId = hdr.packetId;
                cmd.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                // выполнить строку команды
                String s;
                for (size_t i = 0; i < cmd.payloadLen; ++i)
                {
                    s += char(payloadBuf[i]);
                }
                parser.processLine(s);
                break;
            }

            case CMD_TELEMETRY_FRAGMENT:
            {
                PacketTelemetry tel{};
                tel.packetType = hdr.packetType;
                tel.packetId = hdr.packetId;
                tel.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&tel) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onTelemetry(tel);
                break;
            }

            case CMD_INFO_ENGINE:
            {
                PacketInfoEngine info{};
                info.packetType = hdr.packetType;
                info.packetId = hdr.packetId;
                info.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&info) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onInfoEngine(info);
                break;
            }

            case CMD_STATUS:
            {
                PacketStatus st{};
                st.packetType = hdr.packetType;
                st.packetId = hdr.packetId;
                st.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&st) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onStatus(st);
                break;
            }

            case CMD_ACK:
            {
                PacketAck ackIn{};
                ackIn.packetType = hdr.packetType;
                ackIn.packetId = hdr.packetId;
                ackIn.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&ackIn) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onAck(ackIn);
                break;
            }

            case CMD_CONFIG:
            {
                PacketConfig cfg{};
                cfg.packetType = hdr.packetType;
                cfg.packetId = hdr.packetId;
                cfg.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&cfg) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onConfig(cfg);
                break;
            }

            case CMD_NAV:
            {
                PacketNav nav{};
                nav.packetType = hdr.packetType;
                nav.packetId = hdr.packetId;
                nav.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&nav) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onNav(nav);
                break;
            }

            case CMD_HEARTBEAT:
            {
                PacketHeartbeat hb{};
                hb.packetType = hdr.packetType;
                hb.packetId = hdr.packetId;
                hb.payloadLen = hdr.payloadLen;
                memcpy(reinterpret_cast<uint8_t *>(&hb) + sizeof(PacketBase),
                       payloadBuf,
                       hdr.payloadLen);
                onHeartbeat(hb);
                break;
            }
            case CMD_PONG:
            {
                payloadBuf[hdr.payloadLen] = '\0'; // безопасное завершение
                addLog("Received PONFsendpaG from MC: ");
                // loraAdapt->onPongReceived();
                break;
            }
            case CMD_REPOSNCE_ASA:
            {
                addLog("✅ ASA response received. Applying higher LoRa profile");

                waitingForASAAck = false;
                asaActive = true;
                asaLastSwitchTime = millis();

                if (currentProfileIndex < LORA_PROFILE_COUNT - 1)
                {
                    currentProfileIndex++;
                    applyProfile(currentProfileIndex);
                }
                else
                {
                    addLog("⚠️ Already at max LoRa profile, skipping upgrade");
                }

                break;
                // addLog("Received ASA response from MC");
                // loraAdapt->onPongReceived();
                // break;
            }
            case CMD_GET_BOAT_STATUS:
            {
                sendStatusJsonFragmentsTask();
                break;
            }
            case CMD_PING:
            {
                addLog("🔄 Ping ← MC");
                break;
            }
            case CMD_REQUEST_INFO:
            {
                // Универсальный “P” запрос — в payloadBuf[0] лежит нужный код ('T','I','S','F','G','D'…)
                CommandType what = static_cast<CommandType>(payloadBuf[0]);
                switch (what)
                {

                //  ——————————————————————————————————————————————————————————
                // Телеметрия / статус лодки (JSON-фрагменты)
                //  ——————————————————————————————————————————————————————————
                case CMD_BOAT_STATUS_REPORT:
                { // 'T' или 'D'
                    sendStatusJsonFragmentsTask();
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Информация о двигателе
                //  ——————————————————————————————————————————————————————————
                case CMD_INFO_ENGINE:
                { // 'I'
                    PacketInfoEngine info{};
                    info.packetType = CMD_INFO_ENGINE;
                    info.packetId = nextPacketId++;
                    // заполняем поля info... (например, текущие обороты, напряжение и т.п.)
                    loraComm->sendPacket(info, senderId);
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Конфигурация
                //  ——————————————————————————————————————————————————————————
                case CMD_CONFIG:
                { // 'F'
                    PacketConfig cfg{};
                    cfg.packetType = CMD_CONFIG;
                    cfg.packetId = nextPacketId++;
                    // заполняем cfg... (например, текущие порты, параметры PID)
                    loraComm->sendPacket(cfg, senderId);
                    break;
                }
                //  ——————————————————————————————————————————————————————————
                // Навигация
                //  ——————————————————————————————————————————————————————————
                case CMD_NAV:
                { // 'G'
                    PacketNav nav{};
                    nav.packetType = CMD_NAV;
                    nav.packetId = nextPacketId++;
                    // заполняем nav… (координаты, курс, скорость)
                    loraComm->sendPacket(nav, senderId);
                    break;
                }

                default:
                    addLog("Boat: Unsupported info request '" + String((char)what) + "'");
                    break;
                }
                break;
            }

            default:
                addLog("Unknown LoRa cmd: " + String((char)hdr.packetType));
                break;
            }

            // В конце — отправляем ACK обратно отправителю
            PacketAck ackOut{};
            ackOut.packetId = ++nextPacketId;
            ackOut.payloadLen = 1;
            ackOut.ackedId = hdr.packetId;
            loraComm->sendPacket(ackOut, senderId);

            lastPacketReceived = millis();
        }

        adaptiveLoraUpdate();

        // --- ASA: Проверка ожидания ACK и таймаута канала ---
        if (waitingForASAAck && millis() - asaProposalTime > 5000)
        {
            addLog("❌ ASA: Нет ACK от управления. Отклоняем переход.");
            waitingForASAAck = false;
            // Остались на старых параметрах
        }

        if (asaActive && millis() - lastPacketReceived > 120000)
        {
            addLog("⏳ ASA: Таймаут активности. Возврат к дефолтным LoRa-настройкам.");
            restoreDefaultLoRaSettings();
            asaActive = false;
        }

        // Синхронизация при первом валидном значении GPS времени
        // if (gnss.gnss.time.isUpdated() && millis() - lastSync > 10000)
        // {
        //     syncTimeFromGPS();
        //     lastSync = millis();
        // }
        // static unsigned long wakeStart = 0;
        // static bool firstRun = true;

        // if (firstRun)
        // {
        //     wakeStart = millis();
        //     firstRun = false;
        //     Serial.println("⏰ Wake cycle started.");
        // }

        // const unsigned long wakeDuration = 3600000;                 // 1 минута
        // const uint64_t sleepDurationUs = 1ULL * 60ULL * 1000000ULL; // 5 минут

        // if (millis() - wakeStart >= wakeDuration)
        // {
        //     if (!flysky.transmitter_on)
        //     {
        //         flysky.powerOff();
        //         sensorsPowerOff();
        //         delay(100); // Даем время на отключение
        //         Serial.println("😴 No transmitter detected. Going to deep sleep for 5 minutes...");
        //         ESP.deepSleep(sleepDurationUs);
        //     }
        //     else
        //     {
        //         Serial.println("📡 Transmitter is active. Stay awake.");
        //         wakeStart = millis(); // сбрасываем цикл, остаёмся бодрствовать
        //     }
        // }
        while (Serial.available())
        {
            char c = Serial.read();
            if (c == '\n')
            {
                Serial.println("Command processed: " + inputBuffer);
                parser.processLine(inputBuffer);
                inputBuffer = "";
            }
            else if (c >= 32 && c <= 126)
            {
                inputBuffer += c;
            }
        }
        uint16_t ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8, ch9, ch10;

        if (!failsafeTriggered)
        {

            ch1 = flysky.getChannel(0); // Правый stick X
            ch2 = flysky.getChannel(1); // Правый stick Y
            ch3 = flysky.getChannel(2); // Левый stick Y
            ch4 = flysky.getChannel(3); // Левый stick X
            ch5 = flysky.getChannel(4);
            ch6 = flysky.getChannel(5);
            ch7 = flysky.getChannel(6);
            // ch8 = flysky.getChannel(7);
            // ch9 = flysky.getChannel(8);
            // ch10 = flysky.getChannel(9);
            //}

            // if (millis() - lastRandomRudderTime >= nextRandomRudderDelay)
            // {
            //     lastRandomRudderTime = millis();
            //     nextRandomRudderDelay = random(20, 2500); // 3–5 секунд

            //     int randomAngle = random(-91, 91); // Угол от -60 до 60
            //     rudder.setAngle(randomAngle);

            //     // Serial.printf("🎲 Random rudder angle set to: %d°\n", randomAngle);
            // }
            if (ch5 < 1900)
            {
                engine.setState(engine.MOTOR_STOP);
            }
            else
            {
                if (ch6 < 1500 && ch6 >= 1000)
                {
                    engine.setState(engine.MOTOR_FORWARD);
                }
                else if (ch6 > 1500 && ch6 <= 2000)
                {
                    engine.setState(engine.MOTOR_REVERSE);
                }
                else
                {
                    engine.setState(engine.MOTOR_STOP);
                }
            }
            if (millis() - lastChannelPrint > channelPrintInterval)
            {
                lastChannelPrint = millis();
                Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u\n", ch1, ch2, ch3, ch4);
                // Serial.printf("CH1: %u | CH2: %u | CH3: %u | CH4: %u -- CH5: %u | CH6: %u | CH7: %u | CH8: %u :::: CH9: %u - CH10: %u\n", ch1, ch2, ch3, ch4, ch5, ch6, ch7, ch8,ch9, ch10);
            }

            lastControlUpdate = millis();
            setThrottleLimit(ch7);
            engine.apply(ch3, ch4);

            if (ch1 >= 1000 && ch1 <= 2000 && flysky.transmitter_on)
            {
                rudder.setAngle(map(ch1, 1000, 2000, -90, 90)); // Обновляем руль только если есть валидный сигнал
            }
        }
        else
        {
            engine.update(); // Обновляем моторы даже в фейлсейфе, чтобы они остановились
        }
        checkFailsafeTransition();

        rudder.update();
    }

    void adaptiveLoraUpdate()
    {
        if (waitingForASAAck || millis() - lastAdaptiveSwitchTime < switchInterval)
            return;

        lastAdaptiveSwitchTime = millis();

        float snr = loraComm->getRadio().getSNR();
        float rssi = loraComm->getRadio().getRSSI();


        int bestIndex = currentProfileIndex;
        for (int i = LORA_PROFILE_COUNT - 1; i >= 0; --i)
        {
            const auto &p = loraProfiles[i];
            if (rssi > -95 + i * 3 && snr > 1 + i)
            { // чем выше индекс, тем более требовательный профиль
                bestIndex = i;
                break;
            }
        }

        if (bestIndex != currentProfileIndex)
        {
            String direction = bestIndex > currentProfileIndex ? "upgrade" : "downgrade";
            addLog("[adaptiveLoraUpdate] rssi:" + String(rssi) + " snr:" + String(snr) +
                   " → Proposing " + direction + " to BEST profile index " + String(bestIndex));

            PacketAsaRequest asaReq{};
            asaReq.packetType = CMD_REQUEST_ASA;
            asaReq.packetId = nextPacketId++;
            asaReq.payloadLen = sizeof(uint8_t);
            asaReq.profileIndex = bestIndex;

            loraComm->sendPacket(asaReq, MISSION_CONTROL_ID);
            waitingForASAAck = true;
            if(direction == "downgrade")
            {
                applyProfile(bestIndex);
                addLog("🔼 ASA: Requesting upgrade to profile index " + String(bestIndex) + " and force update.");
            }
            asaProposalTime = millis();
        }
    }

    void applyProfile(uint8_t idx)
    {
        auto &p = loraProfiles[idx]; // ✅ Сначала получаем ссылку на профиль

        // ✉️ Готовим строку с параметрами
        char payload[16];
        snprintf(payload, sizeof(payload), "%d,%d,%.0f", p.spreadingFactor, p.codingRate, p.bandwidth);

        // 📦 Формируем ACK_ASA
        PacketAck ack{};
        ack.packetType = CMD_ACK_ASA;
        ack.packetId = ++nextPacketId;
        ack.payloadLen = strlen(payload);
        memcpy(reinterpret_cast<uint8_t *>(&ack) + sizeof(PacketBase), payload, ack.payloadLen);

        // 📤 Отправляем на MC
        loraComm->sendPacket(ack, MISSION_CONTROL_ID);

        // 🛠 Применяем настройки
        loraComm->applySettings(p.spreadingFactor, p.codingRate, p.bandwidth);
    }

    void restoreDefaultLoRaSettings()
    {
        addLog("🔁 Восстановление стандартных LoRa параметров...");
        loraComm->applySettings(LORA_SF, LORA_CODING_RATE, LORA_BANDWIDTH);
    }

    void printSPIFFSInfo()
    {
        addLog("=== SPIFFS Info ===");

        size_t totalBytes = SPIFFS.totalBytes();
        size_t usedBytes = SPIFFS.usedBytes();
        size_t freeBytes = totalBytes - usedBytes;

        addLog("Total size: " + String(totalBytes / 1024) + " Kbytes");
        addLog("Used size:  " + String(usedBytes / 1024) + " Kbytes");
        addLog("Free size:  " + String(freeBytes / 1024) + " Kbytes");
    }

    void listSPIFFSFiles()
    {
        addLog("=== Files in SPIFFS ===");

        File root = SPIFFS.open("/");
        File file = root.openNextFile();

        if (!file)
        {
            addLog("No files found");
            return;
        }

        while (file)
        {
            addLog(String(file.name()) + " (" + String(file.size()) + " bytes)");
            file = root.openNextFile();
        }
    }

    void syncTimeFromGPS()
    {
        if (gnss.gnss.getDateValid() && gnss.gnss.getTimeValid())
        {
            struct tm t;
            t.tm_year = gnss.gnss.getYear() - 1900;
            t.tm_mon = gnss.gnss.getMonth() - 1;
            t.tm_mday = gnss.gnss.getDay();
            t.tm_hour = gnss.gnss.getHour() + 3; // GMT+3
            t.tm_min = gnss.gnss.getMinute();
            t.tm_sec = gnss.gnss.getSecond();
            t.tm_isdst = 0;

            time_t timeSinceEpoch = mktime(&t);
            struct timeval now = {.tv_sec = timeSinceEpoch};
            settimeofday(&now, nullptr);

            Serial.println("⏰ System time synced from GPS!");
        }
    }

    const size_t maxPayload = sizeof(((LoRaPacket *)nullptr)->payload); // обычно 40

    // Примерная длина заголовка "[XX/YY]" — максимум 9 символов
    const size_t headerReserve = 10;                             // с запасом
    const size_t jsonChunkSize = maxPayload - headerReserve - 1; // -1 на '\0'

    static void sendStatusJsonTaskWrapper(void *param)
    {
        Boat *boat = static_cast<Boat *>(param);
        boat->sendStatusJsonFragmentsTask();
        vTaskDelete(nullptr); // убиваем задачу после выполнения
    }

    void sendStatusJsonFragmentsTask()
    {
        if (statusTaskRunning)
        {
            return;
        }
        statusTaskRunning = true;

        // Собираем полный JSON
        String json = getStatusJson();
        size_t totalLen = json.length();
        size_t chunks = (totalLen + jsonChunkSize - 1) / jsonChunkSize;

        // Для каждого фрагмента формируем команду
        for (size_t i = 0; i < chunks; i++)
        {
            // Заголовок вида “[i/N]”
            String header = "[" + String(i) + "/" + String(chunks) + "]";

            // Собственно фрагмент JSON
            String fragment = json.substring(i * jsonChunkSize, (i + 1) * jsonChunkSize);
            String payloadStr = header + fragment;

            // Упаковываем в PacketCommand
            PacketCommand cmd{};
            cmd.packetType = CMD_TELEMETRY_FRAGMENT;
            cmd.packetId = nextPacketId++;
            cmd.payloadLen = payloadStr.length();

            // Копируем payload прямо за заголовок PacketBase
            memcpy(reinterpret_cast<uint8_t *>(&cmd) + sizeof(PacketBase),
                   payloadStr.c_str(),
                   cmd.payloadLen);

            // Отправляем на Mission Control
            loraComm->sendPacket(cmd, MISSION_CONTROL_ID);
            delay(10); // небольшой интервал между фрагментами
        }

        statusTaskRunning = false;
    }

    String getStatusJson()
    {
        JsonDocument doc; // Подбери под размер данных

        JsonObject temp = doc[F("temperature")].to<JsonObject>();
        temp[F("motor1")] = temps.get(MOTOR1);
        temp[F("motor2")] = temps.get(MOTOR2);
        temp[F("radiator")] = temps.get(MOTOR_RAD);
        temp[F("oil")] = temps.get(OIL);
        temp[F("ambient")] = temps.get(ENV);

        FusionQuaternion quat = FusionAhrsGetQuaternion(&ahrs);
        FusionEuler euler = FusionQuaternionToEuler(quat);

        JsonObject imu = doc[F("imu")].to<JsonObject>();
        imu[F("roll")] = euler.angle.roll;
        imu[F("pitch")] = euler.angle.pitch;
        imu[F("yaw")] = euler.angle.yaw;

        JsonObject pumpObj = doc[F("oil_pump")].to<JsonObject>();
        oilPump.toJSON(pumpObj);
        JsonObject sysObj = doc[F("system")].to<JsonObject>();
        SystemStatus::toExtendedJSON(sysObj);

        JsonObject rudderObj = doc[F("rudder")].to<JsonObject>();
        rudder.toJSON(rudderObj);

        JsonObject gnssObj = doc[F("gnss")].to<JsonObject>();
        gnss.toJson(gnssObj);

        JsonObject motorObj = doc[F("motor")].to<JsonObject>();
        engine.toJSON(motorObj);

        JsonObject batteryObj = doc[F("b")].to<JsonObject>();
        battery.toJson(batteryObj);

        JsonObject receiverObj = doc["receiver"].to<JsonObject>();
        flysky.toJson(receiverObj);
        JsonArray inaArray = doc["ina3221"].to<JsonArray>();
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            JsonObject chObj = inaArray.add<JsonObject>();
            chObj["channel"] = ch + 1;
            chObj["bus"] = ina3221.getBusVoltage(ch);
            chObj["shunt"] = ina3221.getShuntVoltage(ch);
            chObj["current"] = ina3221.getCurrentAmps(ch);
        }

        JsonArray inaLowArray = doc["ina3221_low"].to<JsonArray>();
        for (uint8_t ch = 0; ch < 3; ch++)
        {
            JsonObject chObj = inaLowArray.add<JsonObject>();
            chObj["channel"] = ch + 1;
            chObj["bus"] = ina3221_low.getBusVoltage(ch);
            chObj["shunt"] = ina3221_low.getShuntVoltage(ch);
            chObj["current"] = ina3221_low.getCurrentAmps(ch);
        }

        JsonArray logs = doc[F("logs")].to<JsonArray>();
        for (const auto &logEntry : logBuffer)
        {
            logs.add(sanitizeLog(logEntry));
        }

        String result;
        serializeJson(doc, result);
        return result;
    }
    String sanitizeLog(const String &str)
    {
        String cleaned = str;
        cleaned.replace("\r", "");
        cleaned.replace("\n", "");
        cleaned.replace("\x1B", "");
        return cleaned;
    }
    void printStatus()
    {
        // float v = 0, a = 0;
        // sensor.read(v, a);
        // addLog(String("Motor1: ") + temps.get(MOTOR1) + " \t-  Motor 2 " + temps.get(MOTOR2) + " \u00B0C" + String("Radiator: ") + temps.get(MOTOR_RAD) + " \tOil " + temps.get(OIL) + " \u00B0C \tAmbient: " + temps.get(ENV) + " \u00B0C");
        // addLog(String("Battery voltage: ") + battery.getVoltage() + " V \tRaw ADC value: " + battery.getRaw() + " \t" + battery.getMillivolts() + " mV - BEC in V: " + v + " V\tI: " + a + " A");
    }

    void checkFailsafeTransition()
    {
        bool currentState = flysky.transmitter_on;

        if (lastTransmitterState && !currentState && !failsafeTriggered)
        {
            addLog(" ! FAILSAFE detected: switching to fallback mode");
            failsafeTriggered = true;

            // Тут вызывай метод переключения в нужный режим
            // Например:
            enterFailsafeMode();
        }

        if (currentState && failsafeTriggered)
        {
            addLog(" ! Transmitter reconnected");
            failsafeTriggered = false;
            exitFailsafeMode();
        }

        lastTransmitterState = currentState;
    }

    void enterFailsafeMode()
    {
        addLog(" ! Entering autonomous mode...");

        engine.setState(engine.MOTOR_STOP); // Останавливаем моторы
    }

    void exitFailsafeMode()
    {
        addLog(" ! Returning to manual control");
    }

    void addLog(const String &msg) override
    {
        Serial.println(msg);
        if (logBuffer.size() >= logCapacity)
        {
            logBuffer.erase(logBuffer.begin());
        }
        logBuffer.push_back(msg);
    }
    String getMotorConfigJson()
    {
        Preferences prefs;
        prefs.begin("boatcfg", true);
        String leftEsc = prefs.getString("l_esc", "uni");
        String leftAddr = prefs.getString("l_addr", "");
        String rightEsc = prefs.getString("r_esc", "uni");
        String rightAddr = prefs.getString("r_addr", "");
        prefs.end();

        JsonDocument doc;
        doc["left"]["esc"] = leftEsc;
        doc["left"]["sensor"] = leftAddr;
        doc["right"]["esc"] = rightEsc;
        doc["right"]["sensor"] = rightAddr;

        String result;
        serializeJson(doc, result);
        return result;
    }

    void setMotorConfigJson(const String &json)
    {
        flysky.stopIBus(); // Останавливаем iBUS, чтобы избежать конфликтов при записи в память
        Preferences prefs;
        Serial.println("Setting motor config from JSON: " + json);
        prefs.begin("boatcfg", false);
        JsonDocument doc;
        deserializeJson(doc, json);
        Serial.println("Disabled putting motor config to preferences");
        prefs.putString("l_esc", doc["left"]["esc"].as<String>());
        prefs.putString("l_addr", doc["left"]["sensor"].as<String>());
        prefs.putString("r_esc", doc["right"]["esc"].as<String>());
        prefs.putString("r_addr", doc["right"]["sensor"].as<String>());
        prefs.end();
        flysky.startIBus(); // Запускаем iBUS снова
        Serial.println("Finish");
    }

    String getMotorTempsJson()
    {
        Preferences prefs;
        prefs.begin("boatcfg", true);
        String leftAddr = prefs.getString("l_addr", "");
        String rightAddr = prefs.getString("r_addr", "");
        prefs.end();

        float leftTemp = temps.getByAddressString(leftAddr);
        float rightTemp = temps.getByAddressString(rightAddr);

        JsonDocument doc;
        doc["left"] = isnan(leftTemp) ? -127.0 : leftTemp;
        doc["right"] = isnan(rightTemp) ? -127.0 : rightTemp;

        String result;
        serializeJson(doc, result);
        return result;
    }

    String getAllSensorAddressesJson()
    {
        JsonDocument doc;
        JsonArray arr = doc.to<JsonArray>();
        for (const auto &addr : temps.getAllAddresses())
        {
            arr.add(addr);
        }
        String result;
        serializeJson(doc, result);
        return result;
    }

    void sensorsPowerOn()
    {
        addLog("Start Sensors powered on");
        pinMode(VextCtrl, OUTPUT);
        digitalWrite(VextCtrl, LOW);
        addLog("Sensors powered on");
    }

    void sensorsPowerOff()
    {
        addLog("Start Sensors powered off");
        pinMode(VextCtrl, OUTPUT);
        digitalWrite(VextCtrl, HIGH);
        addLog("Sensors powered off");
    }

    void setThrottleLimit(int limit)
    {
        int pwmLimit = PWM_MIN;
        if (limit < 1250)
        {
            pwmLimit = PWM_MIN + 0.20 * (PWM_MAX - PWM_MIN); // 20%
        }
        else if (limit < 1750)
        {
            pwmLimit = PWM_MIN + 0.50 * (PWM_MAX - PWM_MIN); // 50%
        }
        else
        {
            pwmLimit = PWM_MIN + 0.80 * (PWM_MAX - PWM_MIN); // 80%
        }

        engine.setLimit(pwmLimit);
    }

private:
    String inputBuffer;
    uint8_t nextPacketId = 0;
    // Приватные члены класса, если нужны
    // Например, для хранения состояния или вспомогательных функций
    bool lastTransmitterState = true; // было ли соединение ранее
    bool failsafeTriggered = false;   // уже обработали событие потери
    static constexpr size_t logCapacity = 90;
    std::vector<String> logBuffer;

    bool failsafeActiveIbus = false; // Активен ли режим failsafe
    unsigned long lastPingSent = 0;
    uint8_t currentProfileIndex = 3;
    unsigned long lastAdaptiveSwitchTime = 0;
    const unsigned long switchInterval = 8011;
};
