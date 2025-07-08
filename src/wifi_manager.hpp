#pragma once
#include <Arduino.h>
#include <WiFi.h>
#include <time.h>
#include "esp_wifi.h"
/*
 * WiFiManager — периодически включает WiFi, синхронизирует время и отключается.
 * Работает в отдельной задаче на Core 0.
 */
class WiFiManager
{
public:
  static void begin(const char *ssid,
                    const char *pass,
                    long gmtOffsetSeconds = 0,
                    uint32_t intervalMillis = 30 * 60 * 1000, // каждые 30 минут
                    const char *ntp1 = "pool.ntp.org",
                    const char *ntp2 = "time.nist.gov")
  {
    xTaskCreatePinnedToCore(task, "WiFi/NTP", 4096,
                            new Params{ssid, pass, gmtOffsetSeconds, intervalMillis, ntp1, ntp2},
                            1, nullptr, 0);
  }

private:
  struct Params
  {
    const char *ssid;
    const char *pass;
    long gmtOffset;
    uint32_t interval;
    const char *ntp1;
    const char *ntp2;
  };

  static void task(void *arg)
  {
    std::unique_ptr<Params> p{static_cast<Params *>(arg)};
    configTime(p->gmtOffset, 0, p->ntp1, p->ntp2);

    while (true)
    {
      Serial.println(F("\n[WiFiManager] Connecting to WiFi..."));

      WiFi.mode(WIFI_STA);

      // 💡 Улучшенная подготовка Wi-Fi
      WiFi.disconnect(true, true); // принудительный сброс и erase config
      delay(200);                  // немного больше задержки
      esp_wifi_stop();             // 🛑 сбросим стек Wi-Fi
      delay(200);
      esp_wifi_start(); // 🔁 запустим заново
      delay(200);

      WiFi.begin(p->ssid, p->pass);

      uint32_t startAttempt = millis();
      bool connected = false;

      for (int i = 0; i < 30; ++i)
      { // до 15 секунд, по 500 мс
        if (WiFi.status() == WL_CONNECTED)
        {
          connected = true;
          break;
        }
        Serial.print('.');
        vTaskDelay(pdMS_TO_TICKS(500));
      }

      if (connected)
      {
        Serial.printf("\n[WiFiManager] ✅ Connected: %s\n", WiFi.localIP().toString().c_str());

        // ждём синхронизацию
        time_t t0 = time(nullptr);
        for (int i = 0; i < 10 && t0 < 1600000000; ++i)
        {
          vTaskDelay(pdMS_TO_TICKS(1500));
          Serial.print('.');
          t0 = time(nullptr);
        }

        if (t0 > 1600000000)
        {
          Serial.println(F("\n[NTP] 🕒 Time synced"));
        }
        else
        {
          Serial.println(F("\n[NTP] ❌ Time sync failed"));
        }
      }
      else
      {
        Serial.println(F("\n[WiFiManager] ❌ Failed to connect to WiFi"));
      }

      // 📴 Выключаем WiFi до следующей попытки
      WiFi.disconnect(true, true);
      delay(100);
      WiFi.mode(WIFI_OFF);
      esp_wifi_stop(); // дополнительный сброс
      Serial.println(F("[WiFiManager] WiFi OFF"));

      // ⏱ Ждём следующей попытки
      vTaskDelay(pdMS_TO_TICKS(p->interval));
    }
  }
};
