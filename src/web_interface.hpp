// web_interface.hpp
#pragma once
#include <WiFi.h>
#include <WebServer.h>
#include <Update.h>
#include <SPIFFS.h>

#ifdef USE_MDNS
#include <ESPmDNS.h>
#endif

#include "boat.hpp"
#include "settings.h"

class WebInterface
{
public:
  WebInterface(Boat &boat) : server(80), boatRef(boat) {}

  void begin()
  {

    WiFi.mode(WIFI_AP);
    WiFi.softAP(WIFI_SSID, WIFI_PASS);
    delay(100);
    Serial.print("AP IP address: ");
    delay(100);
    Serial.println(WiFi.softAPIP());

#ifdef USE_MDNS
    if (MDNS.begin("boat"))
    {
      boatRef.addLog("mDNS responder started: http://boat.local");
    }
    else
    {
      boatRef.addLog("mDNS setup failed");
    }
#endif
    static bool updateInProgress = false;

    server.on("/api/reboot", HTTP_POST, [this]() {
        server.send(200, "text/plain", "Rebooting...");
        delay(300);
        ESP.restart();
    });


    server.on("/", HTTP_GET, [this]()
              { server.send(200, "text/html", getMainPage()); });

    server.serveStatic("/settings", SPIFFS, "/settings.html");

    server.on("/api/config", HTTP_GET, [this]()
              {
        String json = boatRef.getMotorConfigJson();
        server.send(200, "application/json", json); });

    server.on("/api/config", HTTP_POST, [this]()
              {
      if (!server.hasArg("plain")) {
        server.send(400, "text/plain", "Missing body");
        return;
      }
        String body = server.arg("plain");
        boatRef.setMotorConfigJson(body);
        server.send(200, "text/plain", "OK"); });

    server.on("/api/tempsensors", HTTP_GET, [this]()
              {
        String list = boatRef.getAllSensorAddressesJson();

        server.send(200, "application/json", list); });

    server.on("/api/temps", HTTP_GET, [this]()
              {
        String temps = boatRef.getMotorTempsJson();  // вернёт { "left": 28.1, "right": 29.2 }
        server.send(200, "application/json", temps); });

    server.on("/status", HTTP_GET, [this]()
              { server.send(200, "application/json", boatRef.getStatusJson()); });

    server.on("/cmd", HTTP_POST, [this]()
              {
        if (!server.hasArg("plain")) {
            server.send(400, "text/plain", "Missing body");
            return;
        }
        String cmd = server.arg("plain");
        boatRef.parser.processLine(cmd);
        server.send(200, "text/plain", "OK: " + cmd); });

    server.on("/update", HTTP_POST, [this]()
              {
            server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
            delay(200); // предотвращает ERR_CONNECTION_RESET
            ESP.restart(); }, [this]()
              {
            HTTPUpload& upload = server.upload();

            // локальная переменная, запоминающая, вызывали ли stop
            static bool stopped = false;
            if (!updateInProgress) {
                boatRef.addLog("Firmware update started, stopping background tasks...");
                boatRef.temps.stop();
                boatRef.flysky.stop(); // останавливаем iBUS
                SPI.end(); // освобождает SPI-пины
                updateInProgress = true;
                delay(10);
                // Ждём, пока задача точно остановится
                boatRef.addLog("Continue...Wire.end");
                Wire.end(); // освобождает I2C-пины
                delay(10);
                boatRef.addLog("Continue...");
            }

            if (upload.status == UPLOAD_FILE_START) {

                Serial.printf("Update: %s\n", upload.filename.c_str());

                if (!Update.begin()) {
                    Update.printError(Serial);
                } else {
                    Update.onProgress([](size_t done, size_t total) {
                        Serial.printf("Updating: %.1f%% (%u/%u bytes)\n", (float)done / total * 100, done, total);
                    });
                }

            } else if (upload.status == UPLOAD_FILE_WRITE) {
                if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                    Update.printError(Serial);
                }

            } else if (upload.status == UPLOAD_FILE_END) {
                if (Update.end(true)) {
                    Serial.printf("Update Success: %u bytes\n", upload.totalSize);
                } else {
                    Update.printError(Serial);
                }
            } });
    server.on("/update_spiffs", HTTP_POST, [this]()
              {
        server.send(200, "text/plain", Update.hasError() ? "FAIL" : "OK");
        delay(20);
        ESP.restart(); }, [this]()
              {
        HTTPUpload& upload = server.upload();

        if (upload.status == UPLOAD_FILE_START) {
            Serial.printf("SPIFFS Update Start: %s\n", upload.filename.c_str());
            if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {  // не U_SPIFFS!
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_WRITE) {
            if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
                Update.printError(Serial);
            }
        } else if (upload.status == UPLOAD_FILE_END) {
            if (Update.end(true)) {
                Serial.printf("SPIFFS Update Success: %u bytes\n", upload.totalSize);
            } else {
                Update.printError(Serial);
            }
        } });
    server.on("/api/gnss", HTTP_GET, [this]()
              { server.send(200, "application/json", boatRef.gnss.getStatusJson()); });

    server.on("/api/gnss/on", HTTP_POST, [this]()
              {
      boatRef.gnss.enable();
      server.send(200, "text/plain", "GNSS turned ON"); });

    server.on("/api/gnss/off", HTTP_POST, [this]()
              {
      boatRef.gnss.disable();
      server.send(200, "text/plain", "GNSS turned OFF"); });

    server.on("/api/gnss/psm", HTTP_POST, [this]()
              {
      boatRef.gnss.enablePSM();
      server.send(200, "text/plain", "GNSS Power Save Mode Enabled"); });
    server.on("/api/flysky/on", HTTP_POST, [this]()
              {
  boatRef.flysky.powerOn();
  server.send(200, "text/plain", "FlySky power ON"); });

    server.on("/api/flysky/off", HTTP_POST, [this]()
              {
  boatRef.flysky.powerOff();
  server.send(200, "text/plain", "FlySky power OFF"); });
    server.begin();
  }

  void handle()
  {
    server.handleClient();
  }

private:
  WebServer server;
  Boat &boatRef;

  String getMainPage()
  {
    return R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css" />
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<head>
  <meta charset="UTF-8">
  <title>Boat Dashboard</title>
  <style>
    #map {
    height: 300px;
    width: 100%;
    border-radius: 8px;
    margin-top: 10px;
  }
    body { font-family: sans-serif; margin: 0; padding: 0; background: #f0f2f5; }
    h2 { margin: 0.5em 0; }
    .card {
      background: #fff;
      margin: 1em;
      padding: 1em;
      border-radius: 8px;
      box-shadow: 0 2px 4px rgba(0,0,0,0.1);
    }
    .grid {
      display: flex;
      flex-wrap: wrap;
      gap: 1em;
    }
    .col {
      flex: 1 1 calc(33% - 2em);
      min-width: 250px;
    }
    .progress-bar {
      background: #eee;
      border-radius: 4px;
      overflow: hidden;
      height: 14px;
      margin-top: 4px;
    }
    .progress-bar-inner {
      background: #4caf50;
      height: 100%;
    }
    .badge {
      display: inline-block;
      padding: 2px 6px;
      border-radius: 4px;
      font-size: 12px;
      color: #fff;
      background: #2196f3;
    }
    .badge.off { background: #f44336; }
    .badge.red { background: #f44336; }
.badge.orange { background: #ff9800; }
.badge.green { background: #4caf50; }
.badge.gray { background: #9e9e9e; }

  </style>
</head>
<body>
<script>
function toggleGNSS(action) {
  fetch(`/api/gnss/${action}`, { method: 'POST' })
    .then(res => res.text())
    .then(msg => alert(msg))
    .catch(err => alert("Ошибка: " + err));
}
function toggleFlySky(action) {
  fetch(`/api/flysky/${action}`, {
    method: 'POST'
  })
  .then(res => res.text())
  .then(txt => alert(txt))
  .catch(err => alert("Ошибка: " + err));
}
</script>


<div class="card col">
  <h2>Управление</h2>
  <input id="cmdInput" placeholder="Команда (например: P 60)" style="width: 80%;">
  <button onclick="sendCommand()">Отправить</button>
</div>
<div class="card col">
  <h2>GNSS</h2>
  <button class="btn" onclick="toggleGNSS('on')">Включить GNSS</button>
  <button class="btn" onclick="toggleGNSS('off')">Выключить GNSS</button>
  <button class="btn" onclick="toggleGNSS('psm')">Включить PSM</button>
    <h2>FlySky iBUS</h2>
  <button class="btn" onclick="toggleFlySky('on')">Включить FlySky</button>
  <button class="btn" onclick="toggleFlySky('off')">Выключить FlySky</button>
</div>


<div class="card col">
  <h2>Прошивка (FOTA)</h2>
<input type="file" id="fwFile">
<button onclick="uploadFirmware()">Обновить прошивку</button>
<div id="otaStatus" style="margin-top:10px;"></div>
<div style="background:#ddd; height:10px; width:100%; border-radius:4px; overflow:hidden;">
  <div id="otaProgress" style="height:100%; width:0%; background:#4caf50;"></div>
</div>
</div>


<script>
function uploadFirmware() {
  const fileInput = document.getElementById("fwFile");
  const status = document.getElementById("otaStatus");
  const bar = document.getElementById("otaProgress");

  if (!fileInput.files.length) {
    alert("Выберите файл прошивки");
    return;
  }

  const file = fileInput.files[0];
  const xhr = new XMLHttpRequest();
  xhr.open("POST", "/update", true);

  xhr.upload.onprogress = function(e) {
    if (e.lengthComputable) {
      const percent = Math.round(e.loaded / e.total * 100);
      bar.style.width = percent + "%";
      status.innerText = `Загрузка: ${percent}%`;
    }
  };

  xhr.onload = function() {
    if (xhr.status === 200) {
      status.innerText = "✅ Обновление завершено. Перезагрузка...";
    } else {
      status.innerText = "❌ Ошибка загрузки";
    }
  };

  const form = new FormData();
  form.append("update", file);
  xhr.send(form);
}


  const form = document.querySelector("form");
  form.onsubmit = () => {
    alert("Загрузка прошивки. Пожалуйста, не закрывайте страницу и подождите...");
  }

  async function sendCommand() {
    const cmd = document.getElementById("cmdInput").value;
    if (!cmd) return alert("Введите команду!");
    try {
      const res = await fetch("/cmd", {
        method: "POST",
        headers: { "Content-Type": "text/plain" },
        body: cmd
      });
      const text = await res.text();
      alert(text);
    } catch (err) {
      alert("Ошибка: " + err);
    }
  }
</script>

<div class="grid" id="dashboard"></div>

<script>
// Добавь в JavaScript выше fetchData():
const INA_CHANNEL_LIMITS = {
  ina3221: {
    1: { minVoltage: 4.9, maxVoltage: 5.3, maxCurrent: 0.350 },
    2: { minVoltage: 9, maxVoltage: 18.0, maxCurrent: 1.6 },
    3: { minVoltage: 9, maxVoltage: 18.0, maxCurrent: 1.6 }
  },
  ina3221_low: {
    1: { minVoltage: 3.1, maxVoltage: 5.0,  maxCurrent: 0.09 },
    2: { minVoltage: 4.81, maxVoltage: 5.2, maxCurrent: 0.1 },
    3: { minVoltage: 4.8, maxVoltage: 5.0,  maxCurrent: 1.6 }
  }
};

function colorize(value, min, max) {
  if (value < min || value > max) return '#f44336'; // красный
  const delta = 0.1 * (max - min);
  if (value < min + delta || value > max - delta) return '#ffc107'; // жёлтый
  return '#4caf50'; // зелёный
}

function currentBar(value, limits) {
  const percent = Math.min(100, (value / limits.maxCurrent) * 100);
  const color = colorize(value, 0, limits.maxCurrent);
  return `
    <div class="progress-bar">
      <div class="progress-bar-inner" style="width:${percent.toFixed(1)}%; background:${color};"></div>
    </div>
    <small>${value.toFixed(3)} A</small>
  `;
}



function renderINA(title, arr, limitSetName) {
  const limitsMap = INA_CHANNEL_LIMITS[limitSetName];
  return `
    <h3>${title}</h3>
    <table style="width:100%; font-size: 14px;">
      <tr><th>Канал</th><th>Напряжение</th><th>Шунт</th><th>Ток</th></tr>
      ${arr.map(ch => {
        const lim = limitsMap[ch.channel] || { minVoltage: 0, maxVoltage: 20, maxCurrent: 1 };
        const vColor = colorize(ch.bus, lim.minVoltage, lim.maxVoltage);
        return `
          <tr>
            <td>CH${ch.channel}</td>
            <td style="color:${vColor}">${ch.bus.toFixed(2)} V</td>
            <td>${ch.shunt.toFixed(3)} V</td>
            <td>${currentBar(ch.current, lim)}</td>
          </tr>
        `;
      }).join("")}
    </table>`;
}





  async function fetchData() {
    const res = await fetch('/status');
    const data = await res.json();
    const el = document.getElementById("dashboard");
    el.innerHTML = '';

    let map, marker;

  function updateMap(lat, lon) {
    const lonOffset = 0.009; // сместим карту влево, чтобы маркер оказался правее

    if (!map) {
      // Инициализация карты
      map = L.map('map').setView([lat, lon + lonOffset], 13);

      // Подключаем тайлы
      L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
        attribution: 'Aboat'
      }).addTo(map);

      // Добавляем маркер
      marker = L.marker([lat, lon]).addTo(map)
        .bindPopup('Позиция лодки')
        .openPopup();
    } else {
      // Обновление позиции маркера
      marker.setLatLng([lat, lon]);
    }
  }




    function card(title, body) {
      const div = document.createElement('div');
      div.className = 'card col';
      div.innerHTML = `<h2>${title}</h2>${body}`;
      el.appendChild(div);
    }
    function currentColor(value) {
      if (value > 1.0) return "#f44336";  // Red
      if (value > 0.5) return "#ffc107";  // Yellow
      return "#4caf50";                  // Green
    }

    const tempSensors = ["motor1", "motor2", "radiator", "oil", "ambient"];
    const tempHTML = tempSensors.map(k => {
    const v = data.temperature[k];
    const color = getTempColor(v);
    const value = v === -127 ? "нет данных" : `${v.toFixed(1)} °C`;
    return `<p>${k}: <span class="badge" style="background:${color}">${value}</span></p>`;
  }).join("");

      // Внутри fetchData(), в конец:
card("INA3221", renderINA("INA3221", data.ina3221, "ina3221"));
card("INA3221 LOW", renderINA("INA3221 LOW", data.ina3221_low, "ina3221_low"));



    card("Температура", tempHTML);
    let batColor = "green";
    if (data.battery.voltage < 3.4) batColor = "red";
    else if (data.battery.voltage < 3.6) batColor = "orange";

    card("Battery", `
      <p>Напряжение: <span class="badge" style="background:${batColor}">${data.battery.voltage.toFixed(2)} V</span></p>
      <p>Сырые данные: ${data.battery.raw.toFixed(2)}</p>
      <p>mV: ${data.battery.millivolts.toFixed(0)} mV</p>
    `);

    card("System", `
      <p>Reset: ${data.system.reset_reason}</p>
      <p>Wakeup: ${data.system.wakeup_reason}</p>
      <p>Uptime: ${data.system.uptime_sec.toFixed(1)} сек</p>
      <p>IDF: ${data.system.idf_version}</p>
    `);

    card("Power", `
      <p>Напряжение: ${data.voltage.toFixed(2)} V</p>
      <p>Ток: ${data.current.toFixed(3)} A</p>
      <p>Battery: ${data.battery.voltage.toFixed(5)} V</p>
      <p>Battery raw: ${data.battery.raw.toFixed(5)} V</p>
      <p>Battery: millivolts ${data.battery.millivolts.toFixed(6)} V</p>
    `);
    // card("INA3221", renderINA("INA3221", data.ina3221));
    // card("INA3221 L", renderINA("INA3221 L", data.ina3221_low));

    const temps = Object.entries(data.temperature).map(([k, v]) =>
      `<p>${k}: ${v !== -127 ? v.toFixed(1) + " °C" : "нет данных"}</p>`).join("");

card("GNSS", `
  <p>Lat: ${data.gnss.lat.toFixed(6)} / Lon: ${data.gnss.lon.toFixed(6)} / Alt: ${data.gnss.alt.toFixed(1)} м</p>
  <p>Спутники: ${data.gnss.sats} / Speed: ${data.gnss.speed} / Fix: ${data.gnss.fix ? "Yes" : "No"}</p>
  <p>Date: ${data.gnss.date} - Time: ${data.gnss.time}</p>
  <div id="map"></div>
`);
    updateMap(data.gnss.lat, data.gnss.lon);


    card("IMU", `
      <p>Roll: ${data.imu.roll.toFixed(1)}°- Pitch: ${data.imu.pitch.toFixed(1)}° - Yaw: ${data.imu.yaw.toFixed(1)}°</p>
    `);

    card("Oil Pump", `
      <p>Статус: <span class="badge ${data.oil_pump.enabled ? '' : 'off'}">${data.oil_pump.enabled ? 'Вкл' : 'Выкл'}</span></p>
      <div class="progress-bar"><div class="progress-bar-inner" style="width:${data.oil_pump.speed_percent}%;"></div></div>
      <p>${data.oil_pump.speed_percent}%</p>
    `);

    card("Rudder", `
      <p>Текущий угол: ${data.rudder.current} %</p>
      <p>Целевой угол: ${data.rudder.target} %</p>
      <p>CPWM: ${data.rudder.cPwm} TPWM: ${data.rudder.tPwm}</p>
    `);

    card(`Motors (${data.motor.state})`, `
      <p>Left Motor: ${data.motor.left.current} → ${data.motor.left.target}</p>
      <p>Right Motor: ${data.motor.right.current} → ${data.motor.right.target}</p>
    `);

    card("FlySky Receiver", `
      <p>Питание: <span class="badge ${data.receiver.power ? '' : 'off'}">${data.receiver.power ? 'Вкл' : 'Выкл'}</span></p>
      <p>Связь с приёмником: <span class="badge ${data.receiver.receiver_alive ? '' : 'off'}">${data.receiver.receiver_alive ? 'OK' : 'Нет сигнала'}</span> Связь с передатчиком: <span class="badge ${data.receiver.transmitter_on ? '' : 'off'}">${data.receiver.transmitter_on ? 'OK' : 'Нет сигнала'}</span></p>
      <div class="progress-bar">
          <div class="progress-bar-inner" style="width:${(data.receiver.channels[2] - 1000) / 10}%;"></div>
      </div>

          ${data.receiver.channels.map((v, i) => `<strong>CH${i + 1}</strong>: ${v}&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;`).join("")}
      <p><small>Пакетов: ${data.receiver.ibus_packets}</small> - <small>Сенсоры: ${data.receiver.ibus_sensor_reads}</small> - <small>DISCOVER: ${data.receiver.ibus_discover}</small></p>
    `);

    card("Логи", `
      <pre style="white-space: pre-wrap; font-size: 12px; background: #111; color: #0f0; padding: 10px; border-radius: 5px; max-height: 200px; overflow-y: auto;">
${data.logs.slice().reverse().join("\n")}
      </pre>
    `);

    
  }


  function getTempColor(value) {
  if (value < 0) return 'gray';         // сенсор отвалился
  if (value > 75) return 'red';
  if (value > 55) return 'orange';
  return 'green';
}


  setInterval(fetchData, 4000);
  window.onload = fetchData;
</script>
</body>
</html>
)rawliteral";
  }
};
