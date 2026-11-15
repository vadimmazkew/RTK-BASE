#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <Preferences.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "secrets.h"

#define FW_VERSION "1.3.0"
#define RXD2 16  // К пину TX1 платы ZED-F9P (URTK1.2)
#define TXD2 17  // К пину RX1 платы ZED-F9P (URTK1.2)
#define MAX_LOCAL_CLIENTS 3

HardwareSerial RTKSerial(2);
SFE_UBLOX_GNSS_SERIAL myGNSS;
Preferences prefs;

WiFiClient cloudCasterClient;
WiFiServer localNtripServer(LOCAL_NTRIP_PORT);
WiFiClient localRoverClients[MAX_LOCAL_CLIENTS];
WebServer webServer(80);
HTTPUpdateServer httpUpdater;

// === Настройки в энергонезависимой памяти NVS ===
String wifiSsid     = WIFI_SSID;
String wifiPass     = WIFI_PASSWORD;
bool cloudEnabled   = ENABLE_CLOUD_CASTER;
String casterHost   = CASTER_HOST;
uint16_t casterPort = CASTER_PORT;
String casterMount  = MOUNTPOINT;
String casterPass   = SOURCE_PASSWORD;

bool useFixedBase   = false;
double fixedLatDeg  = 52.1115330;     // Пинск
double fixedLonDeg  = 26.1004610;
double fixedHeightM = 145.000;
uint32_t svinMinDur = 300;
float svinAccLimitM = 2.00;

// === Состояние сети и телеметрии ===
bool isApMode = false;
volatile uint32_t rtcmBytesGenerated = 0;
uint32_t last5SecBytes = 0;
uint8_t currentSIV = 0;
uint8_t currentFixType = 0;
double liveLatDeg = 0.0;
double liveLonDeg = 0.0;
double liveHeightM = 0.0;
bool svinActive = false;
bool svinValid = false;
uint32_t svinObsTime = 0;
float svinMeanAcc = 0.0;
bool f9pConnected = false;

// 1. Загрузка и сохранение всех параметров в NVS
void loadSettings() {
  prefs.begin("rtk-cfg", true);
  wifiSsid      = prefs.getString("w_ssid", WIFI_SSID);
  wifiPass      = prefs.getString("w_pass", WIFI_PASSWORD);
  cloudEnabled  = prefs.getBool("c_en", ENABLE_CLOUD_CASTER);
  casterHost    = prefs.getString("c_host", CASTER_HOST);
  casterPort    = prefs.getUShort("c_port", CASTER_PORT);
  casterMount   = prefs.getString("c_mnt", MOUNTPOINT);
  casterPass    = prefs.getString("c_pass", SOURCE_PASSWORD);

  useFixedBase  = prefs.getBool("fixed", false);
  fixedLatDeg   = prefs.getDouble("lat", 52.1115330);
  fixedLonDeg   = prefs.getDouble("lon", 26.1004610);
  fixedHeightM  = prefs.getDouble("hgt", 145.000);
  svinMinDur    = prefs.getUInt("dur", 300);
  svinAccLimitM = prefs.getFloat("acc", 2.00f);
  prefs.end();
}

void saveSettings() {
  prefs.begin("rtk-cfg", false);
  prefs.putString("w_ssid", wifiSsid);
  prefs.putString("w_pass", wifiPass);
  prefs.putBool("c_en", cloudEnabled);
  prefs.putString("c_host", casterHost);
  prefs.putUShort("c_port", casterPort);
  prefs.putString("c_mnt", casterMount);
  prefs.putString("c_pass", casterPass);

  prefs.putBool("fixed", useFixedBase);
  prefs.putDouble("lat", fixedLatDeg);
  prefs.putDouble("lon", fixedLonDeg);
  prefs.putDouble("hgt", fixedHeightM);
  prefs.putUInt("dur", svinMinDur);
  prefs.putFloat("acc", svinAccLimitM);
  prefs.end();
}

uint8_t getActiveRoverCount() {
  uint8_t count = 0;
  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (localRoverClients[i] && localRoverClients[i].connected()) count++;
  }
  return count;
}

void broadcastRTCM(const uint8_t *data, size_t len) {
  if (len == 0) return;
  rtcmBytesGenerated += len;

  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (localRoverClients[i] && localRoverClients[i].connected()) {
      localRoverClients[i].write(data, len);
    }
  }
  if (cloudEnabled && !isApMode && cloudCasterClient.connected()) {
    cloudCasterClient.write(data, len);
  }
}

// 2. Применение режима координат (TMODE3) в ZED-F9P с точностью 0.1 мм
bool applyBaseModeToF9P() {
  if (!f9pConnected) return false;

  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR);

  if (useFixedBase) {
    int64_t lat_e9   = llround(fixedLatDeg * 1e9);
    int32_t lat_main = (int32_t)(lat_e9 / 100);
    int8_t  lat_hp   = (int8_t)(lat_e9 % 100);

    int64_t lon_e9   = llround(fixedLonDeg * 1e9);
    int32_t lon_main = (int32_t)(lon_e9 / 100);
    int8_t  lon_hp   = (int8_t)(lon_e9 % 100);

    int64_t hgt_mm_e1 = llround(fixedHeightM * 10000.0);
    int32_t hgt_cm    = (int32_t)(hgt_mm_e1 / 100);
    int8_t  hgt_hp    = (int8_t)(hgt_mm_e1 % 100);

    myGNSS.addCfgValset(UBLOX_CFG_TMODE_MODE, 2);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_POS_TYPE, 1);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LAT, lat_main);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LAT_HP, lat_hp);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LON, lon_main);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LON_HP, lon_hp);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_HEIGHT, hgt_cm);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_HEIGHT_HP, hgt_hp);
  } else {
    uint32_t accLimit01mm = (uint32_t)lround(svinAccLimitM * 10000.0f);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_MODE, 1);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_SVIN_MIN_DUR, svinMinDur);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, accLimit01mm);
  }

  return myGNSS.sendCfgValset();
}

// 3. Колбэки телеметрии ZED-F9P
void onPVTData(UBX_NAV_PVT_data_t *pvt) {
  if (pvt == nullptr) return;
  currentSIV = pvt->numSV;
  currentFixType = pvt->fixType;
  liveLatDeg = pvt->lat / 1e7;
  liveLonDeg = pvt->lon / 1e7;
  liveHeightM = pvt->height / 1000.0;
}

void onSVINData(UBX_NAV_SVIN_data_t *svin) {
  if (svin == nullptr) return;
  svinActive = (svin->active == 1);
  svinValid = (svin->valid == 1);
  svinObsTime = svin->dur;
  svinMeanAcc = svin->meanAcc / 10000.0f;
}

// 4. Двухпоточный демультиплексор UART2 (разделяет RTCM3 и UBX для телеметрии)
void processUARTStream() {
  static uint8_t rtcmBuf[1030];
  static uint16_t rtcmIdx = 0;
  static uint16_t rtcmExpectedLen = 0;
  static bool inRtcmFrame = false;

  while (RTKSerial.available() > 0) {
    uint8_t b = RTKSerial.read();

    if (!inRtcmFrame) {
      if (b == 0xD3) {
        inRtcmFrame = true;
        rtcmIdx = 0;
        rtcmBuf[rtcmIdx++] = b;
      } else {
        // Все не-RTCM байты (пакеты UBX-NAV-PVT и UBX-NAV-SVIN) отдаем в парсер SparkFun
        myGNSS.pushRawData(&b, 1);
      }
    } else {
      rtcmBuf[rtcmIdx++] = b;
      if (rtcmIdx == 3) {
        if ((rtcmBuf[1] & 0xFC) != 0x00) {
          // Оказался не RTCM — возвращаем накопленные 3 байта в UBX-парсер
          myGNSS.pushRawData(rtcmBuf, 3);
          inRtcmFrame = false;
          rtcmIdx = 0;
        } else {
          uint16_t payloadLen = ((uint16_t)(rtcmBuf[1] & 0x03) << 8) | rtcmBuf[2];
          rtcmExpectedLen = payloadLen + 6;
          if (rtcmExpectedLen > sizeof(rtcmBuf)) {
            inRtcmFrame = false;
            rtcmIdx = 0;
          }
        }
      } else if (rtcmIdx >= 3 && rtcmIdx == rtcmExpectedLen) {
        broadcastRTCM(rtcmBuf, rtcmExpectedLen);
        inRtcmFrame = false;
        rtcmIdx = 0;
      }
    }
  }
  myGNSS.checkCallbacks();
}

// 5. Настройка Wi-Fi с автоматическим откатом в точку доступа "wifi-iot" (192.168.4.1)
void startFallbackAP() {
  isApMode = true;
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);

  // Жестко фиксируем IP 192.168.4.1 для точки доступа wifi-iot
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  Serial.println("\n[Wi-Fi AP] Поднята собственная точка доступа!");
  Serial.printf("[Wi-Fi AP] Сеть: %s | Пароль: %s | Адрес: http://192.168.4.1\n", AP_SSID, AP_PASSWORD);
}

void initNetwork() {
  bool hasValidCreds = (wifiSsid.length() > 0 && wifiSsid != "YOUR_WIFI_SSID");

  if (hasValidCreds) {
    Serial.printf("\n[Wi-Fi] Подключение к роутеру %s", wifiSsid.c_str());
    WiFi.mode(WIFI_STA);

    // Фиксируем постоянный IP 192.168.0.200 в вашей домашней сети
    IPAddress staticIP(STATIC_IP_ADDR);
    IPAddress gateway(GATEWAY_IP_ADDR);
    IPAddress subnet(SUBNET_MASK);
    IPAddress dns(DNS_IP_ADDR);
    WiFi.config(staticIP, gateway, subnet, dns);

    WiFi.setAutoReconnect(true);
    WiFi.begin(wifiSsid.c_str(), wifiPass.c_str());

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 24) { // Ждем 12 секунд
      delay(500);
      Serial.print(".");
      attempts++;
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    isApMode = false;
    Serial.printf("\n[OK] Подключено к роутеру! Постоянный IP: http://%s\n",
                  WiFi.localIP().toString().c_str());
  } else {
    startFallbackAP(); // Если роутера нет рядом — поднимает wifi-iot (192.168.4.1)
  }

  MDNS.begin(OTA_HOSTNAME);
  localNtripServer.begin();
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// 6. Веб-панель управления с живым обновлением JSON и настройкой Wi-Fi
void setupWebMonitor() {
  httpUpdater.setup(&webServer, "/update", "admin", OTA_PASSWORD);

  // API для живого обновления данных без перезагрузки страницы
  webServer.on("/api/status", HTTP_GET, []() {
    String fixText = !f9pConnected ? "Ожидание ZED-F9P (UART)" :
                     (currentFixType == 5) ? "TIME (БАЗА АКТИВНА)" :
                     (currentFixType == 3) ? "3D FIX (Усреднение)" : "Поиск спутников";
    String svinText = svinValid ? "ГОТОВО" : (svinActive ? "Накопление..." : "Выкл / Fixed");
    String netInfo = isApMode ? ("Точка доступа: " + String(AP_SSID) + " (192.168.4.1)") :
                                ("Роутер: " + wifiSsid + " (" + WiFi.localIP().toString() + ", " + String(WiFi.RSSI()) + " dBm)");

    String json = "{";
    json += "\"f9p\":" + String(f9pConnected ? "true" : "false") + ",";
    json += "\"fixType\":" + String(currentFixType) + ",";
    json += "\"fixText\":\"" + fixText + "\",";
    json += "\"siv\":" + String(currentSIV) + ",";
    json += "\"svinText\":\"" + svinText + " (" + String(svinObsTime) + "с / " + String(svinMeanAcc, 2) + "м)\",";
    json += "\"liveCoords\":\"" + String(liveLatDeg, 7) + ", " + String(liveLonDeg, 7) + " (" + String(liveHeightM, 2) + "м)\",";
    json += "\"rovers\":" + String(getActiveRoverCount()) + ",";
    json += "\"bytes\":" + String(last5SecBytes) + ",";
    json += "\"net\":\"" + netInfo + "\"";
    json += "}";
    webServer.send(200, "application/json", json);
  });

  webServer.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>RTK База v" FW_VERSION "</title>"
      "<style>"
      "body{font-family:-apple-system,sans-serif;background:#121212;color:#eee;margin:0;padding:12px;}"
      ".card{background:#1e1e1e;padding:16px;border-radius:10px;max-width:480px;margin:0 auto 14px;box-shadow:0 4px 10px rgba(0,0,0,0.4);}"
      "h2,h3{margin-top:0;color:#64b5f6;}"
      ".row{display:flex;justify-content:space-between;margin:8px 0;border-bottom:1px solid #2c2c2c;padding-bottom:6px;font-size:14px;}"
      "label{display:block;margin-top:10px;font-size:13px;color:#aaa;}"
      "input,select{width:100%;padding:10px;margin-top:4px;background:#2b2b2b;border:1px solid #444;color:#fff;border-radius:6px;box-sizing:border-box;font-size:15px;}"
      ".btn{display:block;width:100%;padding:11px;margin-top:12px;border:none;border-radius:6px;font-size:14px;font-weight:bold;cursor:pointer;text-align:center;text-decoration:none;box-sizing:border-box;}"
      ".btn-save{background:#2e7d32;color:#fff;}"
      ".btn-copy{background:#1565c0;color:#fff;}"
      ".btn-preset{background:#37474f;color:#fff;width:48%;display:inline-block;margin-top:8px;}"
      ".btn-refresh{background:#424242;color:#fff;}"
      "</style>"
      "<script>"
      "function setPreset(lat,lon,hgt){"
      "document.getElementById('lat').value=lat;"
      "document.getElementById('lon').value=lon;"
      "document.getElementById('hgt').value=hgt;"
      "}"
      "setInterval(()=>{"
      "fetch('/api/status').then(r=>r.json()).then(d=>{"
      "document.getElementById('s_fix').innerText=d.fixText;"
      "document.getElementById('s_fix').style.color=(d.fixType==5?'#4caf50':(d.f9p?'#ff9800':'#ff5252'));"
      "document.getElementById('s_siv').innerText=d.siv+' шт.';"
      "document.getElementById('s_svin').innerText=d.svinText;"
      "document.getElementById('s_coords').innerText=d.liveCoords;"
      "document.getElementById('s_rovers').innerText=d.rovers+' шт.';"
      "document.getElementById('s_bytes').innerText=d.bytes+' байт';"
      "document.getElementById('s_net').innerText=d.net;"
      "}).catch(()=>{});"
      "}, 2000);"
      "</script></head><body>"

      // Карточка 1: Живая телеметрия
      "<div class='card'>"
      "<h2>📡 RTK База (" + String(LOCAL_MOUNTPOINT) + ") <small style='font-size:12px;color:#888;'>v" FW_VERSION "</small></h2>"
      "<div class='row'><span>Приемник:</span><b id='s_fix'>Загрузка...</b></div>"
      "<div class='row'><span>Спутников в работе:</span><b id='s_siv'>" + String(currentSIV) + " шт.</b></div>"
      "<div class='row'><span>Активный режим:</span><b>" + String(useFixedBase ? "FIXED (Постоянный)" : "SURVEY-IN (Авто)") + "</b></div>"
      "<div class='row'><span>Survey-In:</span><b id='s_svin'>-</b></div>"
      "<div class='row'><span>Текущие WGS-84:</span><b id='s_coords'>-</b></div>"
      "<div class='row'><span>Роверов (Порт " + String(LOCAL_NTRIP_PORT) + "):</span><b id='s_rovers'>0 шт.</b></div>"
      "<div class='row'><span>Поток RTCM3 (5с):</span><b id='s_bytes'>0 байт</b></div>"
      "<div class='row'><span>Сеть:</span><b id='s_net' style='font-size:12px;'>-</b></div>"
      "</div>"

      // Карточка 2: Координаты и режим базы
      "<div class='card'>"
      "<h3>⚙️ Координаты и режим базы</h3>"
      "<div style='display:flex;justify-content:space-between;'>"
      "<button type='button' class='btn btn-preset' onclick='setPreset(52.1115330,26.1004610,145.0)'>📍 Пинск</button>"
      "<button type='button' class='btn btn-preset' onclick='setPreset(52.1480380,26.1744910,146.0)'>🏡 Высокое</button>"
      "</div>"
      "<form action='/save_gnss' method='POST'>"
      "<label>Режим запуска базы:</label>"
      "<select name='mode'>"
      "<option value='0'" + String(!useFixedBase ? " selected" : "") + ">Survey-In (Автоусреднение на месте)</option>"
      "<option value='1'" + String(useFixedBase ? " selected" : "") + ">Fixed Mode (Жесткие координаты)</option>"
      "</select>"
      "<label>Широта WGS-84 (град):</label>"
      "<input id='lat' type='number' step='0.00000001' name='lat' value='" + String(fixedLatDeg, 8) + "' required>"
      "<label>Долгота WGS-84 (град):</label>"
      "<input id='lon' type='number' step='0.00000001' name='lon' value='" + String(fixedLonDeg, 8) + "' required>"
      "<label>Высота над эллипсоидом WGS-84 (м):</label>"
      "<input id='hgt' type='number' step='0.0001' name='hgt' value='" + String(fixedHeightM, 4) + "' required>"
      "<label>Время Survey-In (сек) / Допуск (м):</label>"
      "<div style='display:flex;gap:8px;'>"
      "<input type='number' name='dur' value='" + String(svinMinDur) + "' required>"
      "<input type='number' step='0.01' name='acc' value='" + String(svinAccLimitM, 2) + "' required>"
      "</div>"
      "<button type='submit' class='btn btn-save'>💾 Применить координаты в ZED-F9P</button>"
      "</form>"
      "<form action='/use_current' method='POST'>"
      "<button type='submit' class='btn btn-copy'>🎯 Зафиксировать текущие с неба в FIXED</button>"
      "</form>"
      "</div>"

      // Карточка 3: Настройки Wi-Fi и облака (с перезагрузкой)
      "<div class='card'>"
      "<h3>🌐 Сеть Wi-Fi и Кастер</h3>"
      "<form action='/save_net' method='POST'>"
      "<label>Имя домашнего/рабочего Wi-Fi (SSID):</label>"
      "<input type='text' name='w_ssid' value='" + wifiSsid + "' placeholder='Оставьте пустым для работы через wifi-iot'>"
      "<label>Пароль Wi-Fi:</label>"
      "<input type='password' name='w_pass' value='" + wifiPass + "'>"
      "<label>Дублировать поток во внешний NTRIP Caster:</label>"
      "<select name='c_en'>"
      "<option value='0'" + String(!cloudEnabled ? " selected" : "") + ">Выкл (Только прямой сервер на ESP32)</option>"
      "<option value='1'" + String(cloudEnabled ? " selected" : "") + ">Вкл (Отправлять на Emlid / свой сервер)</option>"
      "</select>"
      "<label>Хост / Порт кастера:</label>"
      "<div style='display:flex;gap:8px;'>"
      "<input type='text' name='c_host' value='" + casterHost + "'>"
      "<input type='number' name='c_port' value='" + String(casterPort) + "' style='width:35%;'>"
      "</div>"
      "<label>Точка монтирования / Пароль кастера:</label>"
      "<div style='display:flex;gap:8px;'>"
      "<input type='text' name='c_mnt' value='" + casterMount + "'>"
      "<input type='password' name='c_pass' value='" + casterPass + "'>"
      "</div>"
      "<button type='submit' class='btn btn-save'>🔄 Сохранить сеть и перезагрузить</button>"
      "</form>"
      "<a href='/update' class='btn btn-refresh' style='margin-top:16px;font-size:13px;'>📦 Веб-обновление прошивки (.bin)</a>"
      "</div></body></html>";

    webServer.send(200, "text/html", html);
  });

  webServer.on("/save_gnss", HTTP_POST, []() {
    if (webServer.hasArg("mode")) useFixedBase = (webServer.arg("mode").toInt() == 1);
    if (webServer.hasArg("lat"))  fixedLatDeg  = webServer.arg("lat").toDouble();
    if (webServer.hasArg("lon"))  fixedLonDeg  = webServer.arg("lon").toDouble();
    if (webServer.hasArg("hgt"))  fixedHeightM = webServer.arg("hgt").toDouble();
    if (webServer.hasArg("dur"))  svinMinDur   = webServer.arg("dur").toInt();
    if (webServer.hasArg("acc"))  svinAccLimitM = webServer.arg("acc").toFloat();

    saveSettings();
    applyBaseModeToF9P();
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/use_current", HTTP_POST, []() {
    if (liveLatDeg != 0.0 && liveLonDeg != 0.0) {
      fixedLatDeg = liveLatDeg;
      fixedLonDeg = liveLonDeg;
      fixedHeightM = liveHeightM;
      useFixedBase = true;
      saveSettings();
      applyBaseModeToF9P();
    }
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/save_net", HTTP_POST, []() {
    if (webServer.hasArg("w_ssid")) wifiSsid     = webServer.arg("w_ssid");
    if (webServer.hasArg("w_pass")) wifiPass     = webServer.arg("w_pass");
    if (webServer.hasArg("c_en"))   cloudEnabled = (webServer.arg("c_en").toInt() == 1);
    if (webServer.hasArg("c_host")) casterHost   = webServer.arg("c_host");
    if (webServer.hasArg("c_port")) casterPort   = webServer.arg("c_port").toInt();
    if (webServer.hasArg("c_mnt"))  casterMount  = webServer.arg("c_mnt");
    if (webServer.hasArg("c_pass")) casterPass   = webServer.arg("c_pass");

    saveSettings();
    webServer.send(200, "text/html", "<meta charset='utf-8'><h3>Настройки сохранены! Перезагрузка платы...</h3>");
    delay(800);
    ESP.restart();
  });

  webServer.begin();
}

// 7. Прямой NTRIP-сервер на порту 2101
void handleLocalNtripClients() {
  if (!localNtripServer.hasClient()) return;

  WiFiClient newClient = localNtripServer.accept();
  if (!newClient) return;

  unsigned long timeout = millis();
  while (!newClient.available() && millis() - timeout < 1500) delay(5);

  String request = "";
  if (newClient.available()) {
    request = newClient.readStringUntil('\n');
    request.trim();
  }

  if (request.startsWith("GET / ") || request.startsWith("GET / HTTP")) {
    String sourceTable = 
      "SOURCETABLE 200 OK\r\nContent-Type: text/plain\r\n\r\n"
      "STR;" + String(LOCAL_MOUNTPOINT) + ";Pinsk;RTCM 3.3;1005(1),1077(1),1087(1),1097(1),1127(1),1230(5);2;GPS+GLO+GAL+BDS;LOCAL;BLR;" +
      String(fixedLatDeg, 2) + ";" + String(fixedLonDeg, 2) + ";0;0;ESP32-F9P;none;N;N;9600;\r\nENDSOURCETABLE\r\n";
    newClient.print(sourceTable);
    delay(50);
    newClient.stop();
    return;
  }

  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (!localRoverClients[i] || !localRoverClients[i].connected()) {
      if (request.length() > 0) newClient.print("ICY 200 OK\r\n\r\n");
      localRoverClients[i] = newClient;
      return;
    }
  }
  newClient.stop();
}

// 8. Автоконфигурация ZED-F9P
bool configureF9PBase() {
  RTKSerial.setRxBufferSize(4096);
  RTKSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);

  if (!myGNSS.begin(RTKSerial)) {
    RTKSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);
    if (myGNSS.begin(RTKSerial)) {
      myGNSS.setSerialRate(115200, COM_PORT_UART1);
      delay(250);
      RTKSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
      if (!myGNSS.begin(RTKSerial)) return false;
    } else {
      return false;
    }
  }

  f9pConnected = true;
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR);

  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GPS_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GPS_L1CA_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GPS_L2C_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GAL_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GAL_E1_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GAL_E5B_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_BDS_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_BDS_B1_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_BDS_B2_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GLO_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GLO_L1_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_GLO_L2_ENA, 1);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_SBAS_ENA, 0);
  myGNSS.addCfgValset(UBLOX_CFG_SIGNAL_QZSS_ENA, 0);

  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_DYNMODEL, 2);
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINELEV, 10);
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINCNO, 15);
  myGNSS.addCfgValset(UBLOX_CFG_RATE_MEAS, 1000);

  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_NMEA, 0);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_RTCM3X, 1);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_UBX, 1);

  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 5);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 2);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART1, 2);

  if (myGNSS.sendCfgValset()) {
    myGNSS.setAutoPVTcallbackPtr(&onPVTData);
    myGNSS.setAutoNAVSVINcallbackPtr(&onSVINData);
    return applyBaseModeToF9P();
  }
  return false;
}

bool connectCloudCaster() {
  if (!cloudEnabled || isApMode || cloudCasterClient.connected()) return true;
  if (!cloudCasterClient.connect(casterHost.c_str(), casterPort)) return false;

  String authHeader = "SOURCE " + casterPass + " /" + casterMount + "\r\n" +
                      "Source-Agent: NTRIP ESP32-S3-F9P-Base/" FW_VERSION "\r\n\r\n";
  cloudCasterClient.print(authHeader);

  unsigned long timeout = millis();
  while (cloudCasterClient.available() == 0) {
    if (millis() - timeout > 4000) { cloudCasterClient.stop(); return false; }
    delay(10);
  }
  String resp = cloudCasterClient.readStringUntil('\n');
  if (resp.indexOf("200 OK") != -1) return true;
  cloudCasterClient.stop();
  return false;
}

void setup() {
  Serial.begin(115200);
  delay(1500);
  loadSettings();
  configureF9PBase();
  initNetwork();
  setupWebMonitor();
}

void loop() {
  static uint32_t lastReportTime = 0;
  static uint32_t lastReconnectAttempt = 0;
  static uint32_t lastF9PRetry = 0;

  ArduinoOTA.handle();
  webServer.handleClient();
  handleLocalNtripClients();

  if (f9pConnected) {
    processUARTStream();
  } else if (millis() - lastF9PRetry > 10000) {
    lastF9PRetry = millis();
    configureF9PBase();
  }

  if (millis() - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = millis();
    if (!isApMode && WiFi.status() != WL_CONNECTED) {
      WiFi.reconnect();
    }
    if (cloudEnabled && !isApMode && WiFi.status() == WL_CONNECTED && !cloudCasterClient.connected()) {
      connectCloudCaster();
    }
  }

  if (millis() - lastReportTime > 5000) {
    lastReportTime = millis();
    last5SecBytes = rtcmBytesGenerated;
    rtcmBytesGenerated = 0;

    Serial.printf("[v%s] Спутников: %2u | Режим: %s | Роверов: %u | RTCM3: %lu Б/5с | Адрес: http://%s\n",
                  FW_VERSION,
                  currentSIV,
                  useFixedBase ? "FIXED" : (svinValid ? "SVIN-OK" : "SVIN-RUN"),
                  getActiveRoverCount(),
                  (unsigned long)last5SecBytes,
                  isApMode ? "192.168.4.1 (AP: wifi-iot)" : WiFi.localIP().toString().c_str());
  }
}
