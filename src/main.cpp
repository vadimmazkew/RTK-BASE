#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <Preferences.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "secrets.h"

#define FW_VERSION "1.2.1"
#define RXD2 16  // Подключить к TX1 платы ZED-F9P (URTK1.2)
#define TXD2 17  // Подключить к RX1 платы ZED-F9P (URTK1.2)
#define MAX_LOCAL_CLIENTS 3

HardwareSerial RTKSerial(2);
SFE_UBLOX_GNSS_SERIAL myGNSS;
Preferences prefs;

WiFiClient cloudCasterClient;
WiFiServer localNtripServer(LOCAL_NTRIP_PORT);
WiFiClient localRoverClients[MAX_LOCAL_CLIENTS];
WebServer webServer(80);
HTTPUpdateServer httpUpdater;

// === Настройки режима базы (хранятся в энергонезависимой памяти NVS) ===
bool useFixedBase = false;
double fixedLatDeg = 52.1115330;     // Базовая точка: Пинск
double fixedLonDeg = 26.1004610;
double fixedHeightM = 145.000;
uint32_t svinMinDur = 300;           // Время Survey-In (сек)
float svinAccLimitM = 2.00;          // Допуск Survey-In (м)

// === Переменные живой телеметрии ===
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

// 1. Работа с энергонезависимой памятью NVS
void loadSettings() {
  prefs.begin("rtk-cfg", true);
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

// Отправка отфильтрованного пакета RTCM3 всем роверам и в облако
void broadcastRTCM(const uint8_t *data, size_t len) {
  if (len == 0) return;
  rtcmBytesGenerated += len;

  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (localRoverClients[i] && localRoverClients[i].connected()) {
      localRoverClients[i].write(data, len);
    }
  }
  if (ENABLE_CLOUD_CASTER && cloudCasterClient.connected()) {
    cloudCasterClient.write(data, len);
  }
}

// 2. Передача координат и режима TMODE3 в ZED-F9P с точностью 0.1 мм (High Precision)
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

    myGNSS.addCfgValset(UBLOX_CFG_TMODE_MODE, 2);     // 2 = Fixed Mode
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_POS_TYPE, 1); // 1 = LLH (WGS-84)
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LAT, lat_main);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LAT_HP, lat_hp);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LON, lon_main);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_LON_HP, lon_hp);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_HEIGHT, hgt_cm);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_HEIGHT_HP, hgt_hp);
  } else {
    uint32_t accLimit01mm = (uint32_t)lround(svinAccLimitM * 10000.0f);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_MODE, 1);     // 1 = Survey-In
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_SVIN_MIN_DUR, svinMinDur);
    myGNSS.addCfgValset(UBLOX_CFG_TMODE_SVIN_ACC_LIMIT, accLimit01mm);
  }

  return myGNSS.sendCfgValset();
}

// 3. Колбэки телеметрии SparkFun v3 (UBX-NAV-PVT и UBX-NAV-SVIN)
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

// 4. Опрос ZED-F9P и пересылка потока RTCM3
// Чтобы не блокировать библиотеку SparkFun v3, мы переопределяем метод чтения или
// используем встроенный колбэк pushRawData / чтение через буфер.
// Самый надежный способ в v3.1.15: опрашиваем телеметрию раз в 2 секунды, а все
// остальное время пересылаем бинарный поток напрямую из UART2 с фильтрацией кадров RTCM3 (0xD3).
void processUARTStream() {
  static uint8_t rtcmBuf[1030];
  static uint16_t rtcmIdx = 0;
  static uint16_t rtcmExpectedLen = 0;
  static bool inRtcmFrame = false;

  while (RTKSerial.available() > 0) {
    uint8_t b = RTKSerial.read();

    if (!inRtcmFrame) {
      if (b == 0xD3) { // Стандартный заголовок пакета RTCM 3.x
        inRtcmFrame = true;
        rtcmIdx = 0;
        rtcmBuf[rtcmIdx++] = b;
      }
    } else {
      rtcmBuf[rtcmIdx++] = b;

      if (rtcmIdx == 3) {
        // Байты 1 и 2 содержат 10-битную длину полезной нагрузки (первые 6 бит должны быть 0)
        if ((rtcmBuf[1] & 0xFC) != 0x00) {
          inRtcmFrame = false; // Ложное совпадение 0xD3
          rtcmIdx = 0;
        } else {
          uint16_t payloadLen = ((uint16_t)(rtcmBuf[1] & 0x03) << 8) | rtcmBuf[2];
          rtcmExpectedLen = payloadLen + 6; // 3 байта заголовка + payload + 3 байта CRC24Q
          if (rtcmExpectedLen > sizeof(rtcmBuf)) {
            inRtcmFrame = false;
            rtcmIdx = 0;
          }
        }
      } else if (rtcmIdx >= 3 && rtcmIdx == rtcmExpectedLen) {
        // Полный пакет RTCM3 собран — мгновенно отправляем роверам!
        broadcastRTCM(rtcmBuf, rtcmExpectedLen);
        inRtcmFrame = false;
        rtcmIdx = 0;
      }
    }
  }
}

// 5. Веб-интерфейс управления + Веб-OTA (/update)
void setupWebMonitor() {
  httpUpdater.setup(&webServer, "/update", "admin", OTA_PASSWORD);

  webServer.on("/", HTTP_GET, []() {
    String fixBadge = !f9pConnected ? "<span style='color:#ff5252;'>Ожидание ZED-F9P (UART)</span>" :
                      (currentFixType == 5) ? "<span style='color:#4caf50;'>TIME (БАЗА АКТИВНА)</span>" :
                      (currentFixType == 3) ? "<span style='color:#ff9800;'>3D FIX (Усреднение)</span>" :
                                              "<span style='color:#f44336;'>Поиск спутников</span>";

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
      "</script></head><body>"

      "<div class='card'>"
      "<h2>📡 RTK База (" + String(LOCAL_MOUNTPOINT) + ") <small style='font-size:12px;color:#888;'>v" FW_VERSION "</small></h2>"
      "<div class='row'><span>Статус приемника:</span><b>" + fixBadge + "</b></div>"
      "<div class='row'><span>Спутников в работе:</span><b>" + String(currentSIV) + " шт.</b></div>"
      "<div class='row'><span>Активный режим:</span><b>" + String(useFixedBase ? "FIXED (Постоянный)" : "SURVEY-IN (Авто)") + "</b></div>"
      "<div class='row'><span>Survey-In статус:</span><b>" + String(svinValid ? "ГОТОВО" : (svinActive ? "Накопление..." : "Выкл")) +
      " (" + String(svinObsTime) + "с / " + String(svinMeanAcc, 2) + "м)</b></div>"
      "<div class='row'><span>Текущие WGS-84:</span><b>" + String(liveLatDeg, 7) + ", " + String(liveLonDeg, 7) + "</b></div>"
      "<div class='row'><span>Роверов на порту " + String(LOCAL_NTRIP_PORT) + ":</span><b>" + String(getActiveRoverCount()) + " шт.</b></div>"
      "<div class='row'><span>Поток RTCM3 (5 сек):</span><b>" + String(last5SecBytes) + " байт</b></div>"
      "<div class='row'><span>IP / Сигнал Wi-Fi:</span><b>" + WiFi.localIP().toString() + " (" + String(WiFi.RSSI()) + " dBm)</b></div>"
      "<a href='/' class='btn btn-refresh'>🔄 Обновить телеметрию</a>"
      "</div>"

      "<div class='card'>"
      "<h3>⚙️ Координаты и режим базы</h3>"
      "<div style='display:flex;justify-content:space-between;'>"
      "<button type='button' class='btn btn-preset' onclick='setPreset(52.1115330,26.1004610,145.0)'>📍 Пресет: Пинск</button>"
      "<button type='button' class='btn btn-preset' onclick='setPreset(52.1480380,26.1744910,146.0)'>🏡 Пресет: Высокое</button>"
      "</div>"
      "<form action='/save' method='POST'>"
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
      "<button type='submit' class='btn btn-save'>💾 Сохранить в память и применить</button>"
      "</form>"
      "<form action='/use_current' method='POST'>"
      "<button type='submit' class='btn btn-copy'>🎯 Зафиксировать текущие с неба в FIXED</button>"
      "</form>"
      "<a href='/update' class='btn btn-refresh' style='margin-top:16px;font-size:13px;'>📦 Веб-обновление прошивки (.bin)</a>"
      "</div></body></html>";

    webServer.send(200, "text/html", html);
  });

  webServer.on("/save", HTTP_POST, []() {
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

  webServer.begin();
}

// 6. Прямой NTRIP-сервер на порту 2101 (использует новый метод accept())
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

// 7. Настройка беспроводного обновления ArduinoOTA
void setupOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// 8. Полная автоконфигурация ZED-F9P при старте
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
      Serial.println("[GNSS] ZED-F9P не подключен. Работа в режиме веб-сервера.");
      return false;
    }
  }

  f9pConnected = true;
  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR);

  // Все 4 спутниковые системы + вторые частоты (включая BeiDou B2I)
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

  // Режим Stationary и фильтрация низких спутников
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_DYNMODEL, 2);
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINELEV, 10);
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINCNO, 15);
  myGNSS.addCfgValset(UBLOX_CFG_RATE_MEAS, 1000);

  // Чистый поток RTCM3 + UBX на UART1
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_NMEA, 0);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_RTCM3X, 1);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_UBX, 1);

  // Пакеты поправок MSM7
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 5);

  if (myGNSS.sendCfgValset()) {
    myGNSS.setAutoPVTcallbackPtr(&onPVTData);
    myGNSS.setAutoNAVSVINcallbackPtr(&onSVINData);
    return applyBaseModeToF9P();
  }
  return false;
}

void connectWiFi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint8_t attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    MDNS.begin(OTA_HOSTNAME);
    localNtripServer.begin();
    setupOTA();
    setupWebMonitor();
    Serial.printf("\n[OK] Веб-панель: http://%s (или http://%s.local)\n", 
                  WiFi.localIP().toString().c_str(), OTA_HOSTNAME);
  }
}

bool connectCloudCaster() {
  if (!ENABLE_CLOUD_CASTER || cloudCasterClient.connected()) return true;
  if (!cloudCasterClient.connect(CASTER_HOST, CASTER_PORT)) return false;

  String authHeader = String("SOURCE ") + SOURCE_PASSWORD + " /" + MOUNTPOINT + "\r\n" +
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
  connectWiFi();
  if (ENABLE_CLOUD_CASTER && WiFi.status() == WL_CONNECTED) {
    connectCloudCaster();
  }
}

void loop() {
  static uint32_t lastReportTime = 0;
  static uint32_t lastReconnectAttempt = 0;
  static uint32_t lastF9PRetry = 0;

  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    webServer.handleClient();
    handleLocalNtripClients();
  }

  if (f9pConnected) {
    processUARTStream();
  } else if (millis() - lastF9PRetry > 10000) {
    lastF9PRetry = millis();
    configureF9PBase();
  }

  if (millis() - lastReconnectAttempt > 5000) {
    lastReconnectAttempt = millis();
    if (WiFi.status() != WL_CONNECTED) connectWiFi();
    if (ENABLE_CLOUD_CASTER && WiFi.status() == WL_CONNECTED && !cloudCasterClient.connected()) {
      connectCloudCaster();
    }
  }

  if (millis() - lastReportTime > 5000) {
    lastReportTime = millis();
    last5SecBytes = rtcmBytesGenerated;
    rtcmBytesGenerated = 0;

    Serial.printf("[v%s] Спутников: %2u | Режим: %s | Роверов: %u | RTCM3: %lu Б/5с | http://%s\n",
                  FW_VERSION,
                  currentSIV,
                  useFixedBase ? "FIXED" : (svinValid ? "SVIN-OK" : "SVIN-RUN"),
                  getActiveRoverCount(),
                  (unsigned long)last5SecBytes,
                  WiFi.localIP().toString().c_str());
  }
}
