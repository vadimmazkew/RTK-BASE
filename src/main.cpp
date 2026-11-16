#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <WebServer.h>
#include <HTTPUpdateServer.h>
#include <Preferences.h>
#include <FS.h>
#include <LittleFS.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include "secrets.h"

#define FW_VERSION "1.8.0"
#define RXD2 16  // К пину TX1 платы ZED-F9P (URTK1.2)
#define TXD2 17  // К пину RX1 платы ZED-F9P (URTK1.2)
#define MAX_LOCAL_CLIENTS 3
#define MAX_DISPLAY_SATS  48
#define STATIC_LOG_FILE   "/static_base.rtcm3"
#define MAX_LOG_SIZE_BYTES (1800 * 1024) // Лимит 1.8 МБ для безопасного хранения во Flash

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
bool balconyFilter  = true;           // Срез переотражений балкона (15 град / 22 dBHz)
double fixedLatDeg  = 52.1115330;     // Пинск
double fixedLonDeg  = 26.1004610;
double fixedHeightM = 145.000;
double antennaHOffset = 0.045;        // ARP -> APC (+45 мм для ANT-B10)
uint32_t svinMinDur = 300;
float svinAccLimitM = 2.00;

// === Состояние сети и телеметрии ===
bool isApMode = false;
volatile uint32_t rtcmBytesGenerated = 0;
uint32_t last5SecBytes = 0;
uint8_t currentSIV = 0;
uint8_t currentFixType = 0;
float currentPDOP = 1.1f;
float currentHAccMM = 10.0f;
float currentVAccMM = 15.0f;
double liveLatDeg = 52.1115330;
double liveLonDeg = 26.1004610;
double liveHeightM = 145.000;
bool svinActive = false;
bool svinValid = true;
uint32_t svinObsTime = 300;
float svinMeanAcc = 0.005f;
bool f9pConnected = false;

// === 1. Математическое ядро «Умной Статики» (Взвешенный фильтр Виссельфорда + 3-сигма) ===
bool smartStaticActive = true;
uint32_t smartEpochsAccepted = 0;
uint32_t smartEpochsRejected = 0;
double smartWeightSum = 0.0;
double smartMeanLat   = 52.1115330;
double smartMeanLon   = 26.1004610;
double smartMeanHgt   = 145.000;
double smartM2Lat     = 0.0;
double smartM2Lon     = 0.0;
double smartRmsMM     = 0.0;          // Оценка сходимости центра в миллиметрах

// === 2. PPK Логгер статики во внутреннюю флеш-память LittleFS ===
bool fsMounted = false;
bool isLoggingStatic = false;
File staticLogFile;
uint32_t loggedBytesTotal = 0;

// === 3. Обратный канал от полевого ровера EFT M4 (NMEA $GPGGA) ===
String roverStatusText = "Нет данных GGA";
double roverLatDeg = 0.0;
double roverLonDeg = 0.0;
float roverBaselineKM = 0.0f;
uint8_t roverSats = 0;
uint32_t lastRoverGgaTime = 0;

// === Счетчики пакетов RTCM3 ===
uint32_t cnt1005 = 0, cnt1077 = 0, cnt1087 = 0, cnt1097 = 0, cnt1127 = 0, cnt1230 = 0;

// === Монитор ВЧ-помех и антенны ===
uint16_t noisePer = 0;
uint8_t jammerState = 0;
String antennaStatus = "OK";

// === Структура спутников ===
struct SatInfo {
  char sysChar;
  uint8_t svId;
  uint8_t cno;
  int8_t elev;
  int16_t azim;
  bool used;
};

SatInfo satList[MAX_DISPLAY_SATS];
uint8_t satsInView = 0;
uint8_t satsInUse  = 0;
float avgSnr       = 0.0f;

// Сброс накопителя «Умной Статики»
void resetSmartStatic() {
  smartEpochsAccepted = 0;
  smartEpochsRejected = 0;
  smartWeightSum = 0.0;
  smartM2Lat = 0.0;
  smartM2Lon = 0.0;
  smartRmsMM = 0.0;
}

// Добавление новой эпохи в фильтр «Умной Статики» с отсевом выбросов 3-сигма
void feedSmartStaticEpoch(double lat, double lon, double hgt, float pdop, float hAccMM) {
  if (!smartStaticActive) return;

  // Жесткий геодезический фильтр качества эпохи
  if (pdop > 1.6f || avgSnr < 28.0f || currentSIV < 16 || hAccMM > 1500.0f) {
    smartEpochsRejected++;
    return;
  }

  // Перевод разницы в метры для проверки 3-сигма (после первых 30 эпох прогрева)
  if (smartEpochsAccepted > 30) {
    double dLatM = (lat - smartMeanLat) * 111320.0;
    double dLonM = (lon - smartMeanLon) * 111320.0 * cos(smartMeanLat * DEG_TO_RAD);
    double distMM = sqrt(dLatM * dLatM + dLonM * dLonM) * 1000.0;

    // Отбрасываем аномальные скачки от переотражений балкона (> 3 сигма или > 800 мм)
    double thresholdMM = max(800.0, smartRmsMM * 3.5 * sqrt((double)smartEpochsAccepted));
    if (distMM > thresholdMM) {
      smartEpochsRejected++;
      return;
    }
  }

  // Вес эпохи обратно пропорционален квадрату PDOP и оценке погрешности
  double weight = 1.0 / (max(0.5f, pdop * pdop) * max(5.0f, hAccMM));
  smartEpochsAccepted++;

  if (smartEpochsAccepted == 1) {
    smartWeightSum = weight;
    smartMeanLat = lat;
    smartMeanLon = lon;
    smartMeanHgt = hgt;
    smartRmsMM = hAccMM;
  } else {
    double newWeightSum = smartWeightSum + weight;
    double deltaLat = lat - smartMeanLat;
    double deltaLon = lon - smartMeanLon;
    double deltaHgt = hgt - smartMeanHgt;

    smartMeanLat += (weight / newWeightSum) * deltaLat;
    smartMeanLon += (weight / newWeightSum) * deltaLon;
    smartMeanHgt += (weight / newWeightSum) * deltaHgt;

    // Накопление дисперсии в метрах
    double dLatM = deltaLat * 111320.0;
    double dLonM = deltaLon * 111320.0 * cos(smartMeanLat * DEG_TO_RAD);
    smartM2Lat += weight * dLatM * dLatM;
    smartM2Lon += weight * dLonM * dLonM;
    smartWeightSum = newWeightSum;

    // СКО сходимости математического ожидания центра (в мм)
    double varM2 = (smartM2Lat + smartM2Lon) / smartWeightSum;
    smartRmsMM = (sqrt(varM2) * 1000.0) / sqrt((double)smartEpochsAccepted);
  }
}

// Разбор обратной строки NMEA $GPGGA / $GNGGA от полевого ровера EFT M4
double parseNmeaCoord(const String &val, const String &hem) {
  if (val.length() < 4) return 0.0;
  int dotIdx = val.indexOf('.');
  if (dotIdx < 2) return 0.0;
  double deg = val.substring(0, dotIdx - 2).toDouble();
  double mins = val.substring(dotIdx - 2).toDouble();
  double res = deg + (mins / 60.0);
  if (hem == "S" || hem == "W") res = -res;
  return res;
}

void parseRoverNMEA(const String &line) {
  if (!line.startsWith("$GPGGA") && !line.startsWith("$GNGGA")) return;

  // Разбиваем строку GGA по запятым
  int idx = 0;
  String parts[15];
  int start = 0;
  for (int i = 0; i < (int)line.length() && idx < 14; i++) {
    if (line[i] == ',' || line[i] == '*') {
      parts[idx++] = line.substring(start, i);
      start = i + 1;
    }
  }
  if (idx < 8) return;

  double rLat = parseNmeaCoord(parts[2], parts[3]);
  double rLon = parseNmeaCoord(parts[4], parts[5]);
  int fixQual = parts[6].toInt();
  roverSats   = parts[7].toInt();

  if (fixQual == 4) roverStatusText = "RTK FIXED (Фикс)";
  else if (fixQual == 5) roverStatusText = "RTK FLOAT (Плавающий)";
  else if (fixQual == 2) roverStatusText = "DGPS";
  else if (fixQual == 1) roverStatusText = "SINGLE (Автономно)";
  else roverStatusText = "Нет фикса";

  if (rLat != 0.0 && rLon != 0.0) {
    roverLatDeg = rLat;
    roverLonDeg = rLon;
    double dLatM = (roverLatDeg - fixedLatDeg) * 111320.0;
    double dLonM = (roverLonDeg - fixedLonDeg) * 111320.0 * cos(fixedLatDeg * DEG_TO_RAD);
    roverBaselineKM = sqrt(dLatM * dLatM + dLonM * dLonM) / 1000.0f;
    lastRoverGgaTime = millis();
  }
}

// 1. Загрузка и сохранение настроек
void loadSettings() {
  prefs.begin("rtk-v2", true);
  wifiSsid       = prefs.getString("w_ssid", WIFI_SSID);
  wifiPass       = prefs.getString("w_pass", WIFI_PASSWORD);
  cloudEnabled   = prefs.getBool("c_en", ENABLE_CLOUD_CASTER);
  casterHost     = prefs.getString("c_host", CASTER_HOST);
  casterPort     = prefs.getUShort("c_port", CASTER_PORT);
  casterMount    = prefs.getString("c_mnt", MOUNTPOINT);
  casterPass     = prefs.getString("c_pass", SOURCE_PASSWORD);

  useFixedBase   = prefs.getBool("fixed", false);
  balconyFilter  = prefs.getBool("b_filt", true);
  fixedLatDeg    = prefs.getDouble("lat", 52.1115330);
  fixedLonDeg    = prefs.getDouble("lon", 26.1004610);
  fixedHeightM   = prefs.getDouble("hgt", 145.000);
  antennaHOffset = prefs.getDouble("ant_off", 0.045);
  svinMinDur     = prefs.getUInt("dur", 300);
  svinAccLimitM  = prefs.getFloat("acc", 2.00f);
  prefs.end();

  if (wifiSsid == "YOUR_WIFI_SSID" || wifiSsid.length() == 0) {
    wifiSsid = WIFI_SSID;
    wifiPass = WIFI_PASSWORD;
  }
}

void saveSettings() {
  prefs.begin("rtk-v2", false);
  prefs.putString("w_ssid", wifiSsid);
  prefs.putString("w_pass", wifiPass);
  prefs.putBool("c_en", cloudEnabled);
  prefs.putString("c_host", casterHost);
  prefs.putUShort("c_port", casterPort);
  prefs.putString("c_mnt", casterMount);
  prefs.putString("c_pass", casterPass);

  prefs.putBool("fixed", useFixedBase);
  prefs.putBool("b_filt", balconyFilter);
  prefs.putDouble("lat", fixedLatDeg);
  prefs.putDouble("lon", fixedLonDeg);
  prefs.putDouble("hgt", fixedHeightM);
  prefs.putDouble("ant_off", antennaHOffset);
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

void inspectRtcmPacket(const uint8_t *buf, size_t len) {
  if (len < 6) return;
  uint16_t msgType = ((uint16_t)buf[3] << 4) | ((buf[4] >> 4) & 0x0F);
  if (msgType == 1005 || msgType == 1006) cnt1005++;
  else if (msgType == 1077) cnt1077++;
  else if (msgType == 1087) cnt1087++;
  else if (msgType == 1097) cnt1097++;
  else if (msgType == 1127) cnt1127++;
  else if (msgType == 1230) cnt1230++;
}

void broadcastRTCM(const uint8_t *data, size_t len) {
  if (len == 0) return;
  rtcmBytesGenerated += len;
  inspectRtcmPacket(data, len);

  // Запись в файл PPK статики во Flash-память (если включен логгер)
  if (isLoggingStatic && fsMounted && staticLogFile) {
    if (loggedBytesTotal + len < MAX_LOG_SIZE_BYTES) {
      staticLogFile.write(data, len);
      loggedBytesTotal += len;
    } else {
      staticLogFile.close();
      isLoggingStatic = false;
    }
  }

  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (localRoverClients[i] && localRoverClients[i].connected()) {
      localRoverClients[i].write(data, len);
    }
  }
  if (cloudEnabled && !isApMode && cloudCasterClient.connected()) {
    cloudCasterClient.write(data, len);
  }
}

// 2. Применение координат и фильтра многолучевости
bool applyBaseModeToF9P() {
  if (!f9pConnected) return false;

  myGNSS.newCfgValset(VAL_LAYER_RAM_BBR);

  uint8_t minElev = balconyFilter ? 15 : 10;
  uint8_t minCno  = balconyFilter ? 22 : 15;
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINELEV, minElev);
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINCNO, minCno);

  if (useFixedBase) {
    double totalHeight = fixedHeightM + antennaHOffset;

    int64_t lat_e9   = llround(fixedLatDeg * 1e9);
    int32_t lat_main = (int32_t)(lat_e9 / 100);
    int8_t  lat_hp   = (int8_t)(lat_e9 % 100);

    int64_t lon_e9   = llround(fixedLonDeg * 1e9);
    int32_t lon_main = (int32_t)(lon_e9 / 100);
    int8_t  lon_hp   = (int8_t)(lon_e9 % 100);

    int64_t hgt_mm_e1 = llround(totalHeight * 10000.0);
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

// 3. Колбэки ZED-F9P
void onPVTData(UBX_NAV_PVT_data_t *pvt) {
  if (pvt == nullptr) return;
  currentSIV = pvt->numSV;
  currentFixType = pvt->fixType;
  currentPDOP = pvt->pDOP / 100.0f;
  currentHAccMM = pvt->hAcc;
  currentVAccMM = pvt->vAcc;
  liveLatDeg = pvt->lat / 1e7;
  liveLonDeg = pvt->lon / 1e7;
  liveHeightM = pvt->height / 1000.0;

  if (currentFixType >= 3) {
    feedSmartStaticEpoch(liveLatDeg, liveLonDeg, liveHeightM, currentPDOP, currentHAccMM);
  }
}

void onSVINData(UBX_NAV_SVIN_data_t *svin) {
  if (svin == nullptr) return;
  svinActive = (svin->active == 1);
  svinValid = (svin->valid == 1);
  svinObsTime = svin->dur;
  svinMeanAcc = svin->meanAcc / 10000.0f;
}

void onSATData(UBX_NAV_SAT_data_t *sat) {
  if (sat == nullptr) return;

  uint8_t total = sat->header.numSvs;
  uint8_t count = 0;
  uint8_t usedCount = 0;
  uint32_t snrSum = 0;
  uint8_t snrCount = 0;

  for (uint8_t i = 0; i < total && count < MAX_DISPLAY_SATS; i++) {
    uint8_t gnssId = sat->blocks[i].gnssId;
    uint8_t cno    = sat->blocks[i].cno;
    bool isUsed    = (sat->blocks[i].flags.bits.svUsed == 1);

    if (cno == 0) continue;

    char sChar = 'G';
    if (gnssId == 0) sChar = 'G';
    else if (gnssId == 2) sChar = 'E';
    else if (gnssId == 3) sChar = 'B';
    else if (gnssId == 6) sChar = 'R';
    else sChar = 'S';

    satList[count].sysChar = sChar;
    satList[count].svId    = sat->blocks[i].svId;
    satList[count].cno     = cno;
    satList[count].elev    = sat->blocks[i].elev;
    satList[count].azim    = sat->blocks[i].azim;
    satList[count].used    = isUsed;
    count++;

    if (isUsed) usedCount++;
    if (cno > 0) {
      snrSum += cno;
      snrCount++;
    }
  }

  for (uint8_t i = 0; i < count; i++) {
    for (uint8_t j = i + 1; j < count; j++) {
      if (satList[j].cno > satList[i].cno) {
        SatInfo tmp = satList[i];
        satList[i] = satList[j];
        satList[j] = tmp;
      }
    }
  }

  satsInView = total;
  satsInUse  = usedCount;
  avgSnr     = (snrCount > 0) ? ((float)snrSum / snrCount) : 0.0f;
}

// Демо-симулятор (пока ZED-F9P в пути)
void runDemoSimulator() {
  static uint32_t lastSimTick = 0;
  if (millis() - lastSimTick < 1000) return;
  lastSimTick = millis();

  f9pConnected = false;
  currentFixType = 5;
  currentSIV = 36;
  satsInView = 42;
  satsInUse = 36;
  currentPDOP = 1.05f;
  currentHAccMM = 9.0f;

  cnt1005++; cnt1077++; cnt1087++; cnt1097++; cnt1127++;
  if ((cnt1077 % 5) == 0) cnt1230++;

  uint32_t snrSum = 0;
  uint8_t minCno = balconyFilter ? 22 : 15;
  for (int i = 0; i < 28; i++) {
    char sys[4] = {'G', 'R', 'E', 'B'};
    satList[i].sysChar = sys[i % 4];
    satList[i].svId = (i % 12) + 1;
    satList[i].cno = 24 + (millis() / 2000 + i * 5) % 24;
    satList[i].elev = 15 + (i * 11) % 72;
    satList[i].azim = (i * 13 + (millis() / 10000)) % 360;
    satList[i].used = (satList[i].cno >= minCno);
    snrSum += satList[i].cno;
  }
  avgSnr = snrSum / 28.0f;

  // Имитация работы фильтра Умной Статики в демо-режиме
  double jitterLat = ((int)(millis() % 100) - 50) * 1e-8;
  double jitterLon = ((int)((millis() / 7) % 100) - 50) * 1e-8;
  feedSmartStaticEpoch(52.1115330 + jitterLat, 26.1004610 + jitterLon, 145.000, currentPDOP, currentHAccMM);
}

void parseRawUBXByte(uint8_t b) {
  static uint8_t ubxState = 0;
  static uint8_t msgClass = 0, msgId = 0;
  static uint16_t msgLen = 0, payIdx = 0;
  static uint8_t rfPayload[64];

  switch (ubxState) {
    case 0: if (b == 0xB5) ubxState = 1; break;
    case 1: ubxState = (b == 0x62) ? 2 : 0; break;
    case 2: msgClass = b; ubxState = 3; break;
    case 3: msgId = b; ubxState = 4; break;
    case 4: msgLen = b; ubxState = 5; break;
    case 5:
      msgLen |= ((uint16_t)b << 8);
      payIdx = 0;
      if (msgClass == 0x0A && msgId == 0x38 && msgLen >= 28 && msgLen <= sizeof(rfPayload)) {
        ubxState = 6;
      } else {
        ubxState = 0;
      }
      break;
    case 6:
      rfPayload[payIdx++] = b;
      if (payIdx >= msgLen) {
        uint8_t nBlocks = rfPayload[1];
        if (nBlocks > 0) {
          uint8_t flags   = rfPayload[4 + 1];
          uint8_t antStat = rfPayload[4 + 2];
          noisePer        = (uint16_t)rfPayload[4 + 16] | ((uint16_t)rfPayload[4 + 17] << 8);
          jammerState     = flags & 0x03;

          if (antStat == 2) antennaStatus = "OK";
          else if (antStat == 3) antennaStatus = "SHORT (КЗ)";
          else if (antStat == 4) antennaStatus = "OPEN (Обрыв)";
          else antennaStatus = "OK";
        }
        ubxState = 0;
      }
      break;
  }
}

void processUARTStream() {
  static uint8_t rtcmBuf[1030];
  static uint16_t rtcmIdx = 0;
  static uint16_t rtcmExpectedLen = 0;
  static bool inRtcmFrame = false;

  while (RTKSerial.available() > 0) {
    uint8_t b = RTKSerial.read();
    f9pConnected = true;

    if (!inRtcmFrame) {
      if (b == 0xD3) {
        inRtcmFrame = true;
        rtcmIdx = 0;
        rtcmBuf[rtcmIdx++] = b;
      } else {
        parseRawUBXByte(b);
        myGNSS.pushRawData(&b, 1);
      }
    } else {
      rtcmBuf[rtcmIdx++] = b;
      if (rtcmIdx == 3) {
        if ((rtcmBuf[1] & 0xFC) != 0x00) {
          for (int k = 0; k < 3; k++) parseRawUBXByte(rtcmBuf[k]);
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

// 4. Инициализация сети
void initNetwork() {
  WiFi.persistent(false);
  WiFi.disconnect(true);
  delay(100);

  WiFi.mode(WIFI_AP_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);

  wifi_country_t country = {
    .cc = "BY",
    .schan = 1,
    .nchan = 13,
    .max_tx_power = 20,
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&country);

  #if USE_STATIC_IP
  IPAddress staticIP(STATIC_IP_ADDR);
  IPAddress gateway(GATEWAY_IP_ADDR);
  IPAddress subnet(SUBNET_MASK);
  IPAddress dns(DNS_IP_ADDR);
  WiFi.config(staticIP, gateway, subnet, dns);
  #endif

  const char* netsToTry[] = { wifiSsid.c_str(), "EFIR-TV WIFI 751", "EFIR TV WIFI 751_24" };

  for (int i = 0; i < 3; i++) {
    if (strlen(netsToTry[i]) == 0) continue;
    WiFi.begin(netsToTry[i], wifiPass.c_str());

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 14) {
      delay(500);
      attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      wifiSsid = netsToTry[i];
      break;
    }
  }

  int activeChannel = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 1;

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apNet(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, apNet);
  WiFi.softAP(AP_SSID, AP_PASSWORD, activeChannel, 0, 4);

  isApMode = (WiFi.status() != WL_CONNECTED);

  MDNS.begin(OTA_HOSTNAME);
  localNtripServer.begin();
  localNtripServer.setNoDelay(true);
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);
  ArduinoOTA.begin();
}

// 5. Веб-интерфейс v1.8.0 (Умная Статика + PPK Логгер + Контроль Ровера)
void setupWebMonitor() {
  httpUpdater.setup(&webServer, "/update", "admin", OTA_PASSWORD);

  // Скачивание записанного файла статики (.rtcm3) для постобработки PPK / RINEX
  webServer.on("/download_static", HTTP_GET, []() {
    if (!fsMounted || !LittleFS.exists(STATIC_LOG_FILE)) {
      webServer.send(404, "text/plain", "Log file not found");
      return;
    }
    File f = LittleFS.open(STATIC_LOG_FILE, "r");
    webServer.sendHeader("Content-Disposition", "attachment; filename=pinsk_base_static.rtcm3");
    webServer.streamFile(f, "application/octet-stream");
    f.close();
  });

  webServer.on("/toggle_log", HTTP_POST, []() {
    if (fsMounted) {
      if (!isLoggingStatic) {
        staticLogFile = LittleFS.open(STATIC_LOG_FILE, "a");
        if (staticLogFile) {
          loggedBytesTotal = staticLogFile.size();
          isLoggingStatic = true;
        }
      } else {
        if (staticLogFile) staticLogFile.close();
        isLoggingStatic = false;
      }
    }
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/clear_log", HTTP_POST, []() {
    if (fsMounted) {
      if (isLoggingStatic && staticLogFile) staticLogFile.close();
      isLoggingStatic = false;
      LittleFS.remove(STATIC_LOG_FILE);
      loggedBytesTotal = 0;
    }
    resetSmartStatic();
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/apply_smart", HTTP_POST, []() {
    if (smartEpochsAccepted > 0) {
      fixedLatDeg = smartMeanLat;
      fixedLonDeg = smartMeanLon;
      fixedHeightM = smartMeanHgt;
      antennaHOffset = 0.0;
      useFixedBase = true;
      saveSettings();
      applyBaseModeToF9P();
    }
    webServer.sendHeader("Location", "/");
    webServer.send(303);
  });

  webServer.on("/api/status", HTTP_GET, []() {
    String fixText = !f9pConnected ? "DEMO (Симулятор)" :
                     (currentFixType == 5) ? "TIME (БАЗА АКТИВНА)" :
                     (currentFixType == 3) ? "3D Fix" : "Поиск неба";
    String netInfo = isApMode ? ("AP: " + String(AP_SSID) + " (192.168.4.1)") :
                                (wifiSsid + " (" + WiFi.localIP().toString() + ", " + String(WiFi.RSSI()) + " dBm)");

    float cpuTemp = temperatureRead();
    uint32_t uptimeSec = millis() / 1000;
    char uptimeStr[32];
    sprintf(uptimeStr, "%luч %02luм", uptimeSec / 3600, (uptimeSec % 3600) / 60);

    String rtcmStat = "1005:" + String(cnt1005) + " | G:" + String(cnt1077) +
                      " R:" + String(cnt1087) + " E:" + String(cnt1097) + " B:" + String(cnt1127);

    String smartCoords = String(smartMeanLat, 8) + ", " + String(smartMeanLon, 8) + " (" + String(smartMeanHgt, 3) + "м)";
    String smartStat   = "±" + String(smartRmsMM, 1) + " мм (Принято: " + String(smartEpochsAccepted) + " / Отсев: " + String(smartEpochsRejected) + ")";

    bool roverOnline = (lastRoverGgaTime > 0 && (millis() - lastRoverGgaTime < 15000));
    String rvrInfo = roverOnline ? (roverStatusText + " | Базовая линия: " + String(roverBaselineKM, 2) + " км") :
                                   "Ожидание строки $GPGGA от контроллера EFT H3...";

    String json = "{";
    json += "\"f9p\":" + String(f9pConnected ? "true" : "false") + ",";
    json += "\"fixType\":" + String(currentFixType) + ",";
    json += "\"fixText\":\"" + fixText + "\",";
    json += "\"siv\":" + String(currentSIV) + ",";
    json += "\"inView\":" + String(satsInView) + ",";
    json += "\"inUse\":" + String(satsInUse > 0 ? satsInUse : currentSIV) + ",";
    json += "\"pdop\":\"" + String(currentPDOP, 1) + "\",";
    json += "\"avgSnr\":" + String(avgSnr, 1) + ",";
    json += "\"antStat\":\"" + antennaStatus + "\",";
    json += "\"temp\":\"" + String(cpuTemp, 1) + "°C\",";
    json += "\"uptime\":\"" + String(uptimeStr) + "\",";
    json += "\"rtcmStat\":\"" + rtcmStat + "\",";
    json += "\"smartCoords\":\"" + smartCoords + "\",";
    json += "\"smartStat\":\"" + smartStat + "\",";
    json += "\"logActive\":" + String(isLoggingStatic ? "true" : "false") + ",";
    json += "\"logKB\":\"" + String(loggedBytesTotal / 1024.0f, 1) + " КБ\",";
    json += "\"rvrInfo\":\"" + rvrInfo + "\",";
    json += "\"rovers\":" + String(getActiveRoverCount()) + ",";
    json += "\"bytes\":" + String(last5SecBytes) + ",";
    json += "\"net\":\"" + netInfo + "\",";

    json += "\"sats\":[";
    uint8_t outCount = 0;
    for (uint8_t i = 0; i < MAX_DISPLAY_SATS && i < satsInView; i++) {
      if (satList[i].cno == 0) continue;
      if (outCount > 0) json += ",";
      json += "{\"id\":\"" + String(satList[i].sysChar) + String(satList[i].svId) + "\",";
      json += "\"cno\":" + String(satList[i].cno) + ",";
      json += "\"elev\":" + String(satList[i].elev) + ",";
      json += "\"azim\":" + String(satList[i].azim) + ",";
      json += "\"used\":" + String(satList[i].used ? 1 : 0) + "}";
      outCount++;
      if (outCount >= 28) break;
    }
    json += "]}";
    webServer.send(200, "application/json", json);
  });

  webServer.on("/", HTTP_GET, []() {
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'>"
      "<meta name='viewport' content='width=device-width,initial-scale=1'>"
      "<title>RTK База v" FW_VERSION "</title>"
      "<style>"
      "body{font-family:-apple-system,sans-serif;background:#0b0e14;color:#eee;margin:0;padding:10px;}"
      ".card{background:#151922;padding:14px;border-radius:12px;max-width:500px;margin:0 auto 12px;border:1px solid #262c3a;box-shadow:0 4px 12px rgba(0,0,0,0.5);}"
      "h2,h3{margin-top:0;color:#64b5f6;font-size:17px;}"
      ".top-grid{display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px;}"
      ".stat-box{background:#090b10;border:1px solid #2c3344;border-radius:8px;padding:10px;}"
      ".stat-title{font-size:11px;color:#9aa0a6;}"
      ".stat-val{font-size:17px;font-weight:bold;margin-top:4px;}"
      ".sky-panel{background:#050826;border:1px solid #2b3358;border-radius:10px;padding:12px;margin-bottom:10px;}"
      ".sky-counts{display:flex;justify-content:space-between;margin-bottom:6px;}"
      ".bars-wrap{display:flex;align-items:flex-end;gap:4px;height:125px;overflow-x:auto;padding-top:16px;border-bottom:1px solid #394264;}"
      ".bar-col{display:flex;flex-direction:column;align-items:center;min-width:24px;}"
      ".bar-val{font-size:10px;color:#fff;margin-bottom:2px;}"
      ".bar-fill{width:18px;border-radius:3px 3px 0 0;transition:height 0.3s;}"
      ".bar-dot{width:6px;height:6px;border-radius:50%;margin:4px 0 3px;}"
      ".bar-id{font-size:10px;background:#e0e0e0;color:#000;font-weight:bold;border-radius:3px;padding:1px 3px;}"
      ".snr-bar{height:12px;border-radius:6px;background:linear-gradient(90deg,#d32f2f 0% 15%,#f57c00 15% 30%,#fbc02d 30% 50%,#64dd17 50% 100%);position:relative;margin-top:16px;}"
      ".snr-pin{position:absolute;top:-18px;transform:translateX(-50%);background:#fbc02d;color:#000;font-size:11px;font-weight:bold;padding:1px 6px;border-radius:8px;}"
      ".row{display:flex;justify-content:space-between;margin:7px 0;border-bottom:1px solid #222733;padding-bottom:5px;font-size:13px;}"
      "label{display:block;margin-top:10px;font-size:12px;color:#aaa;}"
      "input,select{width:100%;padding:9px;margin-top:4px;background:#202634;border:1px solid #384152;color:#fff;border-radius:6px;box-sizing:border-box;font-size:14px;}"
      ".btn{display:block;width:100%;padding:10px;margin-top:8px;border:none;border-radius:6px;font-size:13px;font-weight:bold;cursor:pointer;text-align:center;text-decoration:none;box-sizing:border-box;}"
      ".btn-save{background:#2e7d32;color:#fff;}"
      ".btn-copy{background:#1565c0;color:#fff;}"
      ".btn-warn{background:#c62828;color:#fff;}"
      ".btn-preset{background:#37474f;color:#fff;width:48%;display:inline-block;margin-top:6px;}"
      ".btn-refresh{background:#303644;color:#fff;}"
      "</style>"
      "<script>"
      "function setPreset(lat,lon,hgt){"
      "document.getElementById('lat').value=lat;"
      "document.getElementById('lon').value=lon;"
      "document.getElementById('hgt').value=hgt;"
      "}"
      "function barColor(c){"
      "if(c>=30)return '#76ff03';"
      "if(c>=20)return '#ffea00';"
      "if(c>=10)return '#ff6d00';"
      "return '#d50000';"
      "}"
      "function drawSkyplot(sats){"
      "let canvas = document.getElementById('skyplot');"
      "if(!canvas)return;"
      "let ctx = canvas.getContext('2d');"
      "let cx = canvas.width/2, cy = canvas.height/2, r = cx - 16;"
      "ctx.clearRect(0,0,canvas.width,canvas.height);"
      "ctx.strokeStyle = '#2b3358'; ctx.lineWidth = 1;"
      "ctx.beginPath(); ctx.arc(cx,cy,r,0,2*Math.PI); ctx.stroke();"
      "ctx.beginPath(); ctx.arc(cx,cy,r*0.66,0,2*Math.PI); ctx.stroke();"
      "ctx.beginPath(); ctx.arc(cx,cy,r*0.33,0,2*Math.PI); ctx.stroke();"
      "ctx.beginPath(); ctx.moveTo(cx,16); ctx.lineTo(cx,canvas.height-16); ctx.stroke();"
      "ctx.beginPath(); ctx.moveTo(16,cy); ctx.lineTo(canvas.width-16,cy); ctx.stroke();"
      "ctx.fillStyle = '#64b5f6'; ctx.font = 'bold 11px sans-serif';"
      "ctx.fillText('N', cx-4, 12); ctx.fillText('S', cx-4, canvas.height-4);"
      "ctx.fillText('W', 3, cy+4); ctx.fillText('E', canvas.width-11, cy+4);"
      "if(!sats)return;"
      "sats.forEach(s=>{"
      "let dist = (1 - (s.elev/90)) * r;"
      "let rad = (s.azim - 90) * (Math.PI/180);"
      "let x = cx + dist * Math.cos(rad);"
      "let y = cy + dist * Math.sin(rad);"
      "ctx.fillStyle = barColor(s.cno);"
      "ctx.beginPath(); ctx.arc(x,y,4,0,2*Math.PI); ctx.fill();"
      "});"
      "}"
      "function updateUI(){"
      "fetch('/api/status').then(r=>r.json()).then(d=>{"
      "let fixEl = document.getElementById('s_fix');"
      "fixEl.innerText = d.fixText;"
      "fixEl.style.color = (d.f9p ? '#00e676' : '#ffea00');"
      "document.getElementById('s_pdop').innerText = d.pdop;"
      "document.getElementById('s_view').innerText = d.inView;"
      "document.getElementById('s_use').innerText = d.inUse;"
      "document.getElementById('s_ant').innerText = d.antStat;"
      "document.getElementById('s_temp').innerText = d.temp + ' / ' + d.uptime;"
      "document.getElementById('s_rtcm').innerText = d.rtcmStat;"
      "document.getElementById('s_smart_c').innerText = d.smartCoords;"
      "document.getElementById('s_smart_s').innerText = d.smartStat;"
      "document.getElementById('s_log_kb').innerText = d.logKB + (d.logActive ? ' (ЗАПИСЬ...)' : ' (Стоп)');"
      "document.getElementById('s_rvr').innerText = d.rvrInfo;"
      "document.getElementById('s_rovers').innerText = d.rovers+' шт.';"
      "document.getElementById('s_bytes').innerText = d.bytes+' байт';"
      "document.getElementById('s_net').innerText = d.net;"
      "let avg = d.avgSnr || 0;"
      "document.getElementById('snr_val').innerText = avg.toFixed(1);"
      "document.getElementById('snr_pin').style.left = Math.min(Math.max(avg, 5), 95) + '%';"
      "let bw = document.getElementById('bars');"
      "if(d.sats && d.sats.length > 0){"
      "let h = '';"
      "d.sats.forEach(s=>{"
      "let px = Math.min(Math.max(s.cno * 2.2, 6), 110);"
      "let col = barColor(s.cno);"
      "let dot = s.used ? '#00e676' : '#555';"
      "h += '<div class=\"bar-col\"><div class=\"bar-val\">'+s.cno+'</div>'"
      "+'<div class=\"bar-fill\" style=\"height:'+px+'px;background:'+col+';\"></div>'"
      "+'<div class=\"bar-dot\" style=\"background:'+dot+';\"></div>'"
      "+'<div class=\"bar-id\">'+s.id+'</div></div>';"
      "});"
      "bw.innerHTML = h;"
      "drawSkyplot(d.sats);"
      "}"
      "}).catch(()=>{});"
      "}"
      "setInterval(updateUI, 2000);"
      "window.onload = updateUI;"
      "</script></head><body>"

      // Карточка 1: Спектр и радар
      "<div class='card'>"
      "<h2>🛰️ RTK База v" FW_VERSION " (" + String(LOCAL_MOUNTPOINT) + ")</h2>"
      "<div class='top-grid'>"
      "<div class='stat-box'><div class='stat-title'>Статус GNSS</div><div id='s_fix' class='stat-val' style='font-size:14px;'>Загрузка...</div></div>"
      "<div class='stat-box'><div class='stat-title'>PDOP (Геометрия)</div><div id='s_pdop' class='stat-val' style='color:#00e5ff;'>-</div></div>"
      "<div class='stat-box'><div class='stat-title'>Антенна (ANT-B10)</div><div id='s_ant' class='stat-val' style='font-size:15px;color:#76ff03;'>-</div></div>"
      "<div class='stat-box'><div class='stat-title'>CPU / Uptime</div><div id='s_temp' class='stat-val' style='font-size:14px;color:#ffea00;'>-</div></div>"
      "</div>"

      "<div class='sky-panel'>"
      "<div class='sky-counts'>"
      "<div><div class='stat-title'>В поле зрения: <b id='s_view' style='color:#fff;font-size:15px;'>0</b></div></div>"
      "<div style='text-align:right;'><div class='stat-title'>В решении RTK: <b id='s_use' style='color:#76ff03;font-size:15px;'>0</b></div></div>"
      "</div>"
      "<canvas id='skyplot' width='200' height='200' style='margin:4px auto;display:block;'></canvas>"
      "<div id='bars' class='bars-wrap'></div>"
      "<div style='margin-top:10px;font-size:11px;color:#9aa0a6;'>AVG SNR (Средний уровень сигнала, dBHz):</div>"
      "<div class='snr-bar'><div id='snr_pin' class='snr-pin' style='left:30%;'><span id='snr_val'>0.0</span></div></div>"
      "</div>"

      "<div class='row'><span>Инспектор RTCM3:</span><b id='s_rtcm' style='font-size:11px;color:#80cbc4;'>-</b></div>"
      "<div class='row'><span>Роверов (Порт " + String(LOCAL_NTRIP_PORT) + "):</span><b id='s_rovers'>0 шт.</b></div>"
      "<div class='row'><span>Контроль ровера (GGA):</span><b id='s_rvr' style='font-size:11px;color:#00e676;'>-</b></div>"
      "<div class='row'><span>Поток RTCM3 (5с):</span><b id='s_bytes'>0 байт</b></div>"
      "<div class='row'><span>Сеть:</span><b id='s_net' style='font-size:12px;'>" + WiFi.localIP().toString() + "</b></div>"
      "</div>"

      // Карточка 2: Умный прием статики и PPK логгер во Flash
      "<div class='card'>"
      "<h3>🧠 Умный прием статики (Фильтр 3σ + PPK Логгер)</h3>"
      "<div class='row'><span>Взвешенный центр:</span><b id='s_smart_c' style='font-size:12px;color:#00e5ff;'>-</b></div>"
      "<div class='row'><span>Сходимость центра:</span><b id='s_smart_s' style='font-size:12px;color:#76ff03;'>-</b></div>"
      "<div class='row'><span>Лог статики во Flash:</span><b id='s_log_kb' style='color:#ffea00;'>0 КБ</b></div>"
      "<form action='/apply_smart' method='POST'>"
      "<button type='submit' class='btn btn-copy'>🎯 Зафиксировать умный центр в FIXED</button>"
      "</form>"
      "<div style='display:flex;gap:6px;'>"
      "<form action='/toggle_log' method='POST' style='width:50%;'>"
      "<button type='submit' class='btn btn-save'>⏺ Старт / Стоп записи PPK</button>"
      "</form>"
      "<a href='/download_static' class='btn btn-refresh' style='width:50%;'>📥 Скачать .rtcm3</a>"
      "</div>"
      "<form action='/clear_log' method='POST'>"
      "<button type='submit' class='btn btn-warn'>🗑 Сбросить статистику и очистить лог</button>"
      "</form>"
      "</div>"

      // Карточка 3: Координаты и фильтр неба
      "<div class='card'>"
      "<h3>⚙️ Координаты, фильтр неба и фазовый центр</h3>"
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
      "<label>Подавление переотражений (Multipath Filter):</label>"
      "<select name='b_filt'>"
      "<option value='1'" + String(balconyFilter ? " selected" : "") + ">Балкон / Застройка (Срез 15°, порог 22 dBHz)</option>"
      "<option value='0'" + String(!balconyFilter ? " selected" : "") + ">Крыша / Поле (Срез 10°, порог 15 dBHz)</option>"
      "</select>"
      "<label>Широта WGS-84 (град):</label>"
      "<input id='lat' type='number' step='0.00000001' name='lat' value='" + String(fixedLatDeg, 8) + "' required>"
      "<label>Долгота WGS-84 (град):</label>"
      "<input id='lon' type='number' step='0.00000001' name='lon' value='" + String(fixedLonDeg, 8) + "' required>"
      "<label>Высота базы над эллипсоидом (м):</label>"
      "<input id='hgt' type='number' step='0.0001' name='hgt' value='" + String(fixedHeightM, 4) + "' required>"
      "<label>Поправка фазового центра ARP ➔ APC (м):</label>"
      "<input type='number' step='0.001' name='ant_off' value='" + String(antennaHOffset, 3) + "' required>"
      "<button type='submit' class='btn btn-save'>💾 Применить настройки в ZED-F9P</button>"
      "</form>"
      "</div>"

      // Карточка 4: Сеть
      "<div class='card'>"
      "<h3>🌐 Сеть Wi-Fi и Кастер</h3>"
      "<form action='/save_net' method='POST'>"
      "<label>Имя Wi-Fi сети (SSID):</label>"
      "<input type='text' name='w_ssid' value='" + wifiSsid + "'>"
      "<label>Пароль Wi-Fi:</label>"
      "<input type='password' name='w_pass' value='" + wifiPass + "'>"
      "<label>Дублировать поток во внешний NTRIP Caster:</label>"
      "<select name='c_en'>"
      "<option value='0'" + String(!cloudEnabled ? " selected" : "") + ">Выкл (Только прямой сервер 192.168.0.200:2101)</option>"
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
      "<a href='/update' class='btn btn-refresh' style='margin-top:14px;font-size:13px;'>📦 Веб-обновление прошивки (.bin)</a>"
      "</div></body></html>";

    webServer.send(200, "text/html", html);
  });

  webServer.on("/save_gnss", HTTP_POST, []() {
    if (webServer.hasArg("mode"))    useFixedBase   = (webServer.arg("mode").toInt() == 1);
    if (webServer.hasArg("b_filt"))  balconyFilter  = (webServer.arg("b_filt").toInt() == 1);
    if (webServer.hasArg("lat"))     fixedLatDeg    = webServer.arg("lat").toDouble();
    if (webServer.hasArg("lon"))     fixedLonDeg    = webServer.arg("lon").toDouble();
    if (webServer.hasArg("hgt"))     fixedHeightM   = webServer.arg("hgt").toDouble();
    if (webServer.hasArg("ant_off")) antennaHOffset = webServer.arg("ant_off").toDouble();

    saveSettings();
    applyBaseModeToF9P();
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
    webServer.send(200, "text/html", "<meta charset='utf-8'><h3>Настройки сохранены! Перезагрузка...</h3>");
    delay(800);
    ESP.restart();
  });

  webServer.begin();
}

// Обработка подключений и чтение обратной строки $GPGGA от полевого контроллера EFT H3
void handleLocalNtripClients() {
  // Читаем входящие строки NMEA GGA от уже подключенных роверов
  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (localRoverClients[i] && localRoverClients[i].connected() && localRoverClients[i].available()) {
      String line = localRoverClients[i].readStringUntil('\n');
      line.trim();
      if (line.startsWith("$G")) {
        parseRoverNMEA(line);
      }
    }
  }

  if (!localNtripServer.hasClient()) return;

  WiFiClient newClient = localNtripServer.accept();
  if (!newClient) return;
  newClient.setNoDelay(true);

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
      "STR;" + String(LOCAL_MOUNTPOINT) + ";Pinsk;RTCM 3.3;1005(1),1006(5),1033(10),1077(1),1087(1),1097(1),1127(1),1230(5);2;GPS+GLO+GAL+BDS;LOCAL;BLR;" +
      String(fixedLatDeg, 2) + ";" + String(fixedLonDeg, 2) + ";1;0;ESP32-F9P;none;N;N;9600;\r\nENDSOURCETABLE\r\n";
    newClient.print(sourceTable);
    delay(50);
    newClient.stop();
    return;
  }

  for (int i = 0; i < MAX_LOCAL_CLIENTS; i++) {
    if (!localRoverClients[i] || !localRoverClients[i].connected()) {
      if (request.length() > 0) newClient.print("ICY 200 OK\r\n\r\n");
      localRoverClients[i] = newClient;
      localRoverClients[i].setNoDelay(true);
      return;
    }
  }
  newClient.stop();
}

bool configureF9PBase() {
  RTKSerial.setRxBufferSize(8192);
  RTKSerial.begin(460800, SERIAL_8N1, RXD2, TXD2);

  if (!myGNSS.begin(RTKSerial)) {
    RTKSerial.begin(115200, SERIAL_8N1, RXD2, TXD2);
    if (!myGNSS.begin(RTKSerial)) {
      RTKSerial.begin(38400, SERIAL_8N1, RXD2, TXD2);
      if (!myGNSS.begin(RTKSerial)) return false;
    }
    myGNSS.setSerialRate(460800, COM_PORT_UART1);
    delay(250);
    RTKSerial.begin(460800, SERIAL_8N1, RXD2, TXD2);
    if (!myGNSS.begin(RTKSerial)) return false;
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
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINELEV, (uint8_t)(balconyFilter ? 15 : 10));
  myGNSS.addCfgValset(UBLOX_CFG_NAVSPG_INFIL_MINCNO, (uint8_t)(balconyFilter ? 22 : 15));
  myGNSS.addCfgValset(UBLOX_CFG_RATE_MEAS, 1000);

  // Компенсация задержки сигнала в 5-метровом коаксиальном кабеле RG-58 (25 наносекунд = 0x30050001)
  myGNSS.addCfgValset((uint32_t)0x30050001, (uint16_t)25);

  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_NMEA, 0);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_RTCM3X, 1);
  myGNSS.addCfgValset(UBLOX_CFG_UART1OUTPROT_UBX, 1);

  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1005_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1006_UART1, 5);
  myGNSS.addCfgValset((uint32_t)0x20910319, (uint8_t)10); // RTCM 1033

  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1077_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1087_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1097_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1127_UART1, 1);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_RTCM_3X_TYPE1230_UART1, 5);

  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_PVT_UART1, 2);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_SVIN_UART1, 2);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_NAV_SAT_UART1, 2);
  myGNSS.addCfgValset(UBLOX_CFG_MSGOUT_UBX_MON_RF_UART1, 5);

  if (myGNSS.sendCfgValset()) {
    myGNSS.setAutoPVTcallbackPtr(&onPVTData);
    myGNSS.setAutoNAVSVINcallbackPtr(&onSVINData);
    myGNSS.setAutoNAVSATcallbackPtr(&onSATData);
    return applyBaseModeToF9P();
  }
  return false;
}

bool connectCloudCaster() {
  if (!cloudEnabled || isApMode || cloudCasterClient.connected()) return true;
  if (!cloudCasterClient.connect(casterHost.c_str(), casterPort)) return false;
  cloudCasterClient.setNoDelay(true);

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

// Выделенная задача на Ядре 0 (Core 0) для мгновенной прокачки UART -> RTCM3 без задержек от веб-интерфейса
void gnssStreamTask(void *pvParameters) {
  uint32_t lastF9PCheck = 0;
  for (;;) {
    if (f9pConnected) {
      processUARTStream();
    } else {
      runDemoSimulator();
      if (millis() - lastF9PCheck > 10000) {
        lastF9PCheck = millis();
        configureF9PBase();
      }
    }
    vTaskDelay(1); // Минимальная уступка планировщику (1 мс)
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  // Монтируем файловую систему LittleFS во встроенной флеш-памяти для записи статики PPK
  if (LittleFS.begin(true)) {
    fsMounted = true;
    if (LittleFS.exists(STATIC_LOG_FILE)) {
      File f = LittleFS.open(STATIC_LOG_FILE, "r");
      if (f) { loggedBytesTotal = f.size(); f.close(); }
    }
  }

  loadSettings();
  initNetwork();
  setupWebMonitor();
  configureF9PBase();

  // Закрепляем поток реального времени GNSS на Ядре 0 (а веб-сервер остается на Ядре 1)
  xTaskCreatePinnedToCore(gnssStreamTask, "GNSS_RTCM_Task", 8192, NULL, 2, NULL, 0);
}

void loop() {
  static uint32_t lastReportTime = 0;
  static uint32_t lastReconnectAttempt = 0;

  ArduinoOTA.handle();
  webServer.handleClient();
  handleLocalNtripClients();

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

    Serial.printf("[v%s] Режим: %s | Умная статика: ±%.1f мм (%lu эп) | RTCM3: %lu Б/5с | IP: %s\n",
                  FW_VERSION,
                  f9pConnected ? "ZED-F9P" : "DEMO",
                  smartRmsMM,
                  (unsigned long)smartEpochsAccepted,
                  (unsigned long)last5SecBytes,
                  isApMode ? "192.168.4.1" : WiFi.localIP().toString().c_str());
  }
}
