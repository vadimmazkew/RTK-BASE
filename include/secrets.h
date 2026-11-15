#pragma once

// 1. Домашний/рабочий Wi-Fi (по умолчанию, можно поменять позже прямо в веб-панели)
#define WIFI_SSID           "EFIR-TV WIFI 751"
#define WIFI_PASSWORD       "29362271"

// 2. Резервная точка доступа на самой ESP32-S3 (поднимается, если нет связи с роутером)
#define AP_SSID             "wifi-iot"
#define AP_PASSWORD         "12345678"      // Минимум 8 символов!

// 3. Имя в локальной сети и пароль OTA / веб-обновления
#define OTA_HOSTNAME        "rtk-base"
#define OTA_PASSWORD        "admin1234"

// 4. Встроенный прямой NTRIP-сервер (Порт 2101)
#define LOCAL_NTRIP_PORT    2101
#define LOCAL_MOUNTPOINT    "PINSK_BASE"

// 5. Облачный кастер Emlid (по умолчанию выключен, включается в веб-панели)
#define ENABLE_CLOUD_CASTER false
#define CASTER_HOST         "caster.emlid.com"
#define CASTER_PORT         2101
#define MOUNTPOINT          "MP1234"
#define SOURCE_PASSWORD     "YOUR_CASTER_PASSWORD"
