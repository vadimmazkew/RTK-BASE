#pragma once

// 1. Настройки Wi-Fi
#define WIFI_SSID           "EFIR-TV WIFI 751"
#define WIFI_PASSWORD       "29362271"

// 2. Имя устройства в сети и пароль для прошивки по воздуху (OTA)
#define OTA_HOSTNAME        "rtk-base"
#define OTA_PASSWORD        "admin1234"

// 3. Встроенный прямой NTRIP-сервер на ESP32-S3 (Порт 2101)
#define LOCAL_NTRIP_PORT    2101
#define LOCAL_MOUNTPOINT    "PINSK_BASE"

// 4. Облачный кастер Emlid (false = отключен, работа только напрямую с ESP32)
#define ENABLE_CLOUD_CASTER false
#define CASTER_HOST         "caster.emlid.com"
#define CASTER_PORT         2101
#define MOUNTPOINT          "MP1234"
#define SOURCE_PASSWORD     "YOUR_CASTER_PASSWORD"
