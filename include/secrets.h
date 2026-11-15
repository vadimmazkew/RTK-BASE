#pragma once

// 1. Ваша сеть Wi-Fi
#define WIFI_SSID           "EFIR-TV WIFI 751"
#define WIFI_PASSWORD       "29362271"

// Постоянный статический IP в вашей подсети 192.168.0.x
#define USE_STATIC_IP       true
#define STATIC_IP_ADDR      192, 168, 0, 200
#define GATEWAY_IP_ADDR     192, 168, 0, 1
#define SUBNET_MASK         255, 255, 255, 0
#define DNS_IP_ADDR         8, 8, 8, 8

// 2. Резервная точка доступа (если роутер выключен)
#define AP_SSID             "wifi-iot"
#define AP_PASSWORD         "12345678"

// 3. Имя устройства и пароль для прошивки по воздуху (OTA)
#define OTA_HOSTNAME        "rtk-base"
#define OTA_PASSWORD        "admin1234"

// 4. Встроенный прямой NTRIP-сервер на ESP32-S3 (Порт 2101)
#define LOCAL_NTRIP_PORT    2101
#define LOCAL_MOUNTPOINT    "PINSK_BASE"

// 5. Облачный кастер Emlid
#define ENABLE_CLOUD_CASTER false
#define CASTER_HOST         "caster.emlid.com"
#define CASTER_PORT         2101
#define MOUNTPOINT          "MP1234"
#define SOURCE_PASSWORD     "YOUR_CASTER_PASSWORD"
