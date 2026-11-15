#pragma once

// 1. Домашний/рабочий Wi-Fi
#define WIFI_SSID           "YOUR_WIFI_SSID"
#define WIFI_PASSWORD       "YOUR_WIFI_PASSWORD"

// Постоянный статический IP для базы в вашей сети 192.168.0.x
#define STATIC_IP_ADDR      192, 168, 0, 200
#define GATEWAY_IP_ADDR     192, 168, 0, 1      // Адрес вашего роутера
#define SUBNET_MASK         255, 255, 255, 0
#define DNS_IP_ADDR         192, 168, 0, 1

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
