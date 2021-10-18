/*  =================================================
    
    ESP-Now
    THE RECEIVER

  =================================================== */

#include <Arduino.h>
#include "SoftwareSerial.h"
#include "ArduinoJson.h"

#ifdef ESP32
#include <WiFi.h>
#include <esp_now.h>
#else
#include <ESP8266WiFi.h>
#include <espnow.h>
#endif

// Uncomment to get debug output
#define DEBUG_FLAG

#define GROUP_ID 855544
#define BUZZER_PIN 4 // D2 on Wemos D1 mini, beeps on boot and received msgs

SoftwareSerial SSerial(12, 14); // D6 and D5 on Wemos D1 mini
char end_char = '\n';

void OnDataRecv(uint8_t *mac_addr, uint8_t *buff, uint8_t len)
{
    digitalWrite(LED_BUILTIN, LOW);
    SSerial.write(buff, len);
    SSerial.write(end_char);

#ifdef DEBUG_FLAG

    char msg_raw[256];
    size_t bufflen = sizeof(buff);
    memcpy(msg_raw, buff, bufflen);
    msg_raw[bufflen] = '\0'; // 'str' is now a string

    StaticJsonDocument<256> doc;

    Serial.println(msg_raw);
    deserializeJson(doc, msg_raw);

    if (doc["group_id"] == GROUP_ID)
    {
        Serial.print("Received from ");
        char macAddr[18];
        sprintf(macAddr, "%02X%02X%02X%02X%02X%02X", doc["sensor_id"][0], doc["sensor_id"][1], doc["sensor_id"][2], doc["sensor_id"][3], doc["sensor_id"][4], doc["sensor_id"][5]);
        Serial.print(macAddr);
        Serial.print(", data: ");
        String data_raw;
        serializeJson(doc["data"], data_raw);
        Serial.println(data_raw);
        Serial.println();
    }
    else
    {
        Serial.println("Wrong Group ID, ignoring");
    }
#endif
    digitalWrite(LED_BUILTIN, HIGH);
}

void setup()
{
    Serial.begin(115200);
    SSerial.begin(9600);
    WiFi.mode(WIFI_STA);

#ifdef DEBUG_FLAG
    Serial.println();
    Serial.print("ESP Board MAC Address:  ");
    Serial.println(WiFi.macAddress());
#endif
    if (esp_now_init() != 0)
    {
#ifdef DEBUG_FLAG
        Serial.println("Error initializing ESP-NOW, restarting");
#endif
        delay(1000);
        ESP.restart();
    }
    esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
    esp_now_register_recv_cb(OnDataRecv);

    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
}

void loop() {}