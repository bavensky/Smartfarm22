#define MQTT_MAX_PACKET_SIZE 1024

#include <AuthClient.h>
#include <MicroGear.h>
#include <ArduinoJson.h>
#include <MQTTClient.h>
#include <SHA1.h>
#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <EEPROM.h>
#include <MicroGear.h>
#include "timer.hpp"
#include "CMMC_Blink.hpp"
CMMC_Blink blinker;

const char* ssid     = "ESPERT-3020";
const char* password = "espertap";

// NETPIE.io : SmartFarm22 (smartfarm)
#define APPID        "SmartFarm22"
#define KEY          "AlUrcRa3A8c8whu"
#define SECRET       "cSPSeq3qvvFzSFjWjjkqUaioj"
#define ALIAS        "espsmartfarm"

#define PUBLISH_EVERY_SECS (2*1000)

WiFiClient client;
AuthClient *authclient;
CMMC_Interval timer001;

// MQTT CONNECTOR CONCEPT
char jsonStrbuffer[1024];
JsonObject *cmmc_root;
JsonObject *d;
JsonObject *cmmc_info;

StaticJsonBuffer<800> jsonRootBuffer;
StaticJsonBuffer<512> jsonDBuffer;

int timer = 0;
MicroGear microgear(client);

#define SOIL A0
#define RELAY 13

#include "_publish.h"
#include "_receive.h"
#include "utils.h"

void init_hardware();
void init_wifi();
void init_netpie();
void microgear_loop();

void setup() {
  _constructor();
  init_hardware();
  init_wifi();
  init_netpie();
  blinker.detach();
}

void loop() {
  microgear_loop();
}

void microgear_loop() {
  if (microgear.connected()) {
    microgear.loop();
    timer001.every_ms(PUBLISH_EVERY_SECS, [&]() {
      _publish();
    });
  }
  else {
    Serial.println("connection lost, reconnect...");
    microgear.connect(APPID);
  }
}

void init_hardware() {
  blinker.init();
  Serial.begin(115200);
  blinker.blink(50, LED_BUILTIN);
  delay(200);
  
  pinMode(RELAY, OUTPUT);
  digitalWrite(RELAY, HIGH);
  Serial.println("Starting...");
}

void init_wifi() {
  if (WiFi.begin(ssid, password)) {
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
  }

  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  blinker.blink(200, LED_BUILTIN);
}



