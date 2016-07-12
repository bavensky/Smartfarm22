#ifdef ESP8266
extern "C" {
#include "user_interface.h"
}
#endif

void onFoundgear(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.print("Found new member --> ");
  for (int i = 0; i < msglen; i++) {
    Serial.print((char)msg[i]);
  }
  Serial.println();
}

void onLostgear(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.print("Lost member --> ");
  for (int i = 0; i < msglen; i++) {
    Serial.print((char)msg[i]);
  }
  Serial.println();
}

void onConnected(char *attribute, uint8_t* msg, unsigned int msglen) {
  Serial.println("Connected to NETPIE...");
  microgear.setName(ALIAS);
}

static String ipStr = "";
void _hook_on_prepare_data(JsonObject * root) {
  JsonObject& data = (*root)["d"];
  JsonObject& info = (*root)["info"];

  if (ipStr == "") {
    IPAddress ip = WiFi.localIP();
    ipStr = String(ip[0]) + '.' + String(ip[1]) +
            '.' + String(ip[2]) + '.' + String(ip[3]);
  }
  (data)["heap"] = ESP.getFreeHeap();
  (info)["ip"] = ipStr.c_str();
  (data)["rssi"] = WiFi.RSSI();
  (data)["seconds"] = millis() / 1000;

};

void _constructor() {
  JsonObject& _r = jsonRootBuffer.createObject();
  JsonObject& _info = _r.createNestedObject("info");
  JsonObject& _d = jsonDBuffer.createObject();

  _info["ssid"] =  String(ssid);
  _info["flash_size"] = ESP.getFlashChipSize();
  _info["flash_id"] = String(ESP.getFlashChipId(), HEX);
  _info["chip_id"] = String(ESP.getChipId(), HEX);
  _info["mac"] = String(WiFi.macAddress().c_str());
  _info["core"] =  String(ESP.getCoreVersion().c_str());
  _info["sdk"] = String(ESP.getSdkVersion());

  _r["info"] = _info;
  _r["d"] = _d;

  cmmc_root = &_r;
  cmmc_info = &_info;
}

void init_netpie() {
  /* Event listener */
  microgear.on(MESSAGE, onMsghandler);
  microgear.on(PRESENT, onFoundgear);
  microgear.on(ABSENT, onLostgear);
  microgear.on(CONNECTED, onConnected);

  //uncomment the line below if you want to reset token -->
  //microgear.resetToken();
  microgear.init(KEY, SECRET, ALIAS);
  microgear.connect(APPID);
}

void _publish() {
  String topic = String("/gearname/") + ALIAS + String("/status");
  _hook_on_prepare_data(cmmc_root);
  on_prepare_data(cmmc_root);
  Serial.printf("[%lu] Publish to %s ... \r\n", millis(), topic.c_str());
  strcpy(jsonStrbuffer, "");
  cmmc_root->printTo(jsonStrbuffer, sizeof(jsonStrbuffer));
  Serial.println(jsonStrbuffer);

  microgear.publish(const_cast<char*>(topic.c_str()), jsonStrbuffer);
}
