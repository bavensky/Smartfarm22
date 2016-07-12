#include <DHT.h>

#define DHTPIN 12
#define DHTTYPE DHT22   // DHT 22  (AM2302), AM2321
DHT dht(DHTPIN, DHTTYPE);

int soilValue = 0;

void read_sensor(float* t_dht, float* h_dht);

void on_prepare_data(JsonObject * root) {
  float t_dht;
  float h_dht;
  read_sensor(&t_dht, &h_dht);
  
  soilValue = map(analogRead(SOIL), 1023, 300, 1, 10);

  JsonObject& data = (*root)["d"];
  JsonObject& info = (*root)["info"];

  data["myName"] = ALIAS;
  data["temp"] = t_dht;
  data["humid"] = h_dht;
  data["soil"] = soilValue;
  data["millis"] = millis();
};


void read_sensor(float* t_dht, float* h_dht) {
  static float _t_dht;
  static float _h_dht;
  _t_dht = dht.readTemperature();
  _h_dht = dht.readHumidity();

  if (!isnan(_t_dht)) {
    *t_dht = _t_dht;
    *h_dht = _h_dht;
  }
}
