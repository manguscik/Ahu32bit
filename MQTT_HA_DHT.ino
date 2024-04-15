#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include "DHT.h"
#include <ArduinoJson.h>

#define DHTPIN 2
#define DHTTYPE DHT21
DHT dht(DHTPIN, DHTTYPE);

const char *WIFI_SSID = "SSID";
const char *WIFI_PASSWORD = "HASLOWIFI";

const char* mqttServer = "ADRES_IP_SERWERA_MQTT"; // The IP of your MQTT broker
const int mqttPort = 1883;
const char* mqttUser = "LOGIN";
const char* mqttPassword = "HASLO";

float humidity;
float temperature;

int sensorPin = A0; 
int moisture = 0;

int moisture_low = 650;
int moisture_high = 350;
int moisturePercentage = 0;

// My own numerical system for registering devices.
int sensorNumber = 1;
String mqttName = "Plant sensor " + String(sensorNumber);
String stateTopic = "ahu32bit/state";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

void setup() {
  Serial.begin(9600);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.println("Connecting to Wi-Fi");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  Serial.println("Connected to Wi-Fi");

  client.setServer(mqttServer, mqttPort);

  Serial.println("Connecting to MQTT");

  while (!client.connected()) {
    Serial.print(".");

    if (client.connect(mqttName.c_str(), mqttUser, mqttPassword)) {
      client.setBufferSize(512);
      Serial.println("Connected to MQTT");

      sendMQTTTemperatureDiscoveryMsg();
      sendMQTTHumidityDiscoveryMsg();
      sendMQTTMoistureDiscoveryMsg();

    } else {

      Serial.println("failed with state ");
      Serial.print(client.state());
      delay(2000);

    }
  }

  dht.begin();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("===== Sending Data =====");

    humidity = dht.readHumidity();
    temperature = dht.readTemperature();
    moisture = analogRead(sensorPin);

    if (isnan(humidity)) {
      humidity = 0;
    }

    if (isnan(temperature)) {
      temperature = 0;
    }

    // Map moisture sensor values to a percentage value
    moisturePercentage = map(moisture, moisture_low, moisture_high, 0, 100);

    DynamicJsonDocument doc(1024);
    char buffer[512];

    doc["humidity"] = humidity;
    doc["temperature"]   = temperature;
    doc["moisture"] = moisturePercentage;

    size_t n = serializeJson(doc, buffer);

    bool published = client.publish(stateTopic.c_str(), buffer, n);

    // Print the sensor values to Serial out (for debugging)
    Serial.println("published: ");
    Serial.println(published);
    Serial.println("humidity: ");
    Serial.println(humidity);
    Serial.println("temperature: ");
    Serial.println(temperature);
    Serial.println("moisture %: ");
    Serial.println(moisturePercentage);
  }
  else {
    Serial.println("WiFi Disconnected");
  }

  // Go into deep sleep mode for 60 seconds
  Serial.println("Deep sleep mode for 60 seconds");
  ESP.deepSleep(10e6);
}

void sendMQTTTemperatureDiscoveryMsg() {
 
  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Ahu32bit T_Supply";
  doc["unique_id"] = "Ahu32bit T_Supply";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.temperature|default(0) }}";
  JsonObject device  = doc.createNestedObject("device");
  device["identifiers"] = "Sterownik pompy ciepla";
  device["model"] = "Sterownik pompy ciepla AHU32BIT";
  device["name"] = "Sterownik pompy ciepla";   
  device["manufacturer"] = "ESP8266"; 
  device["sw_version"] = "Firmware LCD v1.2.3"; 

  serializeJson(doc, buffer);

  client.publish("homeassistant/sensor/ahu32bit/t_supply/config", buffer);

   Serial.println(buffer);
}

void sendMQTTHumidityDiscoveryMsg() {
  String discoveryTopic = "homeassistant/sensor/ahu32bit/t_return/config";

  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Ahu32bit T_Return";
  doc["unique_id"] = "Ahu32bit T_Return";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.humidity|default(0) }}";
  JsonObject device  = doc.createNestedObject("device");
  device["identifiers"] = "Sterownik pompy ciepla";
  device["model"] = "Sterownik pompy ciepla AHU32BIT";
  device["name"] = "Sterownik pompy ciepla";   
  device["manufacturer"] = "ESP8266"; 
  device["sw_version"] = "Firmware LCD v1.2.3"; 

  serializeJson(doc, buffer);

  client.publish("homeassistant/sensor/ahu32bit/t_return/config", buffer);
 
 Serial.println(buffer);
}

void sendMQTTMoistureDiscoveryMsg() {
  DynamicJsonDocument doc(1024);
  char buffer[512];

  doc["name"] = "Ahu32bit T_Gas";
  doc["unique_id"] = "Ahu32bit T_Gas";
  doc["stat_t"]   = stateTopic;
  doc["unit_of_meas"] = "°C";
  doc["dev_cla"] = "temperature";  
  doc["frc_upd"] = true;
  doc["val_tpl"] = "{{ value_json.moisture|default(0) }}";
  JsonObject device  = doc.createNestedObject("device");
  device["identifiers"] = "Sterownik pompy ciepla";
  device["model"] = "Sterownik pompy ciepla AHU32BIT";
  device["name"] = "Sterownik pompy ciepla";   
  device["manufacturer"] = "ESP8266"; 
  device["sw_version"] = "Firmware LCD v1.2.3"; 
  
  serializeJson(doc, buffer);

  client.publish("homeassistant/sensor/ahu32bit/t_gas/config", buffer);

 Serial.println(buffer);
}

void loop() {}
