#include <Arduino.h>
#include <ESP32PWM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <string>
#include <map>
#include "utility.h"

ESP32PWM *m_pwm_13=new ESP32PWM;
ESP32PWM *m_pwm_12=new ESP32PWM;
ESP32PWM *m_pwm_14=new ESP32PWM;
const char *ssid = "secret";
const char *password = "dahuang123";

const int revPort = 56696;
const int sendPort = 56698;

const std::string key_type = "type";
const std::string type_scanDeviceList = "ScanDeviceList";
const std::string type_setLampBrightness = "SetLampBrightness";
const std::string type_queryLampBrightness = "QuerylightInfo";

const std::string value_localip = "localip";
const std::string value_brightness = "brightness";
const std::string value_model = "model";
const std::string value_deviceIp = "deviceIp";
const std::string value_deviceName = "deviceName";
const std::string value_ledLightList = "lightInfo";

const std::string specialConnector = "*&*";

std::map<std::string, ESP32PWM*> lightMap{{"12", m_pwm_12}, {"13", m_pwm_13}, {"14", m_pwm_14}};

WiFiUDP udp;

std::map<std::string, int> brightnessMap;

bool isConnectedToLight = false;

TaskHandle_t *udpTaskHandle = NULL;

void setupWiFi()
{
  udp.stop();
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    isConnectedToLight = false;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.print(".");
  }

  isConnectedToLight = true;

  //WiFi.setAutoReconnect(true);
  Serial.println("\nConnected to WiFi!");
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  udp.begin(revPort);
}

void handleSetBrightness(const JsonObject& lightInfo) {
  /*
    {"type":"type_setLampBrightness",
    "value_brightness":{"12":33,"13":45}
    }
    */
  for (auto var : lightInfo) {
    std::string key = var.key().c_str();
    int value = var.value().as<int>();
    brightnessMap[key] = value;
    if (lightMap[key]) lightMap[key]->write(value);
  }
}

void handleScanDeviceList(const JsonObject& doc) {
  /*{"type":"type_scanDeviceList",
    "value_deviceName":"flower",
    "value_deviceIp":"127.0.0.1",
    "value_ledLightList":[
    "12","13"
    ]
    }*/
  JsonDocument sendDoc;
  JsonObject root = sendDoc.to<JsonObject>();
  root[key_type] = type_scanDeviceList;
  root[value_deviceName] = "flower";
  root[value_deviceIp] = WiFi.localIP().toString();
  JsonArray lightList = root[value_ledLightList].to<JsonArray>();

  for (const auto& var : lightMap) lightList.add(var.first);
  
  IPAddress ip;
  ip.fromString(doc[value_localip].as<const char*>());
  String jsonData;
  serializeJson(sendDoc, jsonData);
  udp.beginPacket(ip, sendPort);
  udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
  udp.endPacket();
}

void handleQueryBrightness(const JsonObject& doc) {
  /*
    {
    "value_ledLightList":["12","13"]
    }
    */
    /*
    {
    "type":"send_type_lightInfo",
    "value_ledLightList":{"12":12,"13":33}
    }
    */
  JsonDocument sendDoc;
  JsonObject root = sendDoc.to<JsonObject>();
  root[key_type] = type_queryLampBrightness;
  JsonObject lightInfo = root[value_ledLightList].to<JsonObject>();

  for (const auto& var : brightnessMap) lightInfo[var.first] = var.second;
  
  IPAddress ip;
  ip.fromString(doc[value_localip].as<const char*>());
  String jsonData;
  serializeJson(sendDoc, jsonData);
  udp.beginPacket(ip, sendPort);
  udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
  udp.endPacket();
}

void handlePacket(char *buffer, int len)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buffer);
  if (error)
  {
    log("deserializeJson() failed: ");
    log(error.c_str());
    return;
  }

  std::string type = doc[key_type].as<std::string>();
  if (type == type_setLampBrightness) {
    handleSetBrightness(doc[value_brightness].as<JsonObject>());
  } else if (type == type_scanDeviceList) {
    handleScanDeviceList(doc.as<JsonObject>());
  } else if (type == type_queryLampBrightness) {
    handleQueryBrightness(doc.as<JsonObject>());
  }
}

void reconnectWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    log("WiFi disconnected, attempting to reconnect...");
    setupWiFi();
  }
}

void udpReceiveTask(void *param)
{
  setupWiFi();
  char buffer[512];
  while (true)
  {
    if (WiFi.status() == WL_DISCONNECTED)
    {
      vTaskDelay(2000 / portTICK_PERIOD_MS);
      reconnectWiFi();
    }
    else if (WiFi.status() == WL_CONNECTED)
    {
      int packetSize = udp.parsePacket();
      if (packetSize > 0)
      {
        int len = udp.read(buffer, sizeof(buffer) - 1);
        if (len > 0)
        {
          buffer[len] = '\0';
          log(("Received packet from IP:"+std::string(udp.remoteIP().toString().c_str())+"  port:"+std::to_string(udp.remotePort())).c_str());
          handlePacket(buffer, len);
        }
      }
      vTaskDelay(2 / portTICK_PERIOD_MS);
    }
    else
    {
    }
  }
}

void setup()
{
  Serial.begin(115200);
  

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  m_pwm_13->attachPin(13, 5000, 10);
  m_pwm_12->attachPin(12, 5000, 10);
  m_pwm_14->attachPin(14, 5000, 10);

  pinMode(2, OUTPUT);

  BaseType_t result = xTaskCreatePinnedToCore(udpReceiveTask, "UDP Receive Task", 4096, NULL, 1, udpTaskHandle, 1);
  if (result != pdPASS) {
    // 处理任务创建失败的情况
    log("Task creation failed\n");
  }

}

void loop()
{
  
  if(!isConnectedToLight)
  {
    delay(10000);
  }
  if(!isConnectedToLight)
  {
    vTaskDelete(udpTaskHandle);
    delay(2000);
    BaseType_t result = xTaskCreatePinnedToCore(udpReceiveTask, "UDP Receive Task", 4096, NULL, 1, udpTaskHandle, 1);
    if (result != pdPASS) {
      // 处理任务创建失败的情况
      log("Task creation failed\n");
    }
  }

  digitalWrite(2, isConnectedToLight ? HIGH : LOW);
  
  delay(500);
}
