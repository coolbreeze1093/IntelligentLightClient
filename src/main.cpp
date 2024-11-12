#include <Arduino.h>
#include <ESP32PWM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <string>
#include <map>
#include "utility.h"

ESP32PWM m_pwm_13;
ESP32PWM m_pwm_12;
ESP32PWM m_pwm_14;
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

std::map<std::string, ESP32PWM> lightMap{{"12", m_pwm_12}, {"13", m_pwm_13}, {"14", m_pwm_14}};

WiFiUDP udp;

std::map<std::string, int> brightnessMap;

bool isConnectedToLight = false;

void setupWiFi()
{
  udp.stop();
  WiFi.disconnect(true);
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");

  while (WiFi.status() != WL_CONNECTED)
  {
    isConnectedToLight = false;
    delay(500);
    Serial.print(".");
  }

  isConnectedToLight = true;

  WiFi.setAutoReconnect(true);
  Serial.println("\nConnected to WiFi!");
  delay(1000);

  udp.begin(revPort);
}

void handlePacket(char *buffer, int len)
{
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buffer);
  if (error)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
    return;
  }

  std::string _type = doc[key_type];
  if (_type == type_setLampBrightness)
  {
    /*
    {"type":"type_setLampBrightness",
    "value_brightness":{"12":33,"13":45}
    }
    */
    JsonObject _lightInfo = doc[value_brightness].as<JsonObject>();
    for (auto var : _lightInfo)
    {
      std::string _key = var.key().c_str();
      int _value = var.value().as<int>();
      brightnessMap[_key] = _value;
      lightMap[_key].write(_value);
    }
    Serial.printf("%s\n", value_brightness.c_str());
  }
  else if (_type == type_scanDeviceList)
  {
    /*{"type":"type_scanDeviceList",
    "value_deviceName":"flower",
    "value_deviceIp":"127.0.0.1",
    "value_ledLightList":[
    "12","13"
    ]
    }*/
    JsonDocument _senddoc;
    JsonObject _root = _senddoc.to<JsonObject>();
    _root[key_type] = type_scanDeviceList;
    _root[value_deviceName] = "flower";
    _root[value_deviceIp] = WiFi.localIP().toString();
    JsonArray _lightList=_root.createNestedArray(value_ledLightList);

    for (auto var : lightMap)
    {
      _lightList.add<std::string>(var.first);
    }

    String _remotip = doc[value_localip];

    IPAddress _ipa;
    _ipa.fromString(_remotip.c_str());
    String jsonData;
    serializeJson(_senddoc, jsonData);

    udp.beginPacket(_ipa, sendPort);
    udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
    udp.endPacket();
  }
  else if (_type == type_queryLampBrightness)
  {
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
    JsonDocument _senddoc;
    JsonObject _root = _senddoc.to<JsonObject>();
    _root[key_type] = type_queryLampBrightness;
    JsonObject _lightInfo=_root.createNestedObject(value_ledLightList);

    JsonArray _lightName = doc[value_ledLightList].as<JsonArray>();
    for (auto var : _lightName)
    {
      std::string _var=var.as<std::string>();
      _lightInfo[_var] = brightnessMap[_var];
    }

    String _remotip = doc[value_localip];

    IPAddress _ipa;
    _ipa.fromString(_remotip.c_str());
    String jsonData;
    serializeJson(_senddoc, jsonData);

    udp.beginPacket(_ipa, sendPort);
    udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
    udp.endPacket();
  }
}

void reconnectWiFi()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WiFi disconnected, attempting to reconnect...");
    setupWiFi();
  }
}

void udpReceiveTask(void *param)
{
  char buffer[255];
  while (true)
  {
    if (WiFi.status() == WL_DISCONNECTED)
    {
      delay(2000);
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
          Serial.printf("Received packet from %s:%d\n", udp.remoteIP().toString().c_str(), udp.remotePort());
          handlePacket(buffer, len);
        }
      }
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    else
    {
    }
  }
}

void setup()
{
  Serial.begin(115200);
  setupWiFi();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  m_pwm_13.attachPin(13, 10000, 10);
  m_pwm_12.attachPin(12, 10000, 10);
  m_pwm_14.attachPin(14, 10000, 10);

  pinMode(2, OUTPUT);

  xTaskCreatePinnedToCore(udpReceiveTask, "UDP Receive Task", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  // 其他代码
  if (isConnectedToLight)
  {
    digitalWrite(2, HIGH);
  }
  else
  {
    digitalWrite(2, LOW);
  }
  delay(500);
}
