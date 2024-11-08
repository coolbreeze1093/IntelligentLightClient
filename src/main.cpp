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
const std::string rev_type_deviceList = "deviceList";
const std::string rev_type_lightInfo = "lightInfo";
const std::string send_type_lightInfo = "lightInfo";
const std::string send_type_deviceList = "deviceList";
const std::string send_type_querylightInfo = "querylightInfo";

const std::string value_localip = "localip";
const std::string value_brightness = "brightness";
const std::string value_model = "model";
const std::string value_deviceIp = "deviceIp";
const std::string value_deviceName = "deviceName";
const std::string value_lightInfo = "lightInfo";

const std::string specialConnector = "*&*";

const std::map<std::string, ESP32PWM> lightMap{{"12", m_pwm_12}, {"13", m_pwm_13}, {"14", m_pwm_14}};

WiFiUDP udp;

std::map<std::string,int> brightnessMap{{"12",0},{"13",0},{"14",0}};

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
  if (_type == rev_type_lightInfo)
  {
    std::string _lightName = doc[value_lightInfo];
    std::vector<std::string> _LightVec = splitString(_lightName,specialConnector);
    for(auto var:_LightVec)
    {
      int _value=doc[var];
      brightnessMap.insert(std::make_pair<std::string,int>(var,_value));
      lightMap[var].write(_value);
    }
    Serial.printf("%s: %d\n", value_brightness.c_str(), value);
  }
  else if (_type == rev_type_deviceList)
  {
    JsonDocument _senddoc;
    _senddoc[key_type] = send_type_deviceList;
    _senddoc[value_deviceName] = "flower";
    _senddoc[value_deviceIp] = WiFi.localIP().toString();
    std::string _lightList;
    int _i = 1;
    for (auto var : lightMap)
    {
      _lightList += var.first;
      if (_i < lightMap.count())
      {
        _lightList += specialConnector;
      }

      _i++;
    }

    _senddoc[value_lightInfo] = _lightList;

    String _remotip = doc[value_localip];

    IPAddress _ipa;
    _ipa.fromString(_remotip.c_str());
    String jsonData;
    serializeJson(_senddoc, jsonData);

    udp.beginPacket(_ipa, sendPort);
    udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
    udp.endPacket();
  }
  else if (_type == send_type_querylightInfo)
  {
    JsonDocument _senddoc;
    _senddoc[key_type] = send_type_lightInfo;
    std::string _lightName = doc[value_lightInfo];
    std::vector<std::string> _LightVec = splitString(_lightName,specialConnector);
    for(auto var:_LightVec)
    {
      _senddoc[var]=brightnessMap[var];
    }
    _senddoc[value_lightInfo] = _lightName;
    
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
