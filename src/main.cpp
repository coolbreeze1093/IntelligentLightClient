#include <Arduino.h>
#include <ESP32PWM.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <string>

ESP32PWM m_pwm;
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

WiFiUDP udp;

int brightness=0;

void setupWiFi()
{
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  WiFi.setAutoReconnect(true);
  Serial.println("\nConnected to WiFi!");
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
    int value = doc[value_brightness];
    // uint32_t _value = map(value, 0, 100, 0, 1024);
    brightness=value;
    m_pwm.write(value);
    Serial.printf("%s: %d\n", value_brightness.c_str(), value);
  }
  else if (_type == rev_type_deviceList)
  {
    JsonDocument _senddoc;
    _senddoc[key_type] = send_type_deviceList;
    _senddoc[value_deviceName] = "flower";
    _senddoc[value_deviceIp] = WiFi.localIP().toString();

    String _remotip = doc[value_localip];

    IPAddress _ipa;
    _ipa.fromString(_remotip.c_str());
    String jsonData;
    serializeJson(_senddoc, jsonData);

    udp.beginPacket(_ipa, sendPort);
    udp.write((uint8_t *)jsonData.c_str(), jsonData.length());
    udp.endPacket();
  }
  else if(_type == send_type_querylightInfo)
  {
    JsonDocument _senddoc;
    _senddoc[key_type] = send_type_lightInfo;
    _senddoc[value_brightness] = brightness;

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

void udpReceiveTask(void *param)
{
  char buffer[255];
  while (true)
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
}

void setup()
{
  Serial.begin(115200);
  setupWiFi();

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  m_pwm.attachPin(13, 10000, 10);
  udp.begin(revPort);

  for (size_t i = 0; i < 2; i++)
  {
    m_pwm.write(5);
    delay(300);
    m_pwm.write(0);
    delay(300);
  }

  xTaskCreatePinnedToCore(udpReceiveTask, "UDP Receive Task", 4096, NULL, 1, NULL, 1);
}

void loop()
{
  // 其他代码
  delay(500);
}
