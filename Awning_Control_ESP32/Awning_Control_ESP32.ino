/*
  This example uses FreeRTOS softwaretimers as there is no built-in Ticker library
*/


#include <WiFi.h>
extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}
#include <AsyncMqttClient.h>
#include <credentials.h>

#define MQTT_HOST IPAddress(192, 168, 0, 203)
#define MQTT_PORT 1883


#define SHORT_SPACE 250
#define SHORT_SIG 350
#define LONG_SIG_DIST 2000
#define BETW 16000

#define SEND_PIN 4

String extendCommand =   "LGSPSPSPSPSPSPSPLGSPLGLGLGSPLGSPSPSN";
String retractCommand =  "LGSPSPSPSPSPSPSPLGSPLGLGLGSPLGSPLGSN";
String manCommand =  "LGSPSPSPSPSPSPSPLGSPLGLGLGLGLN";
String autoCommand = "LGSPSPSPSPSPSPSPLGSPLGLGSPLGLGLGLGSN";

AsyncMqttClient mqttClient;
TimerHandle_t mqttReconnectTimer;
TimerHandle_t wifiReconnectTimer;

void connectToWifi() {
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(mySSID, myPASSWORD);
}

void connectToMqtt() {
  Serial.println("Connecting to MQTT...");
  mqttClient.connect();
  Serial.println("Connecting to MQTT... exit");
}

void WiFiEvent(WiFiEvent_t event) {
  Serial.printf("[WiFi-event] event: %d\n", event);
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.println("WiFi connected");
      Serial.println("IP address: ");
      Serial.println(WiFi.localIP());
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      xTimerStop(mqttReconnectTimer, 0); // ensure we don't reconnect to MQTT while reconnecting to Wi-Fi
      xTimerStart(wifiReconnectTimer, 0);
      break;
  }
}

void onMqttConnect(bool sessionPresent) {
  Serial.println("Connected to MQTT.");
  Serial.print("Session present: ");
  Serial.println(sessionPresent);
  uint16_t packetIdSub = mqttClient.subscribe("curtain/command", 2);
  Serial.print("Subscribing at QoS 2, packetId: ");
  Serial.println(packetIdSub);
  mqttClient.publish("curtain/status", 0, true, "connected");
  Serial.println("Publishing at QoS 0");
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  Serial.println("Disconnected from MQTT.");

  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  Serial.println("Subscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
  Serial.print("  qos: ");
  Serial.println(qos);
}

void onMqttUnsubscribe(uint16_t packetId) {
  Serial.println("Unsubscribe acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  Serial.println("Message received.");
  Serial.print("Payload: ");
  Serial.println(payload);
  /*
    Serial.print("  topic: ");
    Serial.println(topic);
    Serial.print("  qos: ");
    Serial.println(properties.qos);
    Serial.print("  dup: ");
    Serial.println(properties.dup);
    Serial.print("  retain: ");
    Serial.println(properties.retain);
    Serial.print("  len: ");
    Serial.println(len);
    Serial.print("  index: ");
    Serial.println(index);
    Serial.print("  total: ");
    Serial.println(total);
  */
  int dur = payload[1] - 48;
  // Serial.print("  duration: ");
  // Serial.println(dur);

  switch ( payload[0]) {
    case 'E':
      Serial.println("Extend Curtain");
      sendSig(manCommand, 1);
      sendSig(extendCommand, 10 * dur);
      sendSig(retractCommand, 1);
      break;
    case 'R':
      Serial.println("Retract Curtain");
      sendSig(manCommand, 1);
      sendSig(retractCommand, 2);
      break;
    default:
      Serial.println("Error");
      break;
  }
  Serial.println("Done");
}

void onMqttPublish(uint16_t packetId) {
  Serial.println("Publish acknowledged.");
  Serial.print("  packetId: ");
  Serial.println(packetId);
}

void setup() {
  Serial.begin(115200);
  Serial.println();
  Serial.println();
  pinMode(SEND_PIN, OUTPUT);
  digitalWrite(SEND_PIN, LOW);
  delay(100);
  sendSig(manCommand, 1);
  sendSig(retractCommand, 2);

  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));

  WiFi.onEvent(WiFiEvent);

  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setCredentials("admin", "admin");
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  connectToWifi();
}

void loop() {
}

void sendSig(String sig, int duration) {
  if (duration < 1)duration = 1;
  if (duration > 90)duration = 90;
  unsigned long entry = millis();
  while (millis() < entry + duration * 1000) {
    for (int i = 0; i < sig.length(); i++) {
      switch (sig[i]) {
        case 'S':  // short pulse
          sigPuls(SHORT_SIG);
          break;
        case 'L':  // long pulse
          sigPuls(LONG_SIG_DIST);
          break;
        case 'G':  //short space
          delayMicroseconds(SHORT_SPACE);
          break;
        case 'P':  // long space (pause)
          delayMicroseconds(LONG_SIG_DIST);
          break;
        case 'N':  // new command
          delayMicroseconds(BETW);
          break;
        default:
          Serial.println("Error");
          break;
      }
    }
    //   Serial.println(sig);
  }
}

void sigPuls(int duration) {
  digitalWrite(SEND_PIN, HIGH);
  delayMicroseconds(duration);
  digitalWrite(SEND_PIN, LOW);
}
