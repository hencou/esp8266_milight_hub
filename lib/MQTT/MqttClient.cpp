#include <stddef.h>
#include <MqttClient.h>
#include <TokenIterator.h>
#include <UrlTokenBindings.h>
#include <IntParsing.h>
#include <ArduinoJson.h>
#include <WiFiClient.h>
#include <MiLightRadioConfig.h>
#include <AboutHelper.h>
#include <WiFiManager.h>

static const char* STATUS_CONNECTED = "connected";
static const char* STATUS_DISCONNECTED = "disconnected_clean";
static const char* STATUS_LWT_DISCONNECTED = "disconnected_unclean";

MqttClient::MqttClient(Settings& settings, MiLightClient*& milightClient)
  : mqttClient(tcpClient),
    milightClient(milightClient),
    settings(settings),
    lastConnectAttempt(0)
{
  String strDomain = settings.mqttServer();
  this->domain = new char[strDomain.length() + 1];
  strcpy(this->domain, strDomain.c_str());
}

MqttClient::~MqttClient() {
  String aboutStr = generateConnectionStatusMessage(STATUS_DISCONNECTED);
  mqttClient.publish(settings.mqttClientStatusTopic.c_str(), aboutStr.c_str(), true);
  mqttClient.disconnect();
  delete this->domain;
}

void MqttClient::begin() {
#ifdef MQTT_DEBUG
  printf_P(
    PSTR("MqttClient - Connecting to: %s\nparsed:%s:%u\n"),
    settings._mqttServer.c_str(),
    settings.mqttServer().c_str(),
    settings.mqttPort()
  );
#endif

  mqttClient.setServer(this->domain, settings.mqttPort());
  mqttClient.setCallback(
    [this](char* topic, byte* payload, int length) {
      this->publishCallback(topic, payload, length);
    }
  );
  reconnect();
}

bool MqttClient::connect() {

  if (WiFi.status() != WL_CONNECTED) return false;

  char nameBuffer[30];
  sprintf_P(nameBuffer, PSTR("milight-hub-%u"), ESP.getChipId());

#ifdef MQTT_DEBUG
    Serial.println(F("MqttClient - connecting"));
#endif

  if (settings.mqttUsername.length() > 0 && settings.mqttClientStatusTopic.length() > 0) {
    return mqttClient.connect(
      nameBuffer,
      settings.mqttUsername.c_str(),
      settings.mqttPassword.c_str(),
      settings.mqttClientStatusTopic.c_str(),
      2,
      true,
      generateConnectionStatusMessage(STATUS_LWT_DISCONNECTED).c_str()
    );
  } else if (settings.mqttUsername.length() > 0) {
    return mqttClient.connect(
      nameBuffer,
      settings.mqttUsername.c_str(),
      settings.mqttPassword.c_str()
    );
  } else if (settings.mqttClientStatusTopic.length() > 0) {
    return mqttClient.connect(
      nameBuffer,
      settings.mqttClientStatusTopic.c_str(),
      2,
      true,
      generateConnectionStatusMessage(STATUS_LWT_DISCONNECTED).c_str()
    );
  } else {
    return mqttClient.connect(nameBuffer);
  }
}

void MqttClient::sendBirthMessage() {
  if (settings.mqttClientStatusTopic.length() > 0) {
    String aboutStr = generateConnectionStatusMessage(STATUS_CONNECTED);
    mqttClient.publish(settings.mqttClientStatusTopic.c_str(), aboutStr.c_str(), true);
  }
}

void MqttClient::reconnect() {
  if (lastConnectAttempt > 0 && (millis() - lastConnectAttempt) < MQTT_CONNECTION_ATTEMPT_FREQUENCY) {
    return;
  }

  if (WiFi.status() != WL_CONNECTED) return;

  if (! mqttClient.connected()) {
    if (connect()) {
      subscribe();
      sendBirthMessage();

#ifdef MQTT_DEBUG
      Serial.println(F("MqttClient - Successfully connected to MQTT server"));
#endif
    } else {
      Serial.println(F("ERROR: Failed to connect to MQTT server"));
    }
  }

  lastConnectAttempt = millis();
}

void MqttClient::handleClient() {
  reconnect();
  mqttClient.loop();
}

void MqttClient::sendUpdate(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update) {
  publish(settings.mqttUpdateTopicPattern, remoteConfig, deviceId, groupId, update);
}

void MqttClient::sendState(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update) {
  publish(settings.mqttStateTopicPattern, remoteConfig, deviceId, groupId, update, true);
}

void MqttClient::subscribe() {
  String topic = settings.mqttTopicPattern;

  topic.replace(":device_id", "+");
  topic.replace(":hex_device_id", "+");
  topic.replace(":dec_device_id", "+");
  topic.replace(":group_id", "+");
  topic.replace(":device_type", "+");

#ifdef MQTT_DEBUG
  printf_P(PSTR("MqttClient - subscribing to topic: %s\n"), topic.c_str());
#endif

  mqttClient.subscribe(topic.c_str());
}

void MqttClient::send(const char* topic, const char* message, const bool retain) {

  if (!mqttClient.connected()) return;

  size_t len = strlen(message);
  size_t topicLen = strlen(topic);

  if ((topicLen + len + 10) < MQTT_MAX_PACKET_SIZE ) {
    mqttClient.publish(topic, message, retain);
  } else {
    const uint8_t* messageBuffer = reinterpret_cast<const uint8_t*>(message);
    mqttClient.beginPublish(topic, len, retain);

#ifdef MQTT_DEBUG
    Serial.printf_P(PSTR("Printing message in parts because it's too large for the packet buffer (%d bytes)"), len);
#endif

    for (size_t i = 0; i < len; i += MQTT_PACKET_CHUNK_SIZE) {
      size_t toWrite = std::min(static_cast<size_t>(MQTT_PACKET_CHUNK_SIZE), len - i);
      mqttClient.write(messageBuffer+i, toWrite);
#ifdef MQTT_DEBUG
      Serial.printf_P(PSTR("  Wrote %d bytes\n"), toWrite);
#endif
    }

    mqttClient.endPublish();
  }
}

void MqttClient::publish(
  
  const String& _topic,
  const MiLightRemoteConfig &remoteConfig,
  uint16_t deviceId,
  uint16_t groupId,
  const char* message,
  const bool retain
) {
  if (_topic.length() == 0) {
    return;
  }
  if (!mqttClient.connected()) return;
  
  String topic = _topic;
  MqttClient::bindTopicString(topic, remoteConfig, deviceId, groupId);

#ifdef MQTT_DEBUG
  printf("MqttClient - publishing update to %s\n", topic.c_str());
#endif

  mqttClient.publish(topic.c_str(), message, retain);
}

void MqttClient::publishCallback(char* topic, byte* payload, int length) {
  uint16_t deviceId = 0;
  uint8_t groupId = 0;
  const MiLightRemoteConfig* config = &FUT092Config;
  char cstrPayload[length + 1];
  cstrPayload[length] = 0;
  memcpy(cstrPayload, payload, sizeof(byte)*length);

#ifdef MQTT_DEBUG
  printf("MqttClient - Got message on topic: %s\n%s\n", topic, cstrPayload);
#endif

  char topicPattern[settings.mqttTopicPattern.length()];
  strcpy(topicPattern, settings.mqttTopicPattern.c_str());

  TokenIterator patternIterator(topicPattern, settings.mqttTopicPattern.length(), '/');
  TokenIterator topicIterator(topic, strlen(topic), '/');
  UrlTokenBindings tokenBindings(patternIterator, topicIterator);

  if (tokenBindings.hasBinding("device_id")) {
    deviceId = parseInt<uint16_t>(tokenBindings.get("device_id"));
  } else if (tokenBindings.hasBinding("hex_device_id")) {
    deviceId = parseInt<uint16_t>(tokenBindings.get("hex_device_id"));
  } else if (tokenBindings.hasBinding("dec_device_id")) {
    deviceId = parseInt<uint16_t>(tokenBindings.get("dec_device_id"));
  }

  if (tokenBindings.hasBinding("group_id")) {
    groupId = parseInt<uint16_t>(tokenBindings.get("group_id"));
  }

  if (tokenBindings.hasBinding("device_type")) {
    config = MiLightRemoteConfig::fromType(tokenBindings.get("device_type"));

    if (config == NULL) {
      Serial.println(F("MqttClient - ERROR: could not extract device_type from topic"));
      return;
    }
  } else {
    Serial.println(F("MqttClient - WARNING: could not find device_type token.  Defaulting to FUT092.\n"));
  }

  StaticJsonDocument<400> buffer;
  deserializeJson(buffer, cstrPayload);
  JsonObject obj = buffer.as<JsonObject>();

#ifdef MQTT_DEBUG
  printf("MqttClient - device %04X, group %u\n", deviceId, groupId);
#endif

  milightClient->prepare(config, deviceId, groupId);
  milightClient->update(obj);
}

inline void MqttClient::bindTopicString(
  String& topicPattern,
  const MiLightRemoteConfig& remoteConfig,
  const uint16_t deviceId,
  const uint16_t groupId
) {
  String deviceIdHex = String(deviceId, 16);
  deviceIdHex.toUpperCase();
  deviceIdHex = String("0x") + deviceIdHex;

  topicPattern.replace(":device_id", deviceIdHex);
  topicPattern.replace(":hex_device_id", deviceIdHex);
  topicPattern.replace(":dec_device_id", String(deviceId));
  topicPattern.replace(":group_id", String(groupId));
  topicPattern.replace(":device_type", remoteConfig.name);
}

String MqttClient::generateConnectionStatusMessage(const char* connectionStatus) {
  if (settings.simpleMqttClientStatus) {
    // Don't expand disconnect type for simple status
    if (0 == strcmp(connectionStatus, STATUS_CONNECTED)) {
      return connectionStatus;
    } else {
      return "disconnected";
    }
  } else {
    StaticJsonDocument<1024> json;
    json["status"] = connectionStatus;

    // Fill other fields
    AboutHelper::generateAboutObject(json, true);

    String response;
    serializeJson(json, response);

    return response;
  }
}