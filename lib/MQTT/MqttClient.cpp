#include <stddef.h>
#include <MqttClient.h>
#include <TokenIterator.h>
#include <UrlTokenBindings.h>
#include <IntParsing.h>
#include <ArduinoJson.h>
#include <MiLightRadioConfig.h>
#include <AboutHelper.h>

static const char* STATUS_CONNECTED = "connected";
static const char* STATUS_DISCONNECTED = "disconnected_clean";
static const char* STATUS_LWT_DISCONNECTED = "disconnected_unclean";

MqttClient::MqttClient(Settings& settings, MiLightClient*& milightClient, const char* outTopic)
  : milightClient(milightClient),
    settings(settings),
    outTopic(outTopic)
{
}

MqttClient::~MqttClient() {
}

//<added by HC>
void MqttClient::setCallback(std::function<void(const char *topic, const char *msg, const bool retain)> _callback) {
    callback = _callback;
}
//</added by HC>

void MqttClient::sendUpdate(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update) {
  publish(settings.mqttUpdateTopicPattern, remoteConfig, deviceId, groupId, update);
}

void MqttClient::sendState(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update) {
  publish(settings.mqttStateTopicPattern, remoteConfig, deviceId, groupId, update, true);
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

  String topic = _topic;
  MqttClient::bindTopicString(topic, remoteConfig, deviceId, groupId);

  //<added by HC>
  if (! callback) {
      return;
  }

  char ctopic[64];
  snprintf(ctopic, sizeof(ctopic), "%s%s", outTopic, topic.c_str());

  #ifdef MQTT_DEBUG
    printf("MqttClient - publishing update to %s\r\n", ctopic);
  #endif

  callback(ctopic, message, retain);
  //</added by HC>
}

void MqttClient::fromMeshCallback(const char *topic, const char *msg) {
  uint16_t deviceId = 0;
  uint8_t groupId = 0;
  const MiLightRemoteConfig* config = &FUT092Config;

  #ifdef MQTT_DEBUG
    printf_P(PSTR("MqttClient - Got message on topic: %s : %s\r\n"), topic, msg);
  #endif

  char topicPattern[settings.mqttTopicPattern.length()];
  strcpy(topicPattern, settings.mqttTopicPattern.c_str());

  TokenIterator patternIterator(topicPattern, settings.mqttTopicPattern.length(), '/');
  TokenIterator topicIterator((char*)topic, strlen(topic), '/');
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
  deserializeJson(buffer, msg);
  JsonObject obj = buffer.as<JsonObject>();

  //accept incoming MQTT command only when deviceId is in use as UDP device
  for (size_t i = 0; i < settings.gatewayConfigs.size(); i++) {
    if (deviceId == settings.gatewayConfigs[i]->deviceId) {

    #ifdef MQTT_DEBUG
      printf_P(PSTR("MqttClient - device %04X, group %u\r\n"), deviceId, groupId);
    #endif

    milightClient->prepare(config, deviceId, groupId);
    milightClient->update(obj);
    milightClient->update(obj);
    }
  }
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