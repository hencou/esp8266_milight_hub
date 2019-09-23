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

MqttClient::MqttClient(Settings& settings, MiLightClient*& milightClient, GroupStateStore& stateStore)
  : mqttClient(tcpClient),
    settings(settings),
    milightClient(milightClient),
    stateStore(stateStore),
    lastConnectAttempt(0),
    connected(false)
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

void MqttClient::onConnect(OnConnectFn fn) {
  this->onConnectFn = fn;
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

  repeatTimer = random(2000, 3000);
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

  if (!connected && mqttClient.connected()) {
    this->connected = true;
    this->onConnectFn();
  } else if (!mqttClient.connected()) {
    this->connected = false;
  }

  //<Added by HC: send command multiple after a second to ensure lamps received the command>
  while (millis() - lastCommandTime > repeatTimer && staleGroups.size() > 0) {
    BulbId bulbId = staleGroups.shift();
    
    StaticJsonDocument<200> json;
    JsonObject message = json.to<JsonObject>();
    
    GroupState* groupState = stateStore.get(bulbId);
    if (groupState != NULL) {
      groupState->applyState(message, bulbId, settings.groupStateFields);

      //Remove bulb_mode commands to avoid flickering
      if (message.containsKey(GroupStateFieldNames::BULB_MODE)) {
        message.remove(GroupStateFieldNames::BULB_MODE);
      }
      if (message.containsKey(GroupStateFieldNames::MODE)) {
        message.remove(GroupStateFieldNames::MODE);
      }
      //Remove effect commands when in white_mode to avoid color flickering
      if (message.containsKey(GroupStateFieldNames::EFFECT)) {
        if (message[GroupStateFieldNames::EFFECT] == "white" || message[GroupStateFieldNames::EFFECT] == "white_mode") {
          message.remove(GroupStateFieldNames::EFFECT);
        }
      }

      milightClient->prepare(bulbId.deviceType, bulbId.deviceId, bulbId.groupId);
      milightClient->update(message);
      milightClient->update(message);
    }
  }
  //</Added by HC
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
  topic.replace(":device_alias", "+");

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

  BulbId bulbId(deviceId, groupId, remoteConfig.type);
  String topic = bindTopicString(_topic, bulbId);

#ifdef MQTT_DEBUG
  printf("MqttClient - publishing update to %s\n", topic.c_str());
#endif

  send(topic.c_str(), message, retain);
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

  if (tokenBindings.hasBinding("device_alias")) {
    String alias = tokenBindings.get("device_alias");
    auto itr = settings.groupIdAliases.find(alias);

    if (itr == settings.groupIdAliases.end()) {
      Serial.printf_P(PSTR("MqttClient - WARNING: could not find device alias: `%s'. Ignoring packet.\n"), alias.c_str());
      return;
    } else {
      BulbId bulbId = itr->second;

      deviceId = bulbId.deviceId;
      config = MiLightRemoteConfig::fromType(bulbId.deviceType);
      groupId = bulbId.groupId;
    }
  } else {
    if (tokenBindings.hasBinding(GroupStateFieldNames::DEVICE_ID)) {
      deviceId = parseInt<uint16_t>(tokenBindings.get(GroupStateFieldNames::DEVICE_ID));
    } else if (tokenBindings.hasBinding("hex_device_id")) {
      deviceId = parseInt<uint16_t>(tokenBindings.get("hex_device_id"));
    } else if (tokenBindings.hasBinding("dec_device_id")) {
      deviceId = parseInt<uint16_t>(tokenBindings.get("dec_device_id"));
    }

    if (tokenBindings.hasBinding(GroupStateFieldNames::GROUP_ID)) {
      groupId = parseInt<uint16_t>(tokenBindings.get(GroupStateFieldNames::GROUP_ID));
    }

    if (tokenBindings.hasBinding(GroupStateFieldNames::DEVICE_TYPE)) {
      config = MiLightRemoteConfig::fromType(tokenBindings.get(GroupStateFieldNames::DEVICE_TYPE));
    } else {
      Serial.println(F("MqttClient - WARNING: could not find device_type token.  Defaulting to FUT092.\n"));
    }
  }

  if (config == NULL) {
    Serial.println(F("MqttClient - ERROR: unknown device_type specified"));
    return;
  }

  StaticJsonDocument<400> buffer;
  deserializeJson(buffer, cstrPayload);
  JsonObject obj = buffer.as<JsonObject>();

  //<changed by HC
  //accept incoming MQTT command only when deviceId is in use as UDP device
  for (size_t i = 0; i < settings.gatewayConfigs.size(); i++) {
    if (deviceId == settings.gatewayConfigs[i]->deviceId) {

    #ifdef MQTT_DEBUG
    printf("MqttClient - device %04X, group %u\n", deviceId, groupId);
    #endif

    milightClient->prepare(config, deviceId, groupId);
    milightClient->update(obj);
    milightClient->update(obj);
    
    BulbId bulbId(deviceId, groupId, config->type);
    staleGroups.push(bulbId);
    lastCommandTime = millis();
    }
  }
  //<changed by HC
}

String MqttClient::bindTopicString(const String& topicPattern, const BulbId& bulbId) {
  String boundTopic = topicPattern;
  String deviceIdHex = bulbId.getHexDeviceId();

  boundTopic.replace(":device_id", deviceIdHex);
  boundTopic.replace(":hex_device_id", deviceIdHex);
  boundTopic.replace(":dec_device_id", String(bulbId.deviceId));
  boundTopic.replace(":group_id", String(bulbId.groupId));
  boundTopic.replace(":device_type", MiLightRemoteTypeHelpers::remoteTypeToString(bulbId.deviceType));

  auto it = settings.findAlias(bulbId.deviceType, bulbId.deviceId, bulbId.groupId);
  if (it != settings.groupIdAliases.end()) {
    boundTopic.replace(":device_alias", it->first);
  } else {
    boundTopic.replace(":device_alias", "__unnamed_group");
  }

  return boundTopic;
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
    json[GroupStateFieldNames::STATUS] = connectionStatus;

    // Fill other fields
    AboutHelper::generateAboutObject(json, true);

    String response;
    serializeJson(json, response);

    return response;
  }
}