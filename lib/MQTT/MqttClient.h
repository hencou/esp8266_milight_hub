#include <MiLightClient.h>
#include <Settings.h>
#include <MiLightRadioConfig.h>

#ifndef MQTT_CONNECTION_ATTEMPT_FREQUENCY
#define MQTT_CONNECTION_ATTEMPT_FREQUENCY 5000
#endif

#ifndef _MQTT_CLIENT_H
#define _MQTT_CLIENT_H

class MqttClient {
public:
  MqttClient(Settings& settings, MiLightClient*& milightClient, const char* outTopic);
  ~MqttClient();

  //<added by HC>
  std::function<void(const char *topic, const char *msg, const bool retain)> callback;
  void setCallback(std::function<void(const char *topic, const char *msg, const bool retain)> _callback);
  void fromMeshCallback(const char *topic, const char *msg);
  //</added by HC>

  void sendUpdate(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update);
  void sendState(const MiLightRemoteConfig& remoteConfig, uint16_t deviceId, uint16_t groupId, const char* update);

private:
  MiLightClient*& milightClient;
  Settings& settings;
  const char* outTopic;
  unsigned long lastConnectAttempt;
  void sendBirthMessage();
  void publish(
    const String& topic,
    const MiLightRemoteConfig& remoteConfig,
    uint16_t deviceId,
    uint16_t groupId,
    const char* update,
    const bool retain = false
  );

  inline static void bindTopicString(
    String& topicPattern,
    const MiLightRemoteConfig& remoteConfig,
    const uint16_t deviceId,
    const uint16_t groupId
  );

  String generateConnectionStatusMessage(const char* status);
};

#endif
