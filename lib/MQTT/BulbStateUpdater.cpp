#include <BulbStateUpdater.h>

BulbStateUpdater::BulbStateUpdater(Settings& settings, MqttClient& mqttClient, GroupStateStore& stateStore)
  : settings(settings),
    mqttClient(mqttClient),
    stateStore(stateStore),
    lastFlush(0),
    enabled(true)
{ }

void BulbStateUpdater::enable() {
  //<Added by HC>: wait a little before sending final state to avoid double messages
  lastFlush = millis();
  //</Added by HC>
  this->enabled = true;
}

void BulbStateUpdater::disable() {
  this->enabled = false;
}

void BulbStateUpdater::enqueueUpdate(BulbId bulbId, GroupState& groupState) {
  // If can flush immediately, do so (avoids lookup of group state later).
  if (canFlush()) {
    flushGroup(bulbId, groupState);
  } else {
    staleGroups.push(bulbId);
  }
}

void BulbStateUpdater::loop() {
  while (canFlush() && staleGroups.size() > 0) {
    BulbId bulbId = staleGroups.shift();
    GroupState* groupState = stateStore.get(bulbId);

    if (groupState->isMqttDirty()) {
      flushGroup(bulbId, *groupState);
      groupState->clearMqttDirty();
    }
  }
}

inline void BulbStateUpdater::flushGroup(BulbId bulbId, GroupState& state) {
  char buffer[200];
  StaticJsonDocument<200> json;
  JsonObject message = json.to<JsonObject>();

  state.applyState(message, bulbId, settings.groupStateFields);
  serializeJson(json, buffer);

  //<Added by HC, send night mode state>
  if (state.isNightMode()) {
    mqttClient.sendState(*MiLightRemoteConfig::fromType(bulbId.deviceType),
                          bulbId.deviceId,
                          bulbId.groupId,
                          "{\"state\": \"ON\", \"effect\": \"night_mode\"}"
                          );
    return;
  }
  //</Added by HC>

  mqttClient.sendState(
    *MiLightRemoteConfig::fromType(bulbId.deviceType),
    bulbId.deviceId,
    bulbId.groupId,
    buffer
  );

  lastFlush = millis();
}

inline bool BulbStateUpdater::canFlush() const {
  return enabled && (millis() > (lastFlush + settings.mqttStateRateLimit));
}
