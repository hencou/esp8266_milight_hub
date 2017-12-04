#include <SPI.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <FS.h>
#include <IntParsing.h>
#include <Size.h>
#include <LinkedList.h>
#include <GroupStateStore.h>
#include <MiLightRadioConfig.h>
#include <MiLightRemoteConfig.h>
#include <MiLightHttpServer.h>
#include <Settings.h>
#include <RGBConverter.h>
#include <MiLightClient.h>
#include <BulbStateUpdater.h>
#include <ESP8266MQTTmesh.h>
#include <WallSwitch.h>
#include <credentials.h>

Settings settings;

//<Added by HC>
const char*  inTopic          = "mesh_in/";
const char*  outTopic         = "mesh_out/";
const char*  networks[]       = NETWORK_LIST;
const char*  network_password = NETWORK_PASSWORD;
const char*  mesh_password    = MESH_PASSWORD;
const char*  base_ssid        = "MESH-";
int mesh_port                 = 1884;
#if ASYNC_TCP_SSL_ENABLED
const uint8_t *mqtt_fingerprint = MQTT_FINGERPRINT;
bool         mqtt_secure      = MQTT_SECURE;
bool         mesh_secure      = MESH_SECURE;
#endif

char mqtt_server[20];
int mqtt_port;

ESP8266MQTTMesh *mesh = NULL;
WallSwitch* wallSwitch = NULL;
//</Added by HC>

MiLightClient* milightClient = NULL;
MiLightRadioFactory* radioFactory = NULL;
MiLightHttpServer *httpServer = NULL;
MqttClient* mqttClient = NULL;
uint8_t currentRadioType = 0;

// For tracking and managing group state
GroupStateStore* stateStore = NULL;
BulbStateUpdater* bulbStateUpdater = NULL;

/**
 * Milight RF packet handler.
 *
 * Called both when a packet is sent locally, and when an intercepted packet
 * is read.
 */
void onPacketSentHandler(uint8_t* packet, const MiLightRemoteConfig& config) {
  StaticJsonBuffer<200> buffer;
  JsonObject& result = buffer.createObject();
  BulbId bulbId = config.packetFormatter->parsePacket(packet, result, stateStore);

  if (&bulbId == &DEFAULT_BULB_ID) {
    Serial.println(F("Skipping packet handler because packet was not decoded"));
    return;
  }

  const MiLightRemoteConfig& remoteConfig = *MiLightRemoteConfig::fromType(bulbId.deviceType);

  //update the status of groups for the first UDP device
  if (settings.numGatewayConfigs > 0) {
    if (bulbId.deviceId == settings.gatewayConfigs[0]->deviceId) {

  		#ifdef DEBUG_PRINTF
  		Serial.println(F("onPacketSentHandler - update the status of groups for the first UDP device\r\n"));
  		#endif

      GroupState& groupState = stateStore->get(bulbId);
      groupState.patch(result);
      stateStore->set(bulbId, groupState);

  		if (mqttClient) {

  			// Sends the state delta derived from the raw packet
  			char output[200];
  			result.printTo(output);
  			mqttClient->sendUpdate(remoteConfig, bulbId.deviceId, bulbId.groupId, output);

  			// Sends the entire state
  			bulbStateUpdater->enqueueUpdate(bulbId, groupState);
  		}
  	}
  }

  httpServer->handlePacketSent(packet, remoteConfig);
}

/**Added by HC
 * Callback for the mqttClient to send mqtt publishes to the mesh mqtt
 */
void mqttToMesh(const char *topic, const char *msg, const bool retain) {
  if (settings.mqttServer().length() > 0) {
    mesh->publish(topic, msg, retain);
  }
}

/**
 * Listen for packets on one radio config.  Cycles through all configs as its
 * called.
 */
void handleListen() {
  if (! settings.listenRepeats) {
    return;
  }

  MiLightRadio* radio = milightClient->switchRadio(currentRadioType++ % milightClient->getNumRadios());

  for (size_t i = 0; i < settings.listenRepeats; i++) {
    if (milightClient->available()) {
      uint8_t readPacket[MILIGHT_MAX_PACKET_LENGTH];
      size_t packetLen = milightClient->read(readPacket);

      const MiLightRemoteConfig* remoteConfig = MiLightRemoteConfig::fromReceivedPacket(
        radio->config(),
        readPacket,
        packetLen
      );

      if (remoteConfig == NULL) {
        // This can happen under normal circumstances, so not an error condition
#ifdef DEBUG_PRINTF
        Serial.println(F("WARNING: Couldn't find remote for received packet"));
#endif
        return;
      }
      onPacketSentHandler(readPacket, *remoteConfig);
    }
  }
}

//<added by HC>
void fromMeshCallback(const char *topic, const char *msg) {

  //only accept milight topics
  const char *subtopic = topic + strlen(inTopic);
  if (!strstr(subtopic, "milight")) {
    return;
  }
  if (mqttClient) {
    mqttClient->fromMeshCallback(subtopic, msg);
  }
}
//</added by HC>

/**
 * Called when MqttClient#update is first being processed.  Stop sending updates
 * and aggregate state changes until the update is finished.
 */
void onUpdateBegin() {
  if (bulbStateUpdater) {
    bulbStateUpdater->disable();
  }
}

/**
 * Called when MqttClient#update is finished processing.  Re-enable state
 * updates, which will flush accumulated state changes.
 */
void onUpdateEnd() {
  if (bulbStateUpdater) {
    bulbStateUpdater->enable();
  }
}

/**
 * Apply what's in the Settings object.
 */
void applySettings() {
  if (milightClient) {
    delete milightClient;
  }
  if (radioFactory) {
    delete radioFactory;
  }
  if (mqttClient) {
    delete mqttClient;
    delete bulbStateUpdater;

    mqttClient = NULL;
    bulbStateUpdater = NULL;
  }
  if (stateStore) {
    delete stateStore;
  }
  if (wallSwitch) {
    delete wallSwitch;
    wallSwitch = NULL;
  }
   if (mesh) {
    //delete mesh;
    //mesh = NULL;
    delay(100);
    ESP.restart();
  }

  strlcpy(mqtt_server, settings.mqttServer().c_str(), sizeof(mqtt_server));
  mqtt_port = settings.mqttPort();
  mesh = ESP8266MQTTMesh::Builder(networks, network_password, mqtt_server, mqtt_port)
        .setMeshPassword(mesh_password)
        .setMeshPort(mesh_port)
        .setBaseSSID(base_ssid)
        .setTopic(inTopic, outTopic)
        .buildptr();

  mesh->setCallback(fromMeshCallback);
  mesh->begin();

  radioFactory = MiLightRadioFactory::fromSettings(settings);

  if (radioFactory == NULL) {
    Serial.println(F("ERROR: unable to construct radio factory"));
  }

  stateStore = new GroupStateStore(MILIGHT_MAX_STATE_ITEMS, settings.stateFlushInterval);

  milightClient = new MiLightClient(
    radioFactory,
    *stateStore,
    settings.packetRepeatThrottleThreshold,
    settings.packetRepeatThrottleSensitivity,
    settings.packetRepeatMinimum
  );
  milightClient->begin();
  milightClient->onPacketSent(onPacketSentHandler);
  milightClient->onUpdateBegin(onUpdateBegin);
  milightClient->onUpdateEnd(onUpdateEnd);
  milightClient->setResendCount(settings.packetRepeats);

  if (settings.mqttServer().length() > 0) {
    mqttClient = new MqttClient(settings, milightClient, outTopic);
    //<added by HC>
    mqttClient->setCallback(mqttToMesh);
    //</added by HC>
    bulbStateUpdater = new BulbStateUpdater(settings, *mqttClient, *stateStore);
  }

  //<added by HC>
  wallSwitch = new WallSwitch(settings, milightClient, stateStore, inTopic, outTopic);
  wallSwitch->setCallback(mqttToMesh);
  wallSwitch->begin();
  //</added by HC>
}

/**
 *
 */
bool shouldRestart() {
  if (! settings.isAutoRestartEnabled()) {
    return false;
  }

  return settings.getAutoRestartPeriod()*60*1000 < millis();
}

void setup() {
  Serial.begin(9600);

  SPIFFS.begin();
  Settings::load(settings);
  applySettings();

  httpServer = new MiLightHttpServer(settings, milightClient, stateStore);
  httpServer->onSettingsSaved(applySettings);
  httpServer->begin();

  Serial.println(F("Setup complete"));
}

void loop() {
  httpServer->handleClient();

  if (mqttClient) {
    bulbStateUpdater->loop();
  }

  handleListen();

  stateStore->limitedFlush();

  if (shouldRestart()) {
    Serial.println(F("Auto-restart triggered. Restarting..."));
    ESP.restart();
  }

  //<Addd by HC>
  wallSwitch->loop(mesh->getStandAloneAP());
  //</Added by HC>
}
