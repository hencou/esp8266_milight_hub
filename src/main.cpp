#ifndef UNIT_TEST

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
//#include <MiLightUdpServer.h> //not needed save memory causes crashes
#include <ESP8266mDNS.h>
//#include <ESP8266SSDP.h> //not needed save memory causes crashes
#include <MqttClient.h>
#include <RGBConverter.h>
//#include <MiLightDiscoveryServer.h> //not needed save memory causes crashes
#include <MiLightClient.h>
#include <BulbStateUpdater.h>
//#include <LEDStatus.h> //not needed save memory causes crashes
//<Added by HC>
#include <WallSwitch.h>
#include <credentials.h>
#include <ESP8266WiFi.h> //we want complete non blocking wifi from the beginning or even with no wifi
//</Added by HC>

#include <vector>
#include <memory>

//<Added by HC>
const char*  wifi_ssid    = WIFI_SSID;
const char*  wifi_password    = WIFI_PASSWORD;
//</Added by HC>

Settings settings;

//<Added by HC>
WallSwitch* wallSwitch = NULL;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
boolean wifiConnected = false;
boolean oldWifiConnected = false;
long wifiOfflineTime = 0;
//</Added by HC>

MiLightClient* milightClient = NULL;
std::shared_ptr<MiLightRadioFactory> radioFactory;
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
  StaticJsonDocument<200> buffer;
  JsonObject result = buffer.to<JsonObject>();

  BulbId bulbId = config.packetFormatter->parsePacket(packet, result);

  if (&bulbId == &DEFAULT_BULB_ID) {
    Serial.println(F("Skipping packet handler because packet was not decoded"));
    return;
  }

  const MiLightRemoteConfig& remoteConfig =
    *MiLightRemoteConfig::fromType(bulbId.deviceType);

  //update the status of groups for the registered UDP devices
  for (size_t i = 0; i < settings.gatewayConfigs.size(); i++) {
    if (bulbId.deviceId == settings.gatewayConfigs[i]->deviceId) {

  		#ifdef DEBUG_PRINTF
  		Serial.println(F("onPacketSentHandler - update the status of groups for the first UDP device\r\n"));
  		#endif

      	// update state to reflect changes from this packet
  		GroupState* groupState = stateStore->get(bulbId);

		// pass in previous scratch state as well
  		const GroupState stateUpdates(groupState, result);

	    if (groupState != NULL) {
	      groupState->patch(stateUpdates);

	      // Copy state before setting it to avoid group 0 re-initialization clobbering it
    	  stateStore->set(bulbId, stateUpdates);
	    }

  		if (mqttClient) {

  			// Sends the state delta derived from the raw packet
  			char output[200];
  			serializeJson(result, output);
  			mqttClient->sendUpdate(remoteConfig, bulbId.deviceId, bulbId.groupId, output);

  			// Sends the entire state
    		if (groupState != NULL) {
      		bulbStateUpdater->enqueueUpdate(bulbId, *groupState);
  		  }
  	  }
    }
  }

  httpServer->handlePacketSent(packet, remoteConfig);
}

/**
 * Listen for packets on one radio config.  Cycles through all configs as its
 * called.
 */
void handleListen() {
  if (! settings.listenRepeats) {
    return;
  }

  std::shared_ptr<MiLightRadio> radio = milightClient->switchRadio(currentRadioType++ % milightClient->getNumRadios());

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

      // update state to reflect this packet
      onPacketSentHandler(readPacket, *remoteConfig);
    }
  }
}

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

  radioFactory = MiLightRadioFactory::fromSettings(settings);

  if (radioFactory == NULL) {
    Serial.println(F("ERROR: unable to construct radio factory"));
  }

  stateStore = new GroupStateStore(MILIGHT_MAX_STATE_ITEMS, settings.stateFlushInterval);

  milightClient = new MiLightClient(
    radioFactory,
    stateStore,
    &settings
  );
  milightClient->begin();
  milightClient->onPacketSent(onPacketSentHandler);
  milightClient->onUpdateBegin(onUpdateBegin);
  milightClient->onUpdateEnd(onUpdateEnd);
  milightClient->setResendCount(settings.packetRepeats);

  if (settings.mqttServer().length() > 0) {
    mqttClient = new MqttClient(settings, milightClient);
    mqttClient->begin();
    bulbStateUpdater = new BulbStateUpdater(settings, *mqttClient, *stateStore);
  }

  //<added by HC>
  WiFi.hostname(settings.hostname);
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    Serial.print("Station connected, IP: ");
    Serial.println(WiFi.localIP());
  });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    Serial.println("Station disconnected");
  });

  Serial.printf("Connecting to %s ...\n", wifi_ssid);
  WiFi.begin(wifi_ssid, wifi_password);
  
  wallSwitch = new WallSwitch(settings, milightClient, stateStore, *mqttClient);
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

// Called when a group is deleted via the REST API.  Will publish an empty message to
// the MQTT topic to delete retained state
void onGroupDeleted(const BulbId& id) {
  if (mqttClient != NULL) {
    mqttClient->sendState(
      *MiLightRemoteConfig::fromType(id.deviceType),
      id.deviceId,
      id.groupId,
      ""
    );
  }
}

void setup() {
  Serial.begin(9600);

  // load up our persistent settings from the file system
  SPIFFS.begin();
  Settings::load(settings);
  applySettings();

  // set up the LED status for wifi configuration
  //ledStatus = new LEDStatus(settings.ledPin);
  //ledStatus->continuous(settings.ledModeWifiConfig);

  // start up the wifi manager
  if (! MDNS.begin("milight-hub")) {
    Serial.println(F("Error setting up MDNS responder"));
  }

  MDNS.addService("http", "tcp", 80);

  //  SSDP.setSchemaURL("description.xml");
  //  SSDP.setHTTPPort(80);
  //  SSDP.setName("ESP8266 MiLight Gateway");
  //  SSDP.setSerialNumber(ESP.getChipId());
  //  SSDP.setURL("/");
  //  SSDP.setDeviceType("upnp:rootdevice");
  //  SSDP.begin();

  httpServer = new MiLightHttpServer(settings, milightClient, stateStore);
  httpServer->onSettingsSaved(applySettings);
  httpServer->onGroupDeleted(onGroupDeleted);
  //httpServer->on("/description.xml", HTTP_GET, []() { SSDP.schema(httpServer->client()); });
  httpServer->begin();

  Serial.printf_P(PSTR("Setup complete (version %s)\n"), QUOTE(MILIGHT_HUB_VERSION));
}

void loop() {
  httpServer->handleClient();

  if (mqttClient) {
    mqttClient->handleClient();
    bulbStateUpdater->loop();
  }

  handleListen();

  stateStore->limitedFlush();

  if (shouldRestart()) {
    Serial.println(F("Auto-restart triggered. Restarting..."));
    ESP.restart();
  }

  //<Addd by HC>
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
  }
  
  if (oldWifiConnected != wifiConnected) {
    wifiOfflineTime = millis();
  }
  oldWifiConnected = wifiConnected;

  if ((WiFi.status() != WL_CONNECTED) && (millis() - wifiOfflineTime > 300000)) {
    wifiConnected = false;
  }

  wallSwitch->loop(!wifiConnected);
  //</Added by HC>
}

#endif