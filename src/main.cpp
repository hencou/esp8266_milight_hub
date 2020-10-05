#ifndef UNIT_TEST

#include <SPI.h>
//#include <WiFiManager.h>
#include <ArduinoJson.h>
#include <stdlib.h>
#include <FS.h>
#include <IntParsing.h>
#include <Size.h>
#include <LinkedList.h>
//#include <LEDStatus.h>
#include <GroupStateStore.h>
#include <MiLightRadioConfig.h>
#include <MiLightRemoteConfig.h>
#include <MiLightHttpServer.h>
#include <MiLightRemoteType.h>
#include <Settings.h>
//#include <MiLightUdpServer.h>
#include <ESP8266mDNS.h>
#include <ESP8266SSDP.h>
#include <MqttClient.h>
#include <RGBConverter.h>
//#include <MiLightDiscoveryServer.h>
#include <MiLightClient.h>
#include <BulbStateUpdater.h>
#include <RadioSwitchboard.h>
#include <PacketSender.h>
//#include <HomeAssistantDiscoveryClient.h>
#include <TransitionController.h>
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
// static LEDStatus *ledStatus;

Settings settings;

//<Added by HC>
WallSwitch* wallSwitch = NULL;

WiFiEventHandler gotIpEventHandler, disconnectedEventHandler;
boolean wifiConnected = false;
boolean oldWifiConnected = false;
long wifiOfflineTime = 0;
//</Added by HC>

MiLightClient* milightClient = NULL;
RadioSwitchboard* radios = nullptr;
PacketSender* packetSender = nullptr;
std::shared_ptr<MiLightRadioFactory> radioFactory;
MiLightHttpServer *httpServer = NULL;
MqttClient* mqttClient = NULL;
//MiLightDiscoveryServer* discoveryServer = NULL;
uint8_t currentRadioType = 0;

// For tracking and managing group state
GroupStateStore* stateStore = NULL;
BulbStateUpdater* bulbStateUpdater = NULL;
TransitionController transitions;

// int numUdpServers = 0;
// std::vector<std::shared_ptr<MiLightUdpServer>> udpServers;
// WiFiUDP udpSeder;

// /**
//  * Set up UDP servers (both v5 and v6).  Clean up old ones if necessary.
//  */
// void initMilightUdpServers() {
//   udpServers.clear();

//   for (size_t i = 0; i < settings.gatewayConfigs.size(); ++i) {
//     const GatewayConfig& config = *settings.gatewayConfigs[i];

//     std::shared_ptr<MiLightUdpServer> server = MiLightUdpServer::fromVersion(
//       config.protocolVersion,
//       milightClient,
//       config.port,
//       config.deviceId
//     );

//     if (server == NULL) {
//       Serial.print(F("Error creating UDP server with protocol version: "));
//       Serial.println(config.protocolVersion);
//     } else {
//       udpServers.push_back(std::move(server));
//       udpServers[i]->begin();
//     }
//   }
// }

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

  // set LED mode for a packet movement
  // ledStatus->oneshot(settings.ledModePacket, settings.ledModePacketCount);

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
  // Do not handle listens while there are packets enqueued to be sent
  // Doing so causes the radio module to need to be reinitialized inbetween
  // repeats, which slows things down.
  if (! settings.listenRepeats || packetSender->isSending()) {
    return;
  }

  std::shared_ptr<MiLightRadio> radio = radios->switchRadio(currentRadioType++ % radios->getNumRadios());

  for (size_t i = 0; i < settings.listenRepeats; i++) {
    if (radios->available()) {
      uint8_t readPacket[MILIGHT_MAX_PACKET_LENGTH];
      size_t packetLen = radios->read(readPacket);

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
  if (packetSender) {
    delete packetSender;
  }
  if (radios) {
    delete radios;
  }

  transitions.setDefaultPeriod(settings.defaultTransitionPeriod);
  if (wallSwitch) {
    delete wallSwitch;
    wallSwitch = NULL;
  }

  radioFactory = MiLightRadioFactory::fromSettings(settings);

  if (radioFactory == NULL) {
    Serial.println(F("ERROR: unable to construct radio factory"));
  }

  stateStore = new GroupStateStore(MILIGHT_MAX_STATE_ITEMS, settings.stateFlushInterval);

  radios = new RadioSwitchboard(radioFactory, stateStore, settings);
  packetSender = new PacketSender(*radios, settings, onPacketSentHandler);

  milightClient = new MiLightClient(
    *radios,
    *packetSender,
    stateStore,
    settings,
    transitions
  );
  milightClient->onUpdateBegin(onUpdateBegin);
  milightClient->onUpdateEnd(onUpdateEnd);

  if (settings.mqttServer().length() > 0) {
    mqttClient = new MqttClient(settings, milightClient);
    mqttClient->begin();
    mqttClient->onConnect([]() {
      if (settings.homeAssistantDiscoveryPrefix.length() > 0) {
      //  HomeAssistantDiscoveryClient discoveryClient(settings, mqttClient);
      //  discoveryClient.sendDiscoverableDevices(settings.groupIdAliases);
      //  discoveryClient.removeOldDevices(settings.deletedGroupIdAliases);

        settings.deletedGroupIdAliases.clear();
      }
    });

    bulbStateUpdater = new BulbStateUpdater(settings, *mqttClient, *stateStore);
  }

  // initMilightUdpServers();

  // if (discoveryServer) {
  //   delete discoveryServer;
  //   discoveryServer = NULL;
  // }
  // if (settings.discoveryPort != 0) {
  //   discoveryServer = new MiLightDiscoveryServer(settings);
  //   discoveryServer->begin();
  // }

  // // update LED pin and operating mode
  // if (ledStatus) {
  //   ledStatus->changePin(settings.ledPin);
  //   ledStatus->continuous(settings.ledModeOperating);
  // }

  WiFi.hostname(settings.hostname);

  WiFiPhyMode_t wifiMode;
  switch (settings.wifiMode) {
    case WifiMode::B:
      wifiMode = WIFI_PHY_MODE_11B;
      break;
    case WifiMode::G:
      wifiMode = WIFI_PHY_MODE_11G;
      break;
    default:
    case WifiMode::N:
      wifiMode = WIFI_PHY_MODE_11N;
      break;
  }
  WiFi.setPhyMode(wifiMode);
  
  //<added by HC>
  gotIpEventHandler = WiFi.onStationModeGotIP([](const WiFiEventStationModeGotIP& event)
  {
    Serial.print("Station connected, IP: ");
    Serial.println(WiFi.localIP());

    if (settings.hostname.length() > 0) {
      if (! MDNS.begin(settings.hostname.c_str())) {
      Serial.println(F("Error setting up MDNS responder"));
      }
    } else {
      if (! MDNS.begin("milight-hub")) {
      Serial.println(F("Error setting up MDNS responder"));
      }
    }
    
    MDNS.addService("http", "tcp", 80);

    SSDP.setSchemaURL("description.xml");
    SSDP.setHTTPPort(80);
    SSDP.setName("ESP8266 MiLight Gateway");
    SSDP.setSerialNumber(ESP.getChipId());
    SSDP.setURL("/");
    SSDP.setDeviceType("upnp:rootdevice");
    SSDP.begin();
    });

  disconnectedEventHandler = WiFi.onStationModeDisconnected([](const WiFiEventStationModeDisconnected& event)
  {
    Serial.println("Station disconnected");
  });

  Serial.printf("Connecting to %s ...\n", wifi_ssid);
  WiFi.mode(WIFI_STA);
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

// // give a bit of time to update the status LED
// void handleLED() {
//   ledStatus->handle();
// }

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

  // // set up the LED status for wifi configuration
  // ledStatus = new LEDStatus(settings.ledPin);
  // ledStatus->continuous(settings.ledModeWifiConfig);

  httpServer = new MiLightHttpServer(settings, milightClient, stateStore, packetSender, radios, transitions);
  httpServer->onSettingsSaved(applySettings);
  httpServer->onGroupDeleted(onGroupDeleted);
  httpServer->on("/description.xml", HTTP_GET, []() { SSDP.schema(httpServer->client()); });
  httpServer->begin();

  transitions.addListener(
    [](const BulbId& bulbId, GroupStateField field, uint16_t value) {
      StaticJsonDocument<100> buffer;

      const char* fieldName = GroupStateFieldHelpers::getFieldName(field);
      buffer[fieldName] = value;

      milightClient->prepare(bulbId.deviceType, bulbId.deviceId, bulbId.groupId);
      milightClient->update(buffer.as<JsonObject>());
    }
  );

  Serial.printf_P(PSTR("Setup complete (version %s)\n"), QUOTE(MILIGHT_HUB_VERSION));
}

void loop() {
  httpServer->handleClient();
  MDNS.update();

  if (mqttClient) {
    mqttClient->handleClient();
    bulbStateUpdater->loop();
  }

  // for (size_t i = 0; i < udpServers.size(); i++) {
  //   udpServers[i]->handleClient();
  // }

  // if (discoveryServer) {
  //   discoveryServer->handleClient();
  // }

  handleListen();

  stateStore->limitedFlush();
  packetSender->loop();

  // update LED with status
  // ledStatus->handle();

  transitions.loop();

  if (shouldRestart()) {
    Serial.println(F("Auto-restart triggered. Restarting..."));
    ESP.restart();
  }

  //<Addd by HC>
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
  }
  
  if ((oldWifiConnected != wifiConnected) && wifiConnected == false) {
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