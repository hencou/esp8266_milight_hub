#include <functional>
#include <stdio.h>
#include <Arduino.h>
#include <WallSwitch.h>
#include <MiLightRadioConfig.h>
#include <Settings.h>
#include <Units.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266Wifi.h>

WallSwitch::WallSwitch(Settings& settings, MiLightClient*& milightClient, GroupStateStore*& stateStore, MqttClient& mqttClient)
  : milightClient(milightClient),
    settings(settings),
    stateStore(stateStore),
    mqttClient(mqttClient),
    oneWire(ONE_WIRE_BUS),
    sensors(&oneWire)
{
}

WallSwitch::~WallSwitch() {
}

void WallSwitch::begin() {

  //begin DS18B20 sensor
  sensors.setResolution(12);
  sensors.begin();

  if (settings.gatewayConfigs.size() > 0) {

    remoteConfig = MiLightRemoteConfig::fromType("rgb_cct");
    // Set button input pins
    for (uint8_t i = 0; i < 3; i++) {
      pinMode(buttonPins[i], INPUT_PULLUP);
    }

    //set ADC port to input for LDR
    pinMode(A0, INPUT_PULLUP);

    delayTimer = millis();
    startupTimer = random(15000, 30000);
  }
}

void WallSwitch::loop(bool standAloneAP) {

//Get button event and act accordingly when UDP gateway configured
  if (settings.gatewayConfigs.size() > 0) {

    //Startup with all lamps off
    doLightState();

    for (uint8_t i = 0; i < 3; i++) {
      checkButton(buttonPins[i], i);
    }
    //Switch lamps on LDR state, only in AP mode
    if (standAloneAP) {
      doDayNight();
    } else {
      detectMotion();
    }
  }

  //Publish heap and temperature
  if (millis() - lastMsg > 300000) {
    lastMsg = millis();
    char topic[128];
    char msg[64];
    snprintf(topic, sizeof(topic), "state/%s/heap", WiFi.macAddress().c_str());
    snprintf(msg, sizeof(msg), "%d", ESP.getFreeHeap());
    mqttClient.send(topic, msg, false);

    //send the DS18B20 temperature if available
    int deviceCount = sensors.getDeviceCount();
    //Serial.print("deviceCount:");
    //Serial.println(String(deviceCount).c_str());

    sensors.requestTemperatures();
    if (deviceCount > 0) {
      snprintf(topic, sizeof(topic), "state/%s/temperature", WiFi.macAddress().c_str());
      dtostrf(sensors.getTempCByIndex(0),5, 2, msg);
      mqttClient.send(topic, msg, false);
    }
  }
}

//handle button press
void WallSwitch::checkButton(int buttonPin, uint8_t id) {

  currentState[id] = digitalRead(buttonPin);

  if (currentState[id] == LOW && previousState[id] == HIGH && (millis() - firstTime[id]) > 100) {
    firstTime[id] = millis();
    buttonDirty[id] = true;
    initLongClick[id] = true;
  }

  if (currentState[id] == LOW) {
    millis_held[id] = (millis() - firstTime[id]);
  }

  //reset clicks after long press and toggle raising states
  if (millis_held[id] > 500 && currentState[id] == HIGH && previousState[id] == LOW) {
    if (shortClicks[id] == 0) {raisingBrightness[id] = !raisingBrightness[id];}
    if (shortClicks[id] == 1) {raisingTemperature[id] = !raisingTemperature[id];}

    shortClicks[id] = 0;
  }

  if (millis() > timePressLimit[id] && currentState[id] == HIGH && shortClicks[id] != 0) {
    doShortClicks(id);
    shortClicks[id] = 0;
  }

  if (millis_held[id] > 50)
  {
    if (currentState[id] == HIGH && previousState[id] == LOW && millis_held[id] <= 500)
    {
      //count short clicks
      shortClicks[id]++;
      timePressLimit[id] = firstTime[id] + 1000;
    }

    if (millis_held[id] > 500 && currentState[id] == LOW)
    {
      doLongClicks(id);
    }
  }

  if (buttonDirty[id] == true && currentState[id] == HIGH && (millis() - firstTime[id]) > 1000)
  {
    sendMQTTCommand(id);
    buttonDirty[id] = false;
  }

  previousState[id] = currentState[id];
}

//handle short clicks
void WallSwitch::doShortClicks(uint8_t id)
{
  milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id + 1);

  if (shortClicks[id] == 1)
  {
    milightClient->updateStatus(OFF);
    milightClient->updateStatus(OFF);
  }
  if (shortClicks[id] == 2)
  {
    milightClient->enableNightMode();
    milightClient->enableNightMode();

    //no Off command to mesh_in:
    buttonDirty[id] = false;
  }
  if (shortClicks[id] == 3)
  {
    milightClient->updateColorWhite();
    milightClient->updateColorWhite();
  }
  if (shortClicks[id] == 4)
  {
    milightClient->pair();
  }
  if (shortClicks[id] == 5)
  {
    milightClient->unpair();
  }
}

//handle long clicks
void WallSwitch::doLongClicks(uint8_t id)
{
  BulbId bulbId(settings.gatewayConfigs[0]->deviceId, id + 1, remoteConfig->type);
  milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id + 1);

  //set lamps on before raising brightness and initialize raising state
  if (initLongClick[id])
  {
   
    uint8_t brightness = stateStore->get(bulbId)->getBrightness();
    if (brightness > 90) {raisingBrightness[id] = false;}
    if (brightness < 10) {raisingBrightness[id] = true;}

    uint16_t temperature = stateStore->get(bulbId)->getMireds();
    if (temperature > COLOR_TEMP_MAX_MIREDS - 10) {raisingTemperature[id] = false;}
    if (temperature < COLOR_TEMP_MIN_MIREDS + 10) {raisingTemperature[id] = true;}

    initLongClick[id] = false;
  }

  if (millis() - millis_repeat[id] < 100) {
    return;
  }
  millis_repeat[id] = millis();

  //send always an ON command to ensure the lamp is on
  milightClient->updateStatus(ON);

  //Brightness
  if (shortClicks[id] == 0)
  {
    uint8_t brightness = stateStore->get(bulbId)->getBrightness();
    if (brightness < 100 && raisingBrightness[id] == true)
    {
      brightness += 10;
      milightClient->updateBrightness(brightness);
    }
    if (brightness > 0 && raisingBrightness[id] == false)
    {
      brightness -= 10;
      milightClient->updateBrightness(brightness);
    }
  }

  //Temperature
  if (shortClicks[id] == 1)
  {
    uint16_t temperature = stateStore->get(bulbId)->getMireds();
    if (temperature < COLOR_TEMP_MAX_MIREDS && raisingTemperature[id] == true)
    {
      temperature += 10;
      milightClient->updateTemperature(Units::miredsToWhiteVal(temperature, 100));
    }
    if (temperature > COLOR_TEMP_MIN_MIREDS && raisingTemperature[id] == false)
    {
      temperature -= 10;
      milightClient->updateTemperature(Units::miredsToWhiteVal(temperature, 100));
    }
  }

  //Reset after 10 seconds long press
  if (millis_held[id] > 10000)
  {
    milightClient->updateStatus(OFF);
    ESP.restart();
  }
}

//Send command update to MQTT mesh_in/milight to update other devices with same bulbId
void WallSwitch::sendMQTTCommand(uint8_t id)
{
  BulbId bulbId(settings.gatewayConfigs[0]->deviceId, id + 1, remoteConfig->type);

  char buffer[200];
  StaticJsonDocument<200> json;
  JsonObject message = json.to<JsonObject>();

  GroupState* groupState = stateStore->get(bulbId);
	if (groupState == NULL) return;

  groupState->applyState(message, bulbId, settings.groupStateFields);
  serializeJson(json, buffer);

  String topic = settings.mqttTopicPattern;
  String hexDeviceId = bulbId.getHexDeviceId();

  topic.replace(":device_id", hexDeviceId);
  topic.replace(":hex_device_id", hexDeviceId);
  topic.replace(":dec_device_id", String(bulbId.deviceId));
  topic.replace(":device_type", MiLightRemoteTypeHelpers::remoteTypeToString(bulbId.deviceType));
  topic.replace(":group_id", String(bulbId.groupId));

  #ifdef MQTT_DEBUG
    Serial.printf("WallSwitch - send message to topic: %s : %s\r\n", topic.c_str(), output);
  #endif

  //send command to milight/xxx/xxx/x
  mqttClient.send(topic.c_str(), buffer, false);
}

//handle actions based on LDR state
void WallSwitch::doDayNight()
{
  //detect night with LDR on A0 pin
  if (lastAnalogRead + 1000 < millis())
  {
    lastAnalogRead = millis();
    darknessLevel = analogRead(A0);

    #ifdef DEBUG_PRINTF
      Serial.printf("darknessLevel: %s\r\n", darknessLevel);
    #endif
  }

  if (darknessLevel > 350) { //0-1024, 1024 is dark, 0-1.024V, LDR between ADC and GND, and pullup resistor

    if (previousDelay == false && isNight == false) {
      delayTimer = millis();
      previousDelay = true;
    }

    if (delayTimer + 60000 < millis() && isNight == false && previousDelay == true) {
      isNight = true;
      previousDelay = false;
    }
    nightTimer = (millis()/1000) - nightTime;
  } else if (darknessLevel < 250) {
    if (previousDelay == false && isNight == true) {
      delayTimer = millis();
      previousDelay = true;
    }
    if (delayTimer + 60000 < millis() && isNight == true && previousDelay == true) {
      isNight = false;
      isMidNight = false;
      previousDelay = false;
    }
    nightTimer = 0;
  }

  if (previousNight == false && isNight == true){
    Serial.println(F("LDR triggered night condition, turn lamps on..."));
    nightTime = millis()/1000;

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 1); //zithoek
    milightClient->updateStatus(ON);
    milightClient->updateTemperature(100);
    milightClient->updateBrightness(1);

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 2); //keuken
    milightClient->enableNightMode();

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 3); //kinderhoek
    milightClient->enableNightMode();

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 4); //buitenverlichting
    milightClient->updateStatus(ON);
    milightClient->updateTemperature(100);
    milightClient->updateBrightness(50);
  }

  if (nightTimer > 10800 && isMidNight == false) { //after 3 hours turn lamps on night mode
    Serial.println(F("Nightmode timeout, turn light in nightmode"));
    isMidNight = true;

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 4); //outdoor light
    milightClient->enableNightMode();
  }

  if (previousNight == true && isNight == false){
    Serial.println(F("LDR triggered day condition, turn lamps off..."));

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 0); //all lamps off
    milightClient->updateStatus(OFF);
  }

  previousNight = isNight;
}

void WallSwitch::detectMotion() {

  if (lastAnalogRead + 500 < millis())
  {
    lastAnalogRead = millis();
    motionLevel = analogRead(A0);
  }

  boolean motion = (motionLevel > 1000) ?  true : false;

  if (motion == true && previousMotion == false) {
    firstMotion = millis();
  }

  if (motion == true && motionState == false && (millis() - firstMotion) > 1000) {
    motionState = true;

    char topic[128];    
    snprintf(topic, sizeof(topic), "state/%s/motion", WiFi.macAddress().c_str());
    mqttClient.send(topic, "ON", false);
  }

  if (motion == false && motionState == true) {
    motionState = false;

    char topic[128];
    snprintf(topic, sizeof(topic), "state/%s/motion", WiFi.macAddress().c_str());
    mqttClient.send(topic, "OFF", false);
  }
  previousMotion = motion;
}

//Non blocking startup items
void WallSwitch::doLightState() {

  //turn all lights off at startup
  if (isStartUp == true && millis() > startupTimer) {

    isStartUp = false;

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 0);
    milightClient->updateTemperature(100);
    milightClient->updateStatus(OFF);

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 0);
    milightClient->updateStatus(OFF);
  }
}
