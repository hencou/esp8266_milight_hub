#include <functional>
#include <Arduino.h>
#include <WallSwitch.h>
#include <MiLightRadioConfig.h>
#include <Settings.h>
#include <Units.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ESP8266Wifi.h>

WallSwitch::WallSwitch(Settings& settings, MiLightClient*& milightClient, GroupStateStore*& stateStore, const char *inTopic, const char *outTopic)
  : milightClient(milightClient),
    settings(settings),
    stateStore(stateStore),
    inTopic(inTopic),
    outTopic(outTopic),
    oneWire(ONE_WIRE_BUS),
    sensors(&oneWire)
{
}

WallSwitch::~WallSwitch() {
}

void WallSwitch::setCallback(std::function<void(const char *topic, const char *msg, const bool retain)> _callback) {
    callback = _callback;
}

void WallSwitch::begin() {

  //begin DS18B20 sensor
  sensors.setResolution(12);
  sensors.begin();

  if (settings.numGatewayConfigs > 0) {

    remoteConfig = MiLightRemoteConfig::fromType("rgb_cct");
    // Set button input pins
    for (uint8_t i = 0; i < 3; i++) {
      pinMode(buttonPins[i], INPUT_PULLUP);
    }

    //set ADC port to input for LDR
    pinMode(A0, INPUT_PULLUP);

    delayTimer = millis();
  }
}

void WallSwitch::loop(bool standAloneAP) {

//Get button event and act accordingly when UDP gateway configured
  if (settings.numGatewayConfigs > 0) {

    //Startup with all lamps off
    doLightState();

    for (uint8_t i = 0; i < 3; i++) {
      checkButton(buttonPins[i], i);
    }
    //Switch lamps with LDR, only in AP mode
    if (standAloneAP) doDayNight();
  }

  //Publish variables
  if (millis() - lastMsg > 300000) {
    lastMsg = millis();
    char topic[128];
    char msg[64];
    snprintf(topic, sizeof(topic), "%sstate/%s/heap", outTopic, WiFi.macAddress().c_str());
    snprintf(msg, sizeof(msg), "%d", ESP.getFreeHeap());
    callback(topic, msg, false);

    //send the DS18B20 temperature if available
    sensors.requestTemperatures();
    if (sensors.getDeviceCount() > 0) {
      snprintf(topic, sizeof(topic), "%sstate/%s/temperature", outTopic, WiFi.macAddress().c_str());
      dtostrf(sensors.getTempCByIndex(0),5, 2, msg);
      callback(topic, msg, false);
    }
  }
}

//handle button press
void WallSwitch::checkButton(int buttonPin, uint8_t id) {

  currentState[id] = digitalRead(buttonPin);

  if (currentState[id] == LOW && previousState[id] == HIGH && (millis() - firstTime[id]) > 200) {
    firstTime[id] = millis();
    buttonDirty[id] = true;
  }

  if (currentState[id] == LOW) {
    millis_held[id] = (millis() - firstTime[id]);
  }

  //set raising state for long press, reset clicks after long press
  if (millis_held[id] > 500 && currentState[id] == HIGH && previousState[id] == LOW) {
    raisingState[id] = !raisingState[id];
    clicks[id] = 0;
  }

  //Execute corresponding click commands, and reset click counter
  if (millis() > timePressLimit[id] && currentState[id] == HIGH) {

    if (clicks[id] == 1) {
  	  BulbId bulbId(settings.gatewayConfigs[0]->deviceId, id+1, remoteConfig->type);

	    //bool status = stateStore->get(bulbId).getState();

  	  milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);
      //milightClient->updateStatus(status ? ON : OFF);
	    milightClient->updateStatus(OFF);
    }
    if (clicks[id] == 2) {
      milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);
      milightClient->enableNightMode();

      //no Off command to mesh_in:
      buttonDirty[id] = false;
    }
    if (clicks[id] == 3) {
      milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);
      milightClient->updateColorWhite();
    }
    if (clicks[id] == 4) {
      milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);
      milightClient->pair();
    }
    if (clicks[id] == 5) {
      milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);
      milightClient->unpair();
    }
  	if (clicks[id] == 6) {
        ESP.restart();
    }
    clicks[id] = 0;
  }

  if (millis_held[id] > 40) {

    //Short click
    if (currentState[id] == HIGH && previousState[id] == LOW && millis_held[id] <= 500) {

      //count short clicks
      timePressLimit[id] = firstTime[id] + 1000;
      clicks[id]++;
    }

    //Long press
    if (millis_held[id] > 500 && currentState[id] == LOW) {

      BulbId bulbId(settings.gatewayConfigs[0]->deviceId, id+1, remoteConfig->type);
      milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, id+1);

      uint8_t brightness = stateStore->get(bulbId).getBrightness();
      uint16_t temperature = stateStore->get(bulbId).getMireds();
      bool status = stateStore->get(bulbId).getState();

      //set lamps on when off
      if (status == OFF || stateStore->get(bulbId).isNightMode()) {
        raisingState[id] = true;
        milightClient->updateStatus(ON);
      }

      if (millis() - millis_repeat[id] > 100) {
        millis_repeat[id] = millis();

        if (raisingState[id] == true) {
         if (clicks[id] == 1) {
           if (temperature < 370) {
             temperature += 12;
             if (temperature > 370) {temperature = 370;}
             milightClient->updateTemperature(Units::miredsToWhiteVal(temperature, 100));
           }
         } else {
           if (brightness < 100) {
             brightness += 8;
             if (brightness > 100) {brightness = 100;}
             milightClient->updateBrightness(brightness);
           }
         }
        } else {
          if (clicks[id] == 1) {
            if (temperature > 153) {
                temperature -= 12;
                if (temperature < 153) {temperature = 153;}
                milightClient->updateTemperature(Units::miredsToWhiteVal(temperature, 100));
            }
          } else {
            if (brightness > 0) {
              brightness -= 8;
              if (brightness < 0) {brightness = 0;}
              milightClient->updateBrightness(brightness);
            }
          }
        }
      }
    }
  }

  //Send command change update to MQTT mesh_in/milight to update other devices with same bulbId
  if (buttonDirty[id] == true && currentState[id] == HIGH && (millis() - firstTime[id]) > 1000) {
    buttonDirty[id] = false;

    BulbId bulbId(settings.gatewayConfigs[0]->deviceId, id+1, remoteConfig->type);

    char buffer[200];
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& message = jsonBuffer.createObject();

    GroupState& groupState = stateStore->get(bulbId);
    groupState.applyState(message, settings.groupStateFields, settings.numGroupStateFields);

    message.printTo(buffer);

    String topic = String(inTopic) + String(settings.mqttTopicPattern);

    String deviceIdStr = String(bulbId.deviceId, 16);
    deviceIdStr.toUpperCase();

    topic.replace(":device_id", String("0x") + deviceIdStr);
    topic.replace(":group_id", String(bulbId.groupId));
    topic.replace(":device_type", remoteConfig->name);

    #ifdef MQTT_DEBUG
      printf_P(PSTR("WallSwitch - send message to topic: %s : %s\r\n"), topic.c_str(), buffer);
    #endif

    //send command to mesh_in/milight/xxx/xxx/x
    callback(topic.c_str(), buffer, false);
  }

  previousState[id] = currentState[id];
}

//handle actions based on LDR state
void WallSwitch::doDayNight() {

  //detect night with LDR on A0 pin
  if (lastAnalogRead + 1000 < millis())
  {
    lastAnalogRead = millis();
    darknessLevel = analogRead(A0);

    #ifdef DEBUG_PRINTF
      Serial.print("darknessLevel: ");
      Serial.println(darknessLevel);
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

//Non blocking startup items
void WallSwitch::doLightState() {

  //turn all lights off at startup
  if (isStartUp == true && millis() > 15000) {

    isStartUp = false;

    milightClient->prepare(remoteConfig, settings.gatewayConfigs[0]->deviceId, 0);
    milightClient->updateTemperature(100);
    milightClient->updateStatus(OFF);
  }
}
