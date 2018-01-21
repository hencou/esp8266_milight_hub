#include <functional>
#include <Arduino.h>
#include <MiLightClient.h>
#include <Settings.h>
#include <MiLightRadioConfig.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//DS18B20 sensor
#define ONE_WIRE_BUS D4

class WallSwitch {
public:
  WallSwitch(Settings& settings, MiLightClient*& milightClient, GroupStateStore*& stateStore, const char* inTopic, const char* outTopic);
  ~WallSwitch();

  std::function<void(const char *topic, const char *msg, const bool retain)> callback;
  void setCallback(std::function<void(const char *topic, const char *msg, const bool retain)> _callback);

  void begin();
  void loop(bool standAloneAP);

private:
  MiLightClient*& milightClient;
  const MiLightRemoteConfig* remoteConfig;
  Settings& settings;
  GroupStateStore*& stateStore;
  const char *inTopic;
  const char *outTopic;

  OneWire oneWire;
  DallasTemperature sensors;

  void checkButton(int buttonPin, uint8_t id);
  void doShortClicks(uint8_t id);
  void doLongClicks(uint8_t id);
  void doDayNight();
  void detectMotion();
  void doLightState();
  void sendMQTTCommand(uint8_t id);

  int buttonPins[3] = {D1, D2, D3};
  boolean currentState[4] = {HIGH, HIGH, HIGH, HIGH};       // Current state of the button (LOW is pressed)
  boolean previousState[4] = {HIGH, HIGH, HIGH, HIGH};      // Previous state of the button)
  boolean raisingBrightness[4] = {true, true, true, true};  //Dimming direction, must be known to do toggle
  boolean raisingTemperature[4] = {true, true, true, true}; //Changing temperature direction, must be known to do toggle
  long millis_held[4] = {0, 0, 0, 0};                       // How long the button was held (milliseconds)
  long millis_repeat[4] = {0, 0, 0, 0};                     // store the time from the last brightness/temperature command (milliseconds)
  unsigned long firstTime[4] = {0, 0, 0, 0};                // how long since the button was first pressed
  unsigned long timePressLimit[4] = {0, 0, 0, 0};           // timeslot since the button was first pressed, to do doubleclick
  uint8_t shortClicks[4] = {0, 0, 0, 0};
  //count shortClicks
  boolean buttonDirty[4] = {false, false, false, false};    //button pressed
  boolean initLongClick[4] = {false, false, false, false};  //true when long press starts
  unsigned long lastAnalogRead = 0;
  boolean isNight = false;
  boolean isMidNight = false;
  boolean previousNight = false;
  unsigned int darknessLevel = 0;
  unsigned long nightTime = 0;
  unsigned int nightTimer = 0;
  unsigned long delayTimer = 0;
  boolean previousDelay = false;
  boolean isStartUp = true;
  int motionLevel = 0;
  boolean motionState = false;
  boolean previousMotion = false;
  unsigned long firstMotion = 0;

  //MQTT parameterss
  long lastMsg = 0;
};
