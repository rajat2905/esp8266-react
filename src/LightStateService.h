#ifndef LightStateService_h
#define LightStateService_h

#include <LightMqttSettingsService.h>

#include <HttpEndpoint.h>
#include <MqttPubSub.h>
#include <WebSocketTxRx.h>

#define LED_PIN 2

#define DEFAULT_LED_STATE false
#define DEFAULT_RGB_STRING "r0g0b0&"
#define OFF_STATE "OFF"
#define ON_STATE "ON"

// Note that the built-in LED is on when the pin is low on most NodeMCU boards.
// This is because the anode is tied to VCC and the cathode to the GPIO 4 (Arduino pin 2).
#ifdef ESP32
#define LED_ON 0x1
#define LED_OFF 0x0
#elif defined(ESP8266)
#define LED_ON 0x0
#define LED_OFF 0x1
#endif

#define LIGHT_SETTINGS_ENDPOINT_PATH "/rest/lightState"
#define LIGHT_SETTINGS_SOCKET_PATH "/ws/lightState"

class LightState {
 public:
  bool ledOn;
  String rgbString;
  static void read(LightState& settings, JsonObject& root) {
    root["led_on"] = settings.ledOn;
    root["rgbString"] = settings.rgbString;
  }

  static StateUpdateResult update(JsonObject& root, LightState& lightState) {
    boolean newState = root["led_on"] | DEFAULT_LED_STATE;
    String newRgbString = root["rgbString"] | DEFAULT_RGB_STRING;
    int value = strcmp(lightState.rgbString.c_str(),newRgbString.c_str());
    // change the new state, if required
    if(lightState.ledOn != newState || value != 0){
      if (lightState.ledOn != newState) {
        lightState.ledOn = newState;
      }
      if (value != 0) {
        lightState.rgbString = newRgbString;
      }
      return StateUpdateResult::CHANGED;
    }
    return StateUpdateResult::UNCHANGED;
  }

  static void haRead(LightState& settings, JsonObject& root) {
    root["state"] = settings.ledOn ? ON_STATE : OFF_STATE;
    root["rgbString"] = settings.rgbString;
  }

  static StateUpdateResult haUpdate(JsonObject& root, LightState& lightState) {
    String state = root["state"];
    String newRgbString = root["rgbString"];
    // parse new led state 
    boolean newState = false;
    int value = strcmp(lightState.rgbString.c_str(),newRgbString.c_str());
    if (state.equals(ON_STATE)) {
      newState = true;
    } else if (!state.equals(OFF_STATE)) {
      // return StateUpdateResult::ERROR;
    }
    if (newRgbString.equals("null")) {
      return StateUpdateResult::ERROR;
    }
    // change the new state, if required
    if(lightState.ledOn != newState || value != 0){
      if (lightState.ledOn != newState) {
        lightState.ledOn = newState;
      }
      if (value != 0) {
        lightState.rgbString = newRgbString;
      }
      return StateUpdateResult::CHANGED;
    }
    return StateUpdateResult::UNCHANGED;
  }
};

class LightStateService : public StatefulService<LightState> {
 public:
  LightStateService(AsyncWebServer* server,
                    SecurityManager* securityManager,
                    AsyncMqttClient* mqttClient,
                    LightMqttSettingsService* lightMqttSettingsService);
  void begin();

 private:
  HttpEndpoint<LightState> _httpEndpoint;
  MqttPubSub<LightState> _mqttPubSub;
  WebSocketTxRx<LightState> _webSocket;
  AsyncMqttClient* _mqttClient;
  LightMqttSettingsService* _lightMqttSettingsService;

  void registerConfig();
  void onConfigUpdated();
};

#endif
