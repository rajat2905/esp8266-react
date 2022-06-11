#include <LightStateService.h>
// Decode HTTP GET value
String redString = "0";
String greenString = "0";
String blueString = "0";

// Red, green, and blue pins for PWM control
const int redPin = 12;     // 13 corresponds to GPIO13
const int greenPin = 13;   // 12 corresponds to GPIO12
const int bluePin = 14;    // 14 corresponds to GPIO14

// Setting PWM frequency, channels and bit resolution
const int freq = 5000;
const int redChannel = 0;
const int greenChannel = 1;
const int blueChannel = 2;
// Bit resolution 2^8 = 256
const int resolution = 8;
LightStateService::LightStateService(AsyncWebServer* server,
                                     SecurityManager* securityManager,
                                     AsyncMqttClient* mqttClient,
                                     LightMqttSettingsService* lightMqttSettingsService) :
    _httpEndpoint(LightState::read,
                  LightState::update,
                  this,
                  server,
                  LIGHT_SETTINGS_ENDPOINT_PATH,
                  securityManager,
                  AuthenticationPredicates::IS_AUTHENTICATED),
    _mqttPubSub(LightState::haRead, LightState::haUpdate, this, mqttClient),
    _webSocket(LightState::read,
               LightState::update,
               this,
               server,
               LIGHT_SETTINGS_SOCKET_PATH,
               securityManager,
               AuthenticationPredicates::IS_AUTHENTICATED),
    _mqttClient(mqttClient),
    _lightMqttSettingsService(lightMqttSettingsService) {
  // configure led to be output
  pinMode(LED_PIN, OUTPUT);
  

  // configure MQTT callback
  _mqttClient->onConnect(std::bind(&LightStateService::registerConfig, this));

  // configure update handler for when the light settings change
  _lightMqttSettingsService->addUpdateHandler([&](const String& originId) { registerConfig(); }, false);

  // configure settings service update handler to update LED state
  addUpdateHandler([&](const String& originId) { onConfigUpdated(); }, false);
}

void LightStateService::begin() {
  _state.ledOn = DEFAULT_LED_STATE;
  _state.rgbString = DEFAULT_RGB_STRING;
 // configure LED PWM functionalitites
  ledcSetup(redChannel, freq, resolution);
  ledcSetup(greenChannel, freq, resolution);
  ledcSetup(blueChannel, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(redPin, redChannel);
  ledcAttachPin(greenPin, greenChannel);
  ledcAttachPin(bluePin, blueChannel);

  onConfigUpdated();
}

void LightStateService::onConfigUpdated() {
  digitalWrite(LED_PIN, _state.ledOn ? LED_ON : LED_OFF);

// Changes the output state according to the message
    String messageTemp= _state.rgbString;
    int pos1 = messageTemp.indexOf('r');
    int pos2 = messageTemp.indexOf('g');
    int pos3 = messageTemp.indexOf('b');
    int pos4 = messageTemp.indexOf('&');
    redString = messageTemp.substring(pos1+1, pos2);
    greenString = messageTemp.substring(pos2+1, pos3);
    blueString = messageTemp.substring(pos3+1, pos4);
 
    ledcWrite(redChannel, redString.toInt());
    ledcWrite(greenChannel, greenString.toInt());
    ledcWrite(blueChannel, blueString.toInt());
    // if(_state.ledOn){
    //   ledcWrite(redChannel, 0);
    //   ledcWrite(greenChannel, 0);
    //   ledcWrite(blueChannel, 255);
    // }else{
    //   ledcWrite(redChannel, 0);
    //   ledcWrite(greenChannel, 0);
    //   ledcWrite(blueChannel, 0);
    // }


}

void LightStateService::registerConfig() {
  if (!_mqttClient->connected()) {
    return;
  }
  String configTopic;
  String subTopic;
  String pubTopic;

  DynamicJsonDocument doc(256);
  _lightMqttSettingsService->read([&](LightMqttSettings& settings) {
    configTopic = settings.mqttPath + "/config";
    subTopic = settings.mqttPath + "/set";
    pubTopic = settings.mqttPath + "/state";
    doc["~"] = settings.mqttPath;
    doc["name"] = settings.name;
    doc["unique_id"] = settings.uniqueId;
  });
  doc["cmd_t"] = "~/set";
  doc["stat_t"] = "~/state";
  doc["schema"] = "json";
  doc["brightness"] = false;

  String payload;
  serializeJson(doc, payload);
  _mqttClient->publish(configTopic.c_str(), 0, false, payload.c_str());

  _mqttPubSub.configureTopics(pubTopic, subTopic);
}
