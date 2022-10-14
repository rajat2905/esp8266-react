#include <LightStateService.h>
#include <EEPROM.h>
// Decode HTTP GET value
String aString = "0";
String bString = "0";
String cString = "0";
String dString = "0";
String pubtopic="";

// Red, green, and blue pins for PWM control
const int relay1 = 13;     // 13 corresponds to GPIO12
const int relay2 = 12;   // 12 corresponds to GPIO13
const int relay3 = 14;    // 14 corresponds to GPIO14
const int relay4 = 16;    // 14 corresponds to GPIO16

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
  _state.swString = DEFAULT_SW_STRING;

  EEPROM.begin(512);  //Initialize EEPROM
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  pinMode(relay3, OUTPUT);
  pinMode(relay4, OUTPUT);
  Serial.println(F("relaySwitch example\n\n"));
    int gpio=EEPROM.read(0);
    aString=String(bitRead(gpio, 0));
    bString=String(bitRead(gpio, 1));
    cString=String(bitRead(gpio, 2));
    dString=String(bitRead(gpio, 3));

    digitalWrite(relay1, aString.toInt());
    digitalWrite(relay2, bString.toInt());
    digitalWrite(relay3, cString.toInt());
    digitalWrite(relay4, dString.toInt());
    
    Serial.print("gpio:");
    Serial.println(gpio);
  //onConfigUpdated();
}

void LightStateService::onConfigUpdated() {
  digitalWrite(LED_PIN, _state.ledOn ? LED_ON : LED_OFF);

// Changes the output state according to the message
    String messageTemp= _state.swString;

    int pos1 = messageTemp.indexOf('a');
    int pos2 = messageTemp.indexOf('b');
    int pos3 = messageTemp.indexOf('c');
    int pos4 = messageTemp.indexOf('d');
    if(pos1>-1){
      aString = messageTemp.substring(pos1+1, pos1+2);
      if(aString.toInt()==0 || aString.toInt()==1){
        digitalWrite(relay1, aString.toInt());
      }
    }
    if(pos2>-1){
      bString = messageTemp.substring(pos2+1, pos2+2);
      if(bString.toInt()==0 || bString.toInt()==1){
        digitalWrite(relay2, bString.toInt());
      }
    }
    if(pos3>-1){
      cString = messageTemp.substring(pos3+1, pos3+2);
      if(cString.toInt()==0 || cString.toInt()==1){
        digitalWrite(relay3, cString.toInt());
      }
    }
    if(pos4>-1){
      dString = messageTemp.substring(pos4+1, pos4+2);
      if(dString.toInt()==0 || dString.toInt()==1){
        digitalWrite(relay4, dString.toInt());
      }
    }

    Serial.println(messageTemp);
    int value=0;
    String str=String(aString+bString+cString+dString);
    for(int i=0;i<4;i++){
      value*=2;
      if(str[i]=='1')value++;
    }
    Serial.print("gpioStr:");
    Serial.println(str);
    EEPROM.write(0, value);
    EEPROM.commit();    //Store data to EEPROM
    Serial.print("EEProm gpio:");
    Serial.println(EEPROM.read(0));
    // ledcWrite(redChannel, redString.toInt());
    // ledcWrite(greenChannel, greenString.toInt());
    // ledcWrite(blueChannel, blueString.toInt());
    // if(_state.ledOn){
    //   ledcWrite(redChannel, 0);
    //   ledcWrite(greenChannel, 0);
    //   ledcWrite(blueChannel, 255);
    // }else{
    //   ledcWrite(redChannel, 0);
    //   ledcWrite(greenChannel, 0);
    //   ledcWrite(blueChannel, 0);
    // }
  DynamicJsonDocument doc(256);
  doc["swString"] = String("a"+aString+"b"+bString+"c"+cString+"d"+dString);
  String payload;
  serializeJson(doc, payload);
  _mqttClient->publish(pubtopic.c_str(), 0, false, payload.c_str());

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
    pubtopic=pubTopic;
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
