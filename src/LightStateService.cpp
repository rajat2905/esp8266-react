#include <LightStateService.h>
#include <Wire.h>
#include "SparkFun_VL53L1X.h"

#include <EEPROM.h>
String pubtopic = "";
String devicename = "";

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

uint16_t towardsInCnt = 0;
uint16_t towardsOutCnt = 0;

#define DEBUG_MODE true
// MQTT Topics
// people_counter/DEVICENAME/counter
// people_counter/DEVICENAME/distance
// people_counter/DEVICENAME/receiver

const int threshold_percentage = 80;

// if "true", the raw measurements are sent via MQTT during runtime (for debugging) - I'd recommend setting it to
// "false" to save traffic and system resources. in the calibration phase the raw measurements will still be sent
// through MQTT
static bool update_raw_measurements = false;
static bool caliberate_zones=false;

// this value has to be true if the sensor is oriented as in Duthdeffy's picture
static bool advised_orientation_of_the_sensor = true;

// this value has to be true if you don't need to compute the threshold every time the device is turned on
static bool save_calibration_result = true;

// parameters which define the time between two different measurements in longRange mode
static int delay_between_measurements_long = 55;
static int time_budget_in_ms_long = 50;

// parameters which define the time between two different measurements in longRange mode
static int time_budget_in_ms_short = 20;
static int delay_between_measurements_short = 22;

// value which defines the threshold which activates the short distance mode (the sensor supports it only up to a
// distance of 1300 mm)
static int short_distance_threshold = 1300;

//*******************************************************************************************************************
// all the code from this point and onwards doesn't have to be touched in order to have everything working (hopefully)

char mqtt_serial_publish_ch_cache[50];
char mqtt_serial_publish_message_ch_cache[50];
char mqtt_serial_publish_distance_ch_cache[50];
char mqtt_serial_receiver_ch_cache[50];

int mqtt_counter = sprintf(mqtt_serial_publish_ch_cache, "%s%s%s", "people_counter/", devicename, "/counter");
const PROGMEM char* mqtt_serial_publish_ch = mqtt_serial_publish_ch_cache;
int mqtt_message =
    sprintf(mqtt_serial_publish_message_ch_cache, "%s%s%s", "people_counter/", devicename, "/message");
const PROGMEM char* mqtt_serial_publish_message_ch = mqtt_serial_publish_message_ch_cache;
int mqtt_distance =
    sprintf(mqtt_serial_publish_distance_ch_cache, "%s%s%s", "people_counter/", devicename, "/distance");
const PROGMEM char* mqtt_serial_publish_distance_val = mqtt_serial_publish_distance_ch_cache;
int mqtt_receiver = sprintf(mqtt_serial_receiver_ch_cache, "%s%s%s", "people_counter/", devicename, "/receiver");
const PROGMEM char* mqtt_serial_receiver_ch = mqtt_serial_receiver_ch_cache;

#define EEPROM_SIZE 8
char peopleCounterArray[50];
// Optional interrupt and shutdown pins.  Vanno cambiati e messi quelli che hanno i collegamenti i^2C
#define SHUTDOWN_PIN 4
#define INTERRUPT_PIN 2

// SFEVL53L1X distanceSensor(Wire, SHUTDOWN_PIN, INTERRUPT_PIN);
SFEVL53L1X distanceSensor;
static int NOBODY = 0;
static int SOMEONE = 1;
static int LEFT = 0;
static int RIGHT = 1;

static int DIST_THRESHOLD_MAX[] = {0, 0};  // treshold of the two zones
static int MIN_DISTANCE[] = {0, 0};

static int PathTrack[] = {0, 0, 0, 0};
static int PathTrackFillingSize = 1;  // init this to 1 as we start from state where nobody is any of the zones
static int LeftPreviousStatus = NOBODY;
static int RightPreviousStatus = NOBODY;

static int center[2] = {0, 0}; /* center of the two zones */
static int Zone = 0;
static int PplCounter = 0;

static int ROI_height = 0;
static int ROI_width = 0;

static int delay_between_measurements = 0;
static int time_budget_in_ms = 0;
void LightStateService::publishPersonPassage(int serialData) {
  serialData = max(0, serialData);

  String stringaCounter = String(serialData);
  stringaCounter.toCharArray(peopleCounterArray, stringaCounter.length() + 1);
  _mqttClient->publish(mqtt_serial_publish_ch, 0, false, peopleCounterArray);
}

void LightStateService::publishDistance(int serialData, int zona) {
  // serialData = max(0, serialData);

  // String stringaZona = "\t zona = ";
  // String stringaCounter = String(serialData) + stringaZona + String(zona) + "\t" + String(PathTrack[0]) +
  //                         String(PathTrack[1]) + String(PathTrack[2]) + String(PathTrack[3]);
  // stringaCounter.toCharArray(peopleCounterArray, stringaCounter.length() + 1);
  //   publishDistanceTopic((String)peopleCounterArray);



          DynamicJsonDocument doc(1024);
      doc["Distance"] = serialData;
      doc["zona"] = zona;
      doc["PathTrack0"] = PathTrack[0];
      doc["PathTrack1"] = PathTrack[1];
      doc["PathTrack2"] = PathTrack[2];
      doc["PathTrack3"] = PathTrack[3];
      char jsonString[256];
      size_t n = serializeJson(doc, jsonString);
      Serial.print("Topic:");
      Serial.print(mqtt_serial_publish_distance_val);
      Serial.print(", Message:");
      Serial.println(jsonString);
      _mqttClient->publish(mqtt_serial_publish_distance_val, 0, false, jsonString);
}

void LightStateService::zones_calibration_boot() {
  if (save_calibration_result) {
    Serial.println("saving calibration result!");
    // if possible, we take the old values of the zones contained in the EEPROM memory
    publishDistanceTopic("save calibration result true");
    if (EEPROM.read(0) == 1) {
      // we have data in the EEPROM
      publishDistanceTopic("EEPROM memroy not empty");
      center[0] = EEPROM.read(1);
      center[1] = EEPROM.read(2);
      ROI_height = EEPROM.read(3);
      ROI_width = EEPROM.read(3);
      DIST_THRESHOLD_MAX[0] = EEPROM.read(4) * 100 + EEPROM.read(5);
      DIST_THRESHOLD_MAX[1] = EEPROM.read(6) * 100 + EEPROM.read(7);

      // if the distance measured is small, then we can use the short range mode of the sensor
      if (min(DIST_THRESHOLD_MAX[0], DIST_THRESHOLD_MAX[1]) <= short_distance_threshold) {
        distanceSensor.setIntermeasurementPeriod(time_budget_in_ms_short);
        distanceSensor.setDistanceModeShort();
        time_budget_in_ms = time_budget_in_ms_short;
        delay_between_measurements = delay_between_measurements_short;
      } else {
        distanceSensor.setIntermeasurementPeriod(time_budget_in_ms_long);
        distanceSensor.setDistanceModeLong();
        time_budget_in_ms = time_budget_in_ms_long;
        delay_between_measurements = delay_between_measurements_long;
      }
      publishDistance(center[0], 0);
      publishDistance(center[1], 0);
      publishDistance(ROI_width, 0);
      publishDistance(ROI_height, 0);
      publishDistance(DIST_THRESHOLD_MAX[0], 0);
      publishDistance(DIST_THRESHOLD_MAX[1], 0);
        publishDistanceTopic("All values updated");
    } else {
      // there are no data in the EEPROM memory
      Serial.println("there are no data in the EEPROM memory, going to zone calibration!");
      zones_calibration();
    }
  } else {
    Serial.println("skipping calibration result, going to zone calibration directly!");
    zones_calibration();
  }
}

void LightStateService::zones_calibration() {
  Serial.println("caliberating zones now!");
  // the sensor does 100 measurements for each zone (zones are predefined)
  // each measurements is done with a timing budget of 100 ms, to increase the precision
    publishDistanceTopic("Computation of new threshold");
  // we set the standard values for the measurements
  distanceSensor.setIntermeasurementPeriod(time_budget_in_ms_long);
  distanceSensor.setDistanceModeLong();
  time_budget_in_ms = time_budget_in_ms_long;
  delay_between_measurements = delay_between_measurements_long;
  center[0] = 175;  // was 167 as per UM2555
  center[1] = 231;
  ROI_height = 8;
  ROI_width = 8;
  delay(500);

  Zone = 0;
  float sum_zone_0 = 0;
  float sum_zone_1 = 0;
  uint16_t distance;
  int number_attempts = 20;
  for (int i = 0; i < number_attempts; i++) {
    // increase sum of values in Zone 0
    distanceSensor.setROI(
        ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
    delay(delay_between_measurements);
    distanceSensor.setTimingBudgetInMs(time_budget_in_ms);
    distanceSensor.startRanging();            // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance();  // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_0 = sum_zone_0 + distance;
    publishDistance(distance, 0);
    Zone++;
    Zone = Zone % 2;

    // increase sum of values in Zone 1
    distanceSensor.setROI(
        ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
    delay(delay_between_measurements);
    distanceSensor.setTimingBudgetInMs(time_budget_in_ms);
    distanceSensor.startRanging();            // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance();  // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_1 = sum_zone_1 + distance;
    publishDistance(distance, 1);
    Zone++;
    Zone = Zone % 2;
  }
  // after we have computed the sum for each zone, we can compute the average distance of each zone
  float average_zone_0 = sum_zone_0 / number_attempts;
  float average_zone_1 = sum_zone_1 / number_attempts;
  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the
  // center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (min(average_zone_0, average_zone_1) / 1000)));
  publishDistance(function_of_the_distance, 1);
  delay(1000);
  int ROI_size = min(8, max(4, function_of_the_distance));
  ROI_width = ROI_size;
  ROI_height = ROI_size;
  if (average_zone_0 <= short_distance_threshold || average_zone_1 <= short_distance_threshold) {
    // we can use the short mode, which allows more precise measurements up to 1.3 meters
    distanceSensor.setIntermeasurementPeriod(time_budget_in_ms_short);
    distanceSensor.setDistanceModeShort();
    time_budget_in_ms = time_budget_in_ms_short;
    delay_between_measurements = delay_between_measurements_short;
  }
  delay(250);

  // now we set the position of the center of the two zones
  if (advised_orientation_of_the_sensor) {
    switch (ROI_size) {
      case 4:
        center[0] = 150;
        center[1] = 247;
        break;
      case 5:
        center[0] = 159;
        center[1] = 239;
        break;
      case 6:
        center[0] = 159;
        center[1] = 239;
        break;
      case 7:
        center[0] = 167;
        center[1] = 231;
        break;
      case 8:
        center[0] = 167;
        center[1] = 231;
        break;
    }
  } else {
    switch (ROI_size) {
      case 4:
        center[0] = 193;
        center[1] = 58;
        break;
      case 5:
        center[0] = 194;
        center[1] = 59;
        break;
      case 6:
        center[0] = 194;
        center[1] = 59;
        break;
      case 7:
        center[0] = 195;
        center[1] = 60;
        break;
      case 8:
        center[0] = 195;
        center[1] = 60;
        break;
    }
  }
  publishDistanceTopic("ROI size");
  publishDistance(ROI_size, 0);
  publishDistanceTopic("centers of the ROIs defined");
  publishDistance(center[0], 0);
  publishDistance(center[1], 1);
  delay(2000);
  // we will now repeat the calculations necessary to define the thresholds with the updated zones
  Zone = 0;
  sum_zone_0 = 0;
  sum_zone_1 = 0;
  for (int i = 0; i < number_attempts; i++) {
    // increase sum of values in Zone 0
    distanceSensor.setROI(
        ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
    delay(delay_between_measurements);
    distanceSensor.setTimingBudgetInMs(time_budget_in_ms);
    distanceSensor.startRanging();            // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance();  // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_0 = sum_zone_0 + distance;
    publishDistance(distance, 0);
    Zone++;
    Zone = Zone % 2;

    // increase sum of values in Zone 1
    distanceSensor.setROI(
        ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
    delay(delay_between_measurements);
    distanceSensor.setTimingBudgetInMs(time_budget_in_ms);
    distanceSensor.startRanging();            // Write configuration bytes to initiate measurement
    distance = distanceSensor.getDistance();  // Get the result of the measurement from the sensor
    distanceSensor.stopRanging();
    sum_zone_1 = sum_zone_1 + distance;
    publishDistance(distance, 1);
    Zone++;
    Zone = Zone % 2;
  }
  average_zone_0 = sum_zone_0 / number_attempts;
  average_zone_1 = sum_zone_1 / number_attempts;
  float threshold_zone_0 =
      average_zone_0 * threshold_percentage /
      100;  // they can be int values, as we are not interested in the decimal part when defining the threshold
  float threshold_zone_1 = average_zone_1 * threshold_percentage / 100;

  DIST_THRESHOLD_MAX[0] = threshold_zone_0;
  DIST_THRESHOLD_MAX[1] = threshold_zone_1;
  publishDistanceTopic("new threshold defined");
  publishDistance(threshold_zone_0, 0);
  publishDistance(threshold_zone_1, 1);
  delay(2000);

  // we now save the values into the EEPROM memory
  int hundred_threshold_zone_0 = threshold_zone_0 / 100;
  int hundred_threshold_zone_1 = threshold_zone_1 / 100;
  int unit_threshold_zone_0 = threshold_zone_0 - 100 * hundred_threshold_zone_0;
  int unit_threshold_zone_1 = threshold_zone_1 - 100 * hundred_threshold_zone_1;

  EEPROM.write(0, 1);
  EEPROM.write(1, center[0]);
  EEPROM.write(2, center[1]);
  EEPROM.write(3, ROI_size);
  EEPROM.write(4, hundred_threshold_zone_0);
  EEPROM.write(5, unit_threshold_zone_0);
  EEPROM.write(6, hundred_threshold_zone_1);
  EEPROM.write(7, unit_threshold_zone_1);
  EEPROM.commit();
}
void LightStateService::begin() {
  _state.ledOn = DEFAULT_LED_STATE;
  _state.swString = DEFAULT_SW_STRING;
  Wire.begin();
  // initialize the EEPROM memory
  EEPROM.begin(EEPROM_SIZE);

  Serial.println("VL53L1X Qwiic Test");
  if (distanceSensor.begin() != 0)  // Begin returns 0 on a good init
  {
    Serial.println("Sensor failed to begin. Please check wiring. Freezing...");
    while (1)
      ;
  }
  Serial.println("Sensor online!");
  zones_calibration_boot();

  // onConfigUpdated();
}
void LightStateService::loop() {
  _state.ledOn = DEFAULT_LED_STATE;
  _state.swString = DEFAULT_SW_STRING;
  uint16_t distance;
  if(caliberate_zones){
    caliberate_zones=false;
    zones_calibration();
  }
  distanceSensor.setROI(
      ROI_height, ROI_width, center[Zone]);  // first value: height of the zone, second value: width of the zone
  delay(delay_between_measurements);
  distanceSensor.setTimingBudgetInMs(time_budget_in_ms);
  distanceSensor.startRanging();            // Write configuration bytes to initiate measurement
  distance = distanceSensor.getDistance();  // Get the result of the measurement from the sensor
  distanceSensor.clearInterrupt();
  distanceSensor.stopRanging();

  // Serial.println(distance);
  if (update_raw_measurements == true) {
    publishDistance(distance, Zone);
  }

  // inject the new ranged distance in the people counting algorithm
  processPeopleCountingData(distance, Zone);

  Zone++;
  Zone = Zone % 2;
  // onConfigUpdated();
}

void LightStateService::onConfigUpdated() {
  digitalWrite(LED_PIN, _state.ledOn ? LED_ON : LED_OFF);
  DynamicJsonDocument doc(256);
  // doc["swString"] = String("a"+aString+"b"+bString+"c"+cString+"d"+dString);

  Serial.println("-------new message from broker-----");
  String newPayload = _state.swString;
    if (newPayload == "zones_calibration") {
        caliberate_zones = true;
    }
    if (newPayload == "update_raw_measurements") {
      if (update_raw_measurements) {
        // it is true, then it is going to be changed to false
        update_raw_measurements = false;
      } else {
        update_raw_measurements = true;
      }
      delay(200);
    }
    if (newPayload == "R") {
      towardsInCnt = 0;
      towardsOutCnt = 0;
      DynamicJsonDocument doc(1024);
      doc["Distance"] = 0;
      doc["towardsIn"] = towardsInCnt;
      doc["towardsOut"] = towardsOutCnt;
      doc["InsideRoom"] = 0;
      char jsonString[256];
      size_t n = serializeJson(doc, jsonString);
      Serial.println(jsonString);
      // client.publish(mqtt_topic_response, jsonString, n);
      _mqttClient->publish(pubtopic.c_str(), 0, false, jsonString);
    }
}
void LightStateService::publishDistanceTopic(String str) {
      DynamicJsonDocument doc(1024);
      doc["DistanceString"] = str;
      char jsonString[256];
      size_t n = serializeJson(doc, jsonString);
      Serial.print("Topic:");
      Serial.print(mqtt_serial_publish_message_ch);
      Serial.print(", Message:");
      Serial.println(jsonString);
      _mqttClient->publish(mqtt_serial_publish_message_ch, 0, false, jsonString);
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
    pubtopic = pubTopic;
    devicename = settings.name;

    sprintf(mqtt_serial_publish_ch_cache, "%s%s%s", "people_counter/", devicename, "/counter");
    mqtt_serial_publish_ch = mqtt_serial_publish_ch_cache;
    sprintf(mqtt_serial_publish_message_ch_cache, "%s%s%s", "people_counter/", devicename, "/message");
    mqtt_serial_publish_message_ch = mqtt_serial_publish_message_ch_cache;
    sprintf(mqtt_serial_publish_distance_ch_cache, "%s%s%s", "people_counter/", devicename, "/distance");
    mqtt_serial_publish_distance_val = mqtt_serial_publish_distance_ch_cache;
    sprintf(mqtt_serial_receiver_ch_cache, "%s%s%s", "people_counter/", devicename, "/receiver");
    mqtt_serial_receiver_ch = mqtt_serial_receiver_ch_cache;

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

// NOBODY = 0, SOMEONE = 1, LEFT = 0, RIGHT = 1

void LightStateService::processPeopleCountingData(int16_t Distance, uint8_t zone) {
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;
  // Serial.print("dist=");
  // Serial.print(Distance);
  // Serial.print(", zone=");
  // Serial.println(zone);
  if (Distance < DIST_THRESHOLD_MAX[Zone] && Distance > MIN_DISTANCE[Zone]) {
    //    Serial.println("Someone is in !");
    CurrentZoneStatus = SOMEONE;
  }

  // left zone
  if (zone == LEFT) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != RightPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to left right zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1 3 2) and last event is 0 (nobobdy
      // anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is always the case
        Serial.println();
        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // this is an entry
          publishPersonPassage(1);
          DynamicJsonDocument doc(1024);
          doc["Distance"] = Distance;
          towardsInCnt++;
          doc["towardsIn"] = towardsInCnt;
          doc["towardsOut"] = towardsOutCnt;
          if (towardsInCnt >= towardsOutCnt) {
            doc["InsideRoom"] = towardsInCnt - towardsOutCnt;
          } else {
            doc["InsideRoom"] = 0;
          }
          char jsonString[256];
          size_t n = serializeJson(doc, jsonString);
          Serial.println(jsonString);
          // client.publish(mqtt_topic_response, jsonString, n);
          _mqttClient->publish(pubtopic.c_str(), 0, false, jsonString);
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an exit
          publishPersonPassage(2);
          DynamicJsonDocument doc(1024);
          doc["Distance"] = Distance;
          towardsOutCnt++;
          doc["towardsIn"] = towardsInCnt;
          doc["towardsOut"] = towardsOutCnt;
          if (towardsInCnt >= towardsOutCnt) {
            doc["InsideRoom"] = towardsInCnt - towardsOutCnt;
          } else {
            doc["InsideRoom"] = 0;
          }
          char jsonString[256];
          size_t n = serializeJson(doc, jsonString);
          Serial.println(jsonString);
          // client.publish(mqtt_topic_response, jsonString, n);
          _mqttClient->publish(pubtopic.c_str(), 0, false, jsonString);
        }
      }
      for (int i = 0; i < 4; i++) {
        PathTrack[i] = 0;
      }
      PathTrackFillingSize = 1;
    } else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
    }
  }
}
