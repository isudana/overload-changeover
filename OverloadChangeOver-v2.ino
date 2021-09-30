/**

Overload ChangeOver for Offgrid

   Pins
   ====
   RX           - TX of PZEM
   TX           - RX of PZEM
   GPIO5 (D1)   - Relay 2 (Inverter) Trigger
   GPIO4 (D2)   - Relay 1 (Grid) Trigger
   GPIO16       - Power Status LED
   GPIO2 (D4)   - 
   GPIO15 (D8)  - 

*/

#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <MQTTClient.h>
#include <ModbusMaster.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

/* Configuration Parameters */
// WIFI
const char* ssid1  = "";
const char* password1 = "";
const char* ssid2  = "";
const char* password2 = "";

// MQTT
const char* mqttServerIP = "";
const char* mqttClientName = "Offgrid_OverloadChangeOver";
const char* mqttUserName = "";
const char* mqttPassword = "";

String COMMAND_MAIN_TOPIC = "controllers/offgrid/overloadco/cmd";
String COMMAND_POWER_SUB_TOPIC = "/maxpower";
String COMMAND_INTERVAL_SUB_TOPIC = "/interval";
String COMMAND_RESPONSE_SUB_TOPIC = "/response";
String STATUS_TOPIC = "controllers/offgrid/overloadco/status";
String STAT_TOPIC = "controllers/offgrid/overloadco/stat";
String EVENT_TOPIC = "controllers/offgrid/overloadco/event";

// OTA
const char* OTA_HOSTNAME = "OFFGRID_OVERLOAD_CHANGEOVER";

// Intervals
double powerThreshold = 2000;
double transitionInterval = 5000; // 5 secs
int delayBeforeBackToNormal = 120000;   // 2 mins
int outputStatsPublishingInterval = 5000; // 5 secs
int pzemReadInterval = 1000;
int ledBlinkOnOverloadInterval = 500;
int ledBlinkOnTransitionInterval = 250;
int ledBlinkOnSystemOKOnInterval = 1000; 
int ledBlinkOnSystemOKOffInterval = 5000;
int connectionRetryInterval = 10000;        // 10s
int connectionSuspensionDuration = 600000; // 10min

bool debug = false;
/* End of Configuration Parameters */


// Variable for PINs
const int SYSTEM_STATE_LED_PIN = LED_BUILTIN;;
const int POWER_STATE_LED_PIN = 16;

const int GRID_RELAY_PIN = 4;
const int INVERTER_RELAY_PIN = 5;

const int RX_PIN = D1;
const int TX_PIN = D2;

// Components variables
ESP8266WiFiMulti wifiMulti;
WiFiClient wifiClient;
MQTTClient mqttClient(256);
ModbusMaster modbus;
SoftwareSerial loggingSerial(RX_PIN, TX_PIN); // (RX,TX)

// EEPRom configuration
int powerThresholdIndex = 0;
int transitionIntervalIndex = 1;

// Variables for business logic
enum OVERLOAD_STATE {
  INIT,
  TRANSITION_TO_NORMAL,
  NORMAL,
  TRANSITION_TO_OVERLOADED,
  OVERLOADED,
  RESTORED
};

OVERLOAD_STATE overloadState = INIT;
unsigned long restoredTime = 0;
unsigned long transitionStartTime = 0;
unsigned long statsPublishedTime = 0;
unsigned long pzemReadTime = 0;
unsigned long powerLedBlinkToggleTime = 0;
unsigned long systemLedBlinkToggleTime = 0;
unsigned long connectionNextRetryTime = 0;

bool powerStateLedOn = false;
bool systemStateLedOn = false;
bool offlineMode = false;

void setup() {

  if (debug) {
    loggingSerial.begin(115200);
  }

  // Init WiFi
  wifiMulti.addAP(ssid1, password1);   // add Wi-Fi networks you want to connect to
  wifiMulti.addAP(ssid2, password2);
  connectWifi();

  // Init MQTT
  mqttClient.begin(mqttServerIP, wifiClient);
  connectMqtt();

  // Load configs from EEPRom
  EEPROM.begin(512);
  byte tmpPowerThreshold = EEPROM.read(powerThresholdIndex);
  doLog("Loaded Max Power from EEPRom: "); doLogln(tmpPowerThreshold);
  if (tmpPowerThreshold == 0) {
    storeMaxPower();
  }  else {
    powerThreshold = tmpPowerThreshold * 100;
  }
  doLog("Max Power Threshold: "); doLogln(powerThreshold);

  byte tmpTransitionInterval = EEPROM.read(transitionIntervalIndex);
  doLog("Loaded TransitionInterval parameter from EEPRom: "); doLogln(tmpTransitionInterval);
  if (tmpTransitionInterval == 0) {
    storeTransitionInterval();
  }  else {
    transitionInterval = tmpTransitionInterval * 100;
  }
  doLog("Transition Interval: "); doLogln(transitionInterval);

  setupOTA();

  // Init pzem
  Serial.begin(9600);
  modbus.begin(1, Serial);

  // Variable initialization for business logic
  pinMode(SYSTEM_STATE_LED_PIN, OUTPUT);
  pinMode(POWER_STATE_LED_PIN, OUTPUT);
  pinMode(GRID_RELAY_PIN, OUTPUT);
  pinMode(INVERTER_RELAY_PIN, OUTPUT);

  digitalWrite(SYSTEM_STATE_LED_PIN, HIGH);
  digitalWrite(POWER_STATE_LED_PIN, HIGH);
  digitalWrite(GRID_RELAY_PIN, LOW);
  digitalWrite(INVERTER_RELAY_PIN, LOW);
  
  doLogln("Ready");
  
}

void setupOTA() {
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_SPIFFS
      type = "filesystem";
    }
    doLogln("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    doLogln("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    doLogln("Progress: %u%%\r" + (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    doLogln("Error[%u]: " + error);
    if (error == OTA_AUTH_ERROR) {
      doLogln("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      doLogln("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      doLogln("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      doLogln("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      doLogln("End Failed");
    }
  });
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.begin();
}

void storeMaxPower() {
  EEPROM.write(powerThresholdIndex, powerThreshold / 100);
  EEPROM.commit();
}

void storeTransitionInterval() {
  EEPROM.write(transitionIntervalIndex, transitionInterval / 100);
  EEPROM.commit();  
}

void goOnline() {

  unsigned long currentTime = millis();

  if (currentTime < connectionNextRetryTime) {
     return;   
  }

  if (wifiMulti.run() != WL_CONNECTED) {
    if (!connectWifi()) {
       connectionNextRetryTime = currentTime + connectionSuspensionDuration;
       offlineMode = true;  
       return;
    }  
  }
  
  if (!mqttClient.connected()) {
    if (!connectMqtt()) {
        connectionNextRetryTime = currentTime + connectionSuspensionDuration;
        offlineMode = true;
    }
  }
}

bool connectWifi() {
  unsigned long currentTime = millis();
  boolean isSuccess = true; 
  doLogln("Connecting to WIFI network...");
  while (wifiMulti.run() != WL_CONNECTED) {
    if (millis() > (currentTime + connectionRetryInterval)) {
      isSuccess = false;
      break;
    }
    doLog(".");
    delay(1000);
  }
  if (isSuccess) {
    doLogln("\n");
    doLog("Connected to ");
    doLogln(WiFi.SSID());
  } else {  
    doLogln("Unable to connect to WiFi network, Continuing with offline mode");    
  }
  return isSuccess;
}

bool connectMqtt() {
  unsigned long currentTime = millis();
  boolean isSuccess = true;   
  doLogln("Connecting to MQTT broker...");
  while (!mqttClient.connect(mqttClientName, mqttUserName, mqttPassword)) {
    if (millis() > (currentTime + connectionRetryInterval)) {
      isSuccess = false;
      break;
    }
    delay(1000);
  }
  if (isSuccess) {
    doLogln("Connected to MQTT Broker");
    mqttClient.onMessage(mqttOnMessage);

    // Subscribe to topics
    mqttClient.subscribe(COMMAND_MAIN_TOPIC);
    mqttClient.subscribe(COMMAND_MAIN_TOPIC + COMMAND_POWER_SUB_TOPIC);
    mqttClient.subscribe(COMMAND_MAIN_TOPIC + COMMAND_INTERVAL_SUB_TOPIC);

  
    // Publish the online message to a topic
    publishMessage(STATUS_TOPIC, "ONLINE");
  } else {  
    doLogln("Unable to connect to MQTT Broker, Continuing with offline mode");    
  }
  return isSuccess;
}

void loop() {
  ArduinoOTA.handle();
  mqttClient.loop();
  goOnline();
  businessLogic();
}

void businessLogic() {
  monitorPowerUsage();
  handleLEDGlow();
}

void monitorPowerUsage() {

  unsigned long currentTime = millis();

  // Read from PZEM
  if (currentTime >= (pzemReadTime + pzemReadInterval)) {
    pzemReadTime = currentTime;
    uint8_t pzemResult = modbus.readInputRegisters(0x0000, 10);
    double PZEM_V, PZEM_A, PZEM_W, PZEM_E, PZEM_F, PZEM_PF, PZEM_ALARM;
    if (pzemResult == modbus.ku8MBSuccess) {
      PZEM_V = (modbus.getResponseBuffer(0x00) / 10.0f);
      PZEM_A = (modbus.getResponseBuffer(0x01) / 1000.000f);
      PZEM_W = (modbus.getResponseBuffer(0x03) / 10.0f);
      PZEM_E = (modbus.getResponseBuffer(0x05) / 1000.0f);
      PZEM_F = (modbus.getResponseBuffer(0x07) / 10.0f);
      PZEM_PF = (modbus.getResponseBuffer(0x08) / 100.0f);
      PZEM_ALARM = (modbus.getResponseBuffer(0x09));
      if (PZEM_V > 500 && PZEM_W > 5000) { // Filter Noise
        return;  
      }
    } else {
      PZEM_V = 0;
      PZEM_A = 0;
      PZEM_W = 0;
      PZEM_E = 0;
      PZEM_PF = 0;
      PZEM_ALARM = 0;
    }
    handlePowerOverload(PZEM_W);

    if (debug) {
      doLog("Voltage: "); doLogln(PZEM_V);
      doLog("Current: "); doLogln(PZEM_A);
      doLog("Power: "); doLogln(PZEM_W);
      doLog("Energy: "); doLogln(PZEM_E);
      doLog("Frequency: "); doLogln(PZEM_F);
      doLog("Power Factor: "); doLogln(PZEM_PF);
      doLog("Alarm Status: "); doLogln(PZEM_ALARM);
      doLogln("===");
    }

    String aggregatedResult = "{\"Voltage\":" + String(PZEM_V)
                              + ", \"Current\":" + String(PZEM_A)
                              + ", \"Power\":" + String(PZEM_W)
                              + ", \"Frequency\":" + String(PZEM_F)
                              + ", \"Power Factor\":" + String(PZEM_PF)
                              + ", \"Energy\":" + String(PZEM_E) + "}";
    if (debug) {
      doLogln(aggregatedResult);
    }

    // Send mqtt message
    if (currentTime >= (statsPublishedTime + outputStatsPublishingInterval)) {
      statsPublishedTime = currentTime;
      publishMessage(STAT_TOPIC, aggregatedResult);
    }
  }
}

void handlePowerOverload(double power) {
  if (overloadState == NORMAL) {
    if (power < powerThreshold) {
      return;
    } else {
      publishMessage(COMMAND_MAIN_TOPIC + COMMAND_RESPONSE_SUB_TOPIC, String(power));
      publishMessage(EVENT_TOPIC, "OVERLOAD");
      overloadState = TRANSITION_TO_OVERLOADED;
      transitionStartTime = millis();
      turnOffPower();
    }
  } else if (overloadState == TRANSITION_TO_OVERLOADED) {
    if (millis() > (transitionStartTime + transitionInterval)) {
        switchToGrid();
        overloadState = OVERLOADED;
    }
  } else if (overloadState == OVERLOADED) {
    if (power < powerThreshold) {
      publishMessage(EVENT_TOPIC, "RESTORED");
      overloadState = RESTORED;
      restoredTime = millis();
    }
  } else if (overloadState == RESTORED) {
    if (power >= powerThreshold) {
      publishMessage(EVENT_TOPIC, "OVERLOAD");
      overloadState = OVERLOADED;
    } else {
      if (millis() > (restoredTime + delayBeforeBackToNormal)) {
        publishMessage(EVENT_TOPIC, "NORMAL");
        turnOffPower();
        transitionStartTime = millis();
        overloadState = TRANSITION_TO_NORMAL;
      }
    }
  } else if (overloadState == TRANSITION_TO_NORMAL) {
    if (millis() > (transitionStartTime + transitionInterval)) {
        switchToInverter();
        overloadState = NORMAL;
    }
  } else if (overloadState == INIT) {
    if (power < powerThreshold) {
        publishMessage(EVENT_TOPIC, "NORMAL");
        switchToInverter();
        overloadState = NORMAL;
    } else {
        switchToGrid();
        publishMessage(EVENT_TOPIC, "OVERLOAD");
        overloadState = OVERLOADED;  
    }
  }
}

void mqttOnMessage(String &topic, String &payload) {

  if (topic == (COMMAND_MAIN_TOPIC + COMMAND_POWER_SUB_TOPIC)) {
    doLog("Setting Max Power to: "); doLogln(payload);
    powerThreshold = payload.toDouble();
    storeMaxPower();
  } else if (topic == (COMMAND_MAIN_TOPIC + COMMAND_INTERVAL_SUB_TOPIC)) {
    doLog("Setting Transistion Interval to: "); doLogln(payload);
    transitionInterval = payload.toDouble();
    storeTransitionInterval();
  } else if (topic == COMMAND_MAIN_TOPIC) {
    if (payload == "OFF") {
      turnOffPower();
    } else if (payload == "GRID") {
      switchToGrid();
    } else if (payload == "INVERTER") {
      switchToInverter();
    } else if (payload == "DEBUG_ON") {
      debug = true;
      loggingSerial.begin(115200);
    } else if (payload == "DEBUG_OFF") {
      debug = false;
      loggingSerial.end();
    } else if (payload == "GET_MAX_POWER") {
      publishMessage(COMMAND_MAIN_TOPIC + COMMAND_RESPONSE_SUB_TOPIC, String(powerThreshold));
    } else if (payload == "GET_INTERVAL") {
      publishMessage(COMMAND_MAIN_TOPIC + COMMAND_RESPONSE_SUB_TOPIC, String(transitionInterval));
    }
  } 
}

void handleLEDGlow() {
  
  unsigned long currentTime = millis();

  if (currentTime > (systemLedBlinkToggleTime + ledBlinkOnSystemOKOffInterval)) {
    systemLedBlinkToggleTime = currentTime;
    turnOnSystemStateLed();
  } else if (currentTime > (systemLedBlinkToggleTime + ledBlinkOnSystemOKOnInterval)) {
    turnOffSystemStateLed();
  }
  
  if (overloadState == NORMAL) {
    turnOnPowerStateLed();
  } else if (overloadState == TRANSITION_TO_OVERLOADED || overloadState == TRANSITION_TO_NORMAL) {
    if (currentTime > (powerLedBlinkToggleTime + ledBlinkOnTransitionInterval)) {
        togglePowerStateLed();
        powerLedBlinkToggleTime = currentTime;
    }  
  } else if (overloadState == OVERLOADED || overloadState == RESTORED) {
    if (currentTime > (powerLedBlinkToggleTime + ledBlinkOnOverloadInterval)) {
        togglePowerStateLed();
        powerLedBlinkToggleTime = currentTime;
    }
  } else if (overloadState == INIT) {
    togglePowerStateLed();
    toggleSystemStateLed();
    delay(500);
    togglePowerStateLed();
    toggleSystemStateLed();
    delay(500);
    togglePowerStateLed();
    toggleSystemStateLed();
    delay(500);
    turnOffPowerStateLed();
    turnOffSystemStateLed();
  }
}

void turnOffPower() {
  digitalWrite(GRID_RELAY_PIN, LOW);
  digitalWrite(INVERTER_RELAY_PIN, LOW);
}

void switchToGrid() {
  digitalWrite(GRID_RELAY_PIN, HIGH);
  digitalWrite(INVERTER_RELAY_PIN, LOW);
}

void switchToInverter() {
  digitalWrite(GRID_RELAY_PIN, LOW);
  digitalWrite(INVERTER_RELAY_PIN, HIGH);
}

void turnOnPowerStateLed() {
  powerStateLedOn = true;
  digitalWrite(POWER_STATE_LED_PIN, LOW);
}

void turnOffPowerStateLed() {
  powerStateLedOn = false;
  digitalWrite(POWER_STATE_LED_PIN, HIGH);
}

void turnOnSystemStateLed() {
  systemStateLedOn = true;
  digitalWrite(SYSTEM_STATE_LED_PIN, LOW);
}

void turnOffSystemStateLed() {
  systemStateLedOn = false;
  digitalWrite(SYSTEM_STATE_LED_PIN, HIGH);
}

void togglePowerStateLed() {
  if (powerStateLedOn) {
     powerStateLedOn = false;
     digitalWrite(POWER_STATE_LED_PIN, HIGH); 
  } else {
     powerStateLedOn = true;
     digitalWrite(POWER_STATE_LED_PIN, LOW);
  }
}

void toggleSystemStateLed() {
  if (systemStateLedOn) {
    systemStateLedOn = false;
    digitalWrite(SYSTEM_STATE_LED_PIN, HIGH);
  } else {
    systemStateLedOn = true;
    digitalWrite(SYSTEM_STATE_LED_PIN, LOW);  
  }
}

void publishMessage(String topic, String value) {
  if (!offlineMode) {
    mqttClient.publish(topic, value);  
  }
}

void doLogln(String message) {
  if (debug) {
    loggingSerial.println(message);  
  }
}

void doLogln(double message) {
  if (debug) {
    loggingSerial.println(message);  
  }
}

void doLogln(byte message) {
  if (debug) {
    loggingSerial.println(message);  
  }
}

void doLog(String message) {
  if (debug) {
    loggingSerial.print(message);  
  }
}
