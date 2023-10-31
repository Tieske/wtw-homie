// Board selected: ESP32 Dev Module"
// Hardware: AZ-Delivery ESP32 DevKitC V2
// CPU: ESP32-WROOM-32

/*
WtW regelt zichzelf op basis van de gemeten CO2 waarde. De richtwaarde kan in de WtW
bediening worden ingesteld, 800 ppm is een goede richtwaarde. De WtW heeft echter zelf
geen CO2 meter.

We ontvangen de CO2 meting van andere sensoren (NetAtmo), en moeten deze doorgeven aan
de WtW. De WtW heeft een ingangssignaal, 0-10v wat gebruikt wordt om de gemeten CO2
waarde door te geven. Hierbij geldt 0 volt == 0 ppm, en 10 volt = 2000 ppm.

De ESP32 heeft echter geen 0-10 volt uitgang, alleen PWM. Daarom is er in de hardware
een PWM --> 0-10 volt module aangesloten.

Dus op basis van de gemeten CO2 waarde, moeten we een PWM signaal uitsturen (0-100%)
wat door de hardware wordt omgezet (in 0-10 volt). Waarna de WtW dit tot zich neemt
als een CO2 ppm meting (van 0-2000 ppm).

Calibratie:
-----------
Er zit wat speling/imprecisie in de componenten. Aangezien het WtW setpoint rond de
800 tot 1000 ppm zit. Moet in dat bereik de waarde juist zijn.
ppm 800  -> 4.0 volt uitgang spanning
ppm 900  -> 4.5 volt uitgang spanning
ppm 1000 -> 5.0 volt uitgang spanning

De potmeter (blauwe blokje met schroefje aan de bovenzijde) op de pwm->0-10v print
kan gebruikt worden voor het calibreren.
*/

// ----------------------------------------------------------------------------------
//   Initialization and globals
// ----------------------------------------------------------------------------------

#include <WiFi.h>
#include <PubSubClient.h>
#include "wtw-homie.h"   // Check this file for setup and secrets !!!!!


WiFiClient espClient;
PubSubClient client(espClient);
String deviceId = TIESKE_DEVICE_ID;


// ----------------------------------------------------------------------------------
//   Logging
// ----------------------------------------------------------------------------------


// Function to set up logging
void setupLogging() {
  Serial.begin(SERIAL_BAUD_RATE);
  Serial.println("Logging initialized");
}

// Function to write log messages to the serial console
void logMessage(const char *format, ...) {
  char logBuffer[128]; // Adjust the buffer size as needed
  va_list args;
  va_start(args, format);
  vsnprintf(logBuffer, sizeof(logBuffer), format, args); // Format the log message
  va_end(args);

  Serial.println(logBuffer);
}


// ----------------------------------------------------------------------------------
// . WiFi connection
// ----------------------------------------------------------------------------------


void wifiSetup()
{
  logMessage("Connecting to WiFi %s", TIESKE_WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(TIESKE_WIFI_SSID, TIESKE_WIFI_PASSWORD);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  logMessage("WiFi connected. IP address: %s", WiFi.localIP().toString());
  delay(1500);
}


// ----------------------------------------------------------------------------------
//   PWM-out control
// ----------------------------------------------------------------------------------


// maximum ppm value we can send out; WtW reads 0-10volt as 0-2000ppm
#define MAX_PPM 2000
// 10 bits => 0 - 1024
#define LEDC_TIMER_RESOLUTION_BITS 10
// Freq 1000; hardware PWM to 0-10 volt module works best at lower frequencies, hence 1000
#define LEDC_BASE_FREQ 1000
#define LEDC_CHANNEL_0 0
#define LED_PIN 12

// based on timer resolution duty range = 0 to "maxrange"
uint32_t maxrange = (1UL << LEDC_TIMER_RESOLUTION_BITS) - 1;
float dutyPerPpm = maxrange / (MAX_PPM * 1.0); // 1.0 to force float-math

// pwm setup
void pwmSetup() {
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_RESOLUTION_BITS);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  logMessage("PWM initialized on pin %d", LED_PIN);
  logMessage("PWM frequency %d hz", LEDC_BASE_FREQ);
  logMessage("PWM precision bits %d, duty range 0-%d", LEDC_TIMER_RESOLUTION_BITS, maxrange);
  logMessage("PWM max ppm %d, duty per ppm: %f", MAX_PPM, dutyPerPpm);
}

// convert co2 ppm measurement into duty cycle
uint32_t ppm2duty(uint32_t ppm)
{
  uint32_t duty = floor(dutyPerPpm * ppm);
  if (duty > maxrange) {
    duty = maxrange;
  }
  return duty;
}

// Sends a ppm value to the output pwm pins
void writeppm(uint32_t ppm)
{
  uint32_t duty = ppm2duty(ppm);  
  ledcWrite(LEDC_CHANNEL_0, duty);
  logMessage("Sent ppm value %d to pwm duty: %d", ppm, duty);
}


// ----------------------------------------------------------------------------------
//   Homie device setup
// ----------------------------------------------------------------------------------


String co2PropertyTopic = "homie/" + deviceId + "/inputs/co2";
String co2PropertySetTopic = "homie/" + deviceId + "/inputs/co2/set";
String lastValue = "800";

// Check if a received payload is a float
bool isFloat(const String& value) {
  char* endPtr;
  float result = strtof(value.c_str(), &endPtr);
  return *endPtr == '\0';
}

// Callback handling subscribed topic values received
void callback(char* topic, byte* payload, unsigned int length) {
  //logMessage("received MQTT data");

  if (length <= 100) { // we do not expect payloads that big, so ignore if > 100
  
    payload[length] = '\0'; // Null-terminate the payload
    String value = String((char*)payload);
    //logMessage("received MQTT data: %s", value);
  
    if (String(topic) == co2PropertySetTopic) {
      if (isFloat(value) && value.toFloat() > 0) {
        lastValue = value;
        writeppm(floor(lastValue.toFloat()));
        client.publish(co2PropertyTopic.c_str(), lastValue.c_str(), true);
      } else {
        logMessage("'%s' received bad value: '%s'", co2PropertySetTopic, value);
      }
    } else {
      logMessage("received value on unknown topic: '%s'", String(topic));
    }
  }
}

void writeTopic(String topic, String value, bool retain) {
  client.publish(topic.c_str(), value.c_str(), retain);
}

// Write the Homie description to the MQTT topics and subscribe to topics
void publishHomieDeviceDescription() {
  writeTopic("homie/" + deviceId + "/$state",                "init", true);
  writeTopic("homie/" + deviceId + "/$extensions",           "none", true);
  writeTopic("homie/" + deviceId + "/$homie",                "4.0.0", true);
  writeTopic("homie/" + deviceId + "/$implementation",       "tieske.wtw.arduino 0.1", true);
  writeTopic("homie/" + deviceId + "/$name",                 "WtW CO2 aansturing", true);
  writeTopic("homie/" + deviceId + "/$nodes",                "inputs", true);
  writeTopic("homie/" + deviceId + "/inputs/$name",          "Input waarden voor CO2 aansturing", true);
  writeTopic("homie/" + deviceId + "/inputs/$properties",    "co2", true);
  writeTopic("homie/" + deviceId + "/inputs/$type",          "input-sensor", true);
  writeTopic("homie/" + deviceId + "/inputs/co2",            lastValue, true);
  writeTopic("homie/" + deviceId + "/inputs/co2/$datatype",  "integer", true);
  writeTopic("homie/" + deviceId + "/inputs/co2/$name",      "CO2 meting input", true);
  writeTopic("homie/" + deviceId + "/inputs/co2/$retained",  "true", true);
  writeTopic("homie/" + deviceId + "/inputs/co2/$settable",  "true", true);
  writeTopic("homie/" + deviceId + "/inputs/co2/$unit",      "ppm", true);

  // subscribe to setter topic
  client.subscribe(co2PropertySetTopic.c_str(), 1);  // QoS level 1; at least once

  // Mark device as ready
  writeTopic("homie/" + deviceId + "/$state", "ready", true);
}


// ----------------------------------------------------------------------------------
//   MQTT setup
// ----------------------------------------------------------------------------------


void setupMQTT() {
  client.setServer(TIESKE_MQTT_SERVER, TIESKE_MQTT_PORT);
  client.setCallback(callback);  
}

void reconnectMQTT() {
  while (!client.connected()) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost, reconnecting...");
      wifiSetup();  // will not return until connected
    }

    if (!client.connect(
          TIESKE_DEVICE_ID, 
          TIESKE_MQTT_USER, 
          TIESKE_MQTT_PASSWORD, 
          ("homie/" + deviceId + "/$state").c_str(),  // LWT: topic
          1,                                        // LWT: QoS
          true,                                     // LWT: retain
          "lost",                                   // LWT: value
          false))                                   // no clean session
      {  
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }

  Serial.println("Connected to MQTT broker");
  publishHomieDeviceDescription();
}


// ----------------------------------------------------------------------------------
//   Main application setup and loop
// ----------------------------------------------------------------------------------


void setup() {
  setupLogging();
  pwmSetup();
  writeppm(floor(lastValue.toFloat()));  // set a base value
  wifiSetup();   // set up and connect
  setupMQTT();   // only set up, connect is automatic in the lmain loop
}

void loop() {
  if (!client.connected()) {
    reconnectMQTT(); 
  }

  client.loop();
}
