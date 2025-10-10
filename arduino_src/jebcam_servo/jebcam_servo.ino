#include <WiFi.h>
#include <ArduinoMqttClient.h>

#include <ESP32Servo.h>

#include "secrets.h"
// Secrets.h contains the following:
/* Put your SSID & Password */
//const char* ssid = "XXX";
//const char* password = "XXX";
//const char* mqtt_server = "XXX";
//const int   mqtt_port = 1883;
//const char* mqtt_user = "XXX";
//const char* mqtt_pass = "XXX";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char *subscribeTopic = "halloween/head_angle";
const char *publishTopic = "halloween/head_heartbeat";

const long interval = 1000;
unsigned long previousMillis = 0;
int heartbeat = 0;

// Servo setup
Servo headServo;
const int servoPin = 13;  // Change to your GPIO pin

// This will store the latest servo angle from MQTT
int targetAngle = 90;  // default midpoint

#define DEBUG 0

#if DEBUG
  #define DEBUG_PRINT(x)    Serial.print(x)
  #define DEBUG_PRINTLN(x)  Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

void setup() {
  delay(3000);
  //Initialize serial and wait for port to open:
#if DEBUG
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
#endif

  // attempt to connect to WiFi network:
  DEBUG_PRINT("Attempting to connect to WPA SSID: ");
  DEBUG_PRINTLN(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    // failed, retry
    DEBUG_PRINT(".");
    delay(500);
  }

  DEBUG_PRINTLN("You're connected to the network");
  DEBUG_PRINTLN();
  DEBUG_PRINT("Attempting to connect to the MQTT broker: ");
  DEBUG_PRINTLN(mqtt_server);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
  if (!mqttClient.connect(mqtt_server, mqtt_port)) {
    DEBUG_PRINT("MQTT connection failed! Error code = ");
    DEBUG_PRINTLN(mqttClient.connectError());

    while (1);
  }

  DEBUG_PRINTLN("You're connected to the MQTT broker!");
  DEBUG_PRINTLN();

  // set the message receive callback
  mqttClient.onMessage(onMqttMessage);

  DEBUG_PRINT("Subscribing to topic: ");
  DEBUG_PRINTLN(subscribeTopic);
  DEBUG_PRINTLN();

  // subscribe to a topic
  mqttClient.subscribe(subscribeTopic);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  DEBUG_PRINT("Waiting for messages on topic: ");
  DEBUG_PRINTLN(subscribeTopic);
  DEBUG_PRINTLN();

  // Attach servo
  headServo.attach(servoPin);
  headServo.write(targetAngle);  // move to default
}


void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(publishTopic);
    mqttClient.print(heartbeat);
    mqttClient.endMessage();

    heartbeat++;
  }

  // Move servo to target angle
  headServo.write(targetAngle);
}

void onMqttMessage(int messageSize) {
  DEBUG_PRINT("Received message on topic '");
  DEBUG_PRINT(mqttClient.messageTopic());
  DEBUG_PRINT("' (");
  DEBUG_PRINT(messageSize);
  DEBUG_PRINTLN(" bytes):");
  // Read the message payload into a string
  String payload;
  while (mqttClient.available()) {
    char c = (char)mqttClient.read();
    payload += c;
  }

  DEBUG_PRINT("Payload: ");
  DEBUG_PRINTLN(payload);

  // Trim whitespace (newline, spaces, etc.)
  payload.trim();

  // Try to convert to integer
  if (payload.length() > 0) {
    int angle = payload.toInt(); // Safe conversion; returns 0 if invalid

    // Validate angle range (0–180 typical for servos)
    if (angle >= 0 && angle <= 190) {
      targetAngle = angle;
      DEBUG_PRINT("✅ Parsed angle = ");
      DEBUG_PRINTLN(targetAngle);
    } else {
      DEBUG_PRINT("⚠️  Ignoring out-of-range angle: ");
      DEBUG_PRINTLN(angle);
    }
  } else {
    DEBUG_PRINTLN("⚠️  Empty payload, ignoring.");
  }
}
