#include <WiFi.h>
#include <ArduinoMqttClient.h>

#include <ESP32Servo.h>

#include "secrets.h"
// Secrets.h contains the following:
/* Put your SSID & Password */
//const char* ssid = "XXX";
//const char* password = "XXX";
//const char* mqtt_server = "XXX";
//const char* mqtt_user = "XXX";
//const char* mqtt_pass = "XXX";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const char *subscribeTopic = "halloween/head_angle";
const char *subscribeTopicEyes = "halloween/eyes";
const char *publishTopic = "halloween/head_heartbeat";

const long interval = 1000;
unsigned long previousMillis = 0;

unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // ms

int heartbeat = 0;

// Servo setup
Servo headServo;
const int servoPin = 13;  // Change to your GPIO pin
const int LED1_PIN = 27;   // Port for LED #1
const int LED2_PIN = 14;   // Port for LED #2

// This will store the latest servo angle from MQTT
float targetAngle = 90;  // default midpoint
float smoothedAngle = targetAngle;
const float SMOOTH_ALPHA = 0.1;
const float SMOOTH_ALPHA_NOTARGET = 0.05;
const float MAX_DELTA = 2.5;
const float MAX_DELTA_NOTARGET = 0.75;
int cachedServoVal = -1;

bool targetLocked = true;

#define DEBUG 0

#if DEBUG
#define DEBUG_PRINT(x) Serial.print(x)
#define DEBUG_PRINTLN(x) Serial.println(x)
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
    ;  // wait for serial port to connect. Needed for native USB port only
  }
#endif

  // attempt to connect to WiFi network:
  DEBUG_PRINT("Attempting to connect to WPA SSID: ");
  DEBUG_PRINTLN(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED) {
    // failed, retry
    DEBUG_PRINT(".");
    delay(500);
    if(attempts++ > 300) {
      DEBUG_PRINTLN("Can't connect, restarting...");
      ESP.restart(); // reset and try again
      attempts = 0;
    }
  }

  DEBUG_PRINTLN("You're connected to the network");
  DEBUG_PRINTLN();
  DEBUG_PRINT("Attempting to connect to the MQTT broker: ");
  DEBUG_PRINTLN(mqtt_server);
  mqttClient.setUsernamePassword(mqtt_user, mqtt_pass);
  if (!mqttClient.connect(mqtt_server, mqtt_port)) {
    DEBUG_PRINT("MQTT connection failed! Error code = ");
    DEBUG_PRINTLN(mqttClient.connectError());

    while (1)
      ;
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
  mqttClient.subscribe(subscribeTopicEyes);

  // topics can be unsubscribed using:
  // mqttClient.unsubscribe(topic);

  DEBUG_PRINT("Waiting for messages on topic: ");
  DEBUG_PRINTLN(subscribeTopic);
  DEBUG_PRINTLN();

  // Attach servo
  headServo.attach(servoPin);
  headServo.write(smoothedAngle);  // move to default

  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // Light 'em both to start
  digitalWrite(LED1_PIN, HIGH);
  digitalWrite(LED2_PIN, HIGH);
  targetLocked = true;
}


void loop() {
  // call poll() regularly to allow the library to receive MQTT messages and
  // send MQTT keep alives which avoids being disconnected by the broker
  mqttClient.poll();

  unsigned long currentMillis = millis();

  if ((currentMillis - lastServoUpdate) >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = currentMillis;

    float delta = (targetLocked ? SMOOTH_ALPHA : SMOOTH_ALPHA_NOTARGET) * (targetAngle - smoothedAngle);
    float maxDelta = targetLocked ? MAX_DELTA : MAX_DELTA_NOTARGET;
    if(delta < -maxDelta) { delta = -maxDelta; }
    else if(delta > maxDelta) { delta = maxDelta; }

    smoothedAngle += delta;
    DEBUG_PRINT("targetAngle:");
    DEBUG_PRINT(targetAngle);
    DEBUG_PRINT("\tsmoothedAngle:");
    DEBUG_PRINTLN(smoothedAngle);

    int servoVal = round(smoothedAngle);
    if(servoVal != cachedServoVal) {
      headServo.write(servoVal);
      cachedServoVal = servoVal;
    }
  }

  if (currentMillis - previousMillis >= interval) {
    // save the last time a message was sent
    previousMillis = currentMillis;

    // send message, the Print interface can be used to set the message contents
    mqttClient.beginMessage(publishTopic);
    mqttClient.print(heartbeat);
    mqttClient.endMessage();

    heartbeat++;
  }

}

void onMqttMessage(int messageSize) {
  DEBUG_PRINT("Received message on topic '");
  DEBUG_PRINT(mqttClient.messageTopic());
  DEBUG_PRINT("' (");
  DEBUG_PRINT(messageSize);
  DEBUG_PRINTLN(" bytes):");
  int t = 0;
  String topic = mqttClient.messageTopic();
  if (topic.equals(subscribeTopic)) {
    t = 1;
  } else if (topic.equals(subscribeTopicEyes)) {
    t = 2;
  }

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
    int intval = payload.toInt();  // Safe conversion; returns 0 if invalid

    if (t == 1) {
      int angle = intval;
      // Validate angle range (0–180 typical for servos)
      if (angle >= 0 && angle <= 190) {
        targetAngle = angle;
        DEBUG_PRINT("✅ Parsed angle = ");
        DEBUG_PRINTLN(targetAngle);
      } else {
        DEBUG_PRINT("⚠️  Ignoring out-of-range angle: ");
        DEBUG_PRINTLN(angle);
      }
    } else if(t == 2) {
      if (intval == 0) {
        digitalWrite(LED1_PIN, LOW);
        digitalWrite(LED2_PIN, LOW);
        DEBUG_PRINTLN("👁️ Eyes OFF");
        targetLocked = false;
      } else {
        digitalWrite(LED1_PIN, HIGH);
        digitalWrite(LED2_PIN, HIGH);
        DEBUG_PRINTLN("👁️ Eyes ON");
        targetLocked = true;
      }
    } else {
      DEBUG_PRINTLN("⚠️  Unknown topic, ignoring");
    }
  } else {
    DEBUG_PRINTLN("⚠️  Empty payload, ignoring.");
  }
}
