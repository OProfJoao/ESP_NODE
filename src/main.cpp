//!---------------------       Inclusões de bibliotecas ---------------------
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "PubSubClient.h"
#include "env.h"

//!---------------------       Definição dos pinos      ---------------------
#define FORWARD_DIRECTION_PIN 32   //* Forward Direction
#define BACKWARD_DIRECTION_PIN 33  //* Backward Direction

#define STATUS_LED_R 25
#define STATUS_LED_G 26
#define STATUS_LED_B 27

#define PWM_FORWARD 0
#define PWM_BACKWARD 1

#define PWM_LED_R 2
#define PWM_LED_G 3
#define PWM_LED_B 4

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

//!---------------------       Cabeçalho de Funções     ---------------------

void callback(char* topic, byte* message, unsigned int length);
void connectToMQTT();
void connectToWiFi();
void statusLED(byte status);
void turnOffLEDs();
void handleError();

//!---------------------       Definições de Constantes ---------------------

WiFiClientSecure client;
PubSubClient mqttClient(client);

#define NODE_ID "NODE_1_"

//                 Values set in /include/env.h
const char* mqtt_broker = MQTT_BROKER_CONN;
const char* mqtt_user = MQTT_USER_CONN;
const char* mqtt_password = MQTT_PASSWORD_CONN;
const int mqtt_port = MQTT_PORT_CONN;

const char* wifi_ssid = WIFI_CONN_SSID;
const char* wifi_password = WIFI_CONN_PASSWORD;
//

//!---------------------       Definição dos tópicos        ---------------------
//Sensors
const char* topicPresenceSensor = "ferrorama/station/presence";
const char* topicTemperatureSensor = "ferrorama/station/temperature";
const char* topicHumiditySensor = "ferrorama/station/humidity";
const char* topicLuminanceSensor = "ferrorama/station/luminance";

//Actuators
const char* topicTrainSpeed = "ferrorama/train/speed";


int currentSpeed = 0;

//!---------------------       Loops Principais        ---------------------

void setup() {
  Serial.begin(115200);
  // As the free tier of HiveMQ does not allow generating a CA, it is necessary
  // to disable certificate verification
  client.setInsecure();

  // H-Bridge
  ledcSetup(PWM_FORWARD, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_BACKWARD, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(FORWARD_DIRECTION_PIN, PWM_FORWARD);
  ledcAttachPin(BACKWARD_DIRECTION_PIN, PWM_BACKWARD);
  ledcWrite(PWM_FORWARD, 0);
  ledcWrite(PWM_BACKWARD, 0);
  digitalWrite(FORWARD_DIRECTION_PIN, LOW);
  digitalWrite(BACKWARD_DIRECTION_PIN, LOW);

  // Status LED
  ledcSetup(PWM_LED_R, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_LED_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_LED_B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(STATUS_LED_R, PWM_LED_R);
  ledcAttachPin(STATUS_LED_G, PWM_LED_G);
  ledcAttachPin(STATUS_LED_B, PWM_LED_B);
  turnOffLEDs();
  delay(2000);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    handleError();
    connectToWiFi();
  }
  if (!mqttClient.connected()) {
    handleError();
    connectToMQTT();
  }
  mqttClient.loop();
}

//!---------------------       Funções extras        ---------------------

void setLEDColor(byte r, byte g, byte b) {
  ledcWrite(PWM_LED_R, r);
  ledcWrite(PWM_LED_G, g);
  ledcWrite(PWM_LED_B, b);
}

void connectToWiFi() {
  statusLED(1);
  delay(500);
  WiFi.begin(wifi_ssid, wifi_password);
  Serial.print("Conectando ao WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Conectando ao WiFi: ");
    Serial.println(String(wifi_ssid) + " / " + String(wifi_password));
  }
  else {
    Serial.println("Falha ao conectar ao WiFi!");
  }
}

void connectToMQTT() {
  statusLED(2);
  delay(500);
  mqttClient.setServer(mqtt_broker, mqtt_port);

  while (!mqttClient.connected()) {
    Serial.print("Conectando ao Broker MQTT...");
    String mqtt_id = NODE_ID;
    mqtt_id += String(random(0xffff), HEX);
    if (mqttClient.connect(mqtt_id.c_str(), mqtt_user, mqtt_password)) {
      mqttClient.subscribe(topic);
      mqttClient.setCallback(callback);

      Serial.println("Conectado ao Broker MQTT");
      Serial.print("Inscrito no tópico: ");
      Serial.print(topic);
      turnOffLEDs();
    }
    else {
      Serial.println("Falha ao conectar ao Broker MQTT");
      Serial.print("Erro: ");
      Serial.println(mqttClient.state());
      delay(2000);
      Serial.println("Tentando novamente...");
    }
  }
}

void statusLED(byte status) {
  turnOffLEDs();
  switch (status) {
  case 254:  // Erro (Vermelho)
    setLEDColor(255, 0, 0);
    break;

  case 1:  // Conectando ao Wi-Fi (Amarelo)
    setLEDColor(150, 255, 0);
    break;

  case 2:  // Conectando ao MQTT (Rosa)
    setLEDColor(150, 0, 255);
    break;

  case 3:  // Movendo para frente (Verde)
    setLEDColor(0, 255, 0);
    break;

  case 4:  // Movendo para trás (Ciano)
    setLEDColor(0, 255, 255);
    break;

  default:
    for (byte i = 0; i < 4; i++) {
      setLEDColor(0, 0, 255);  // erro no status (pisca azul)
      delay(100);
      turnOffLEDs();
      delay(100);
    }
    break;
  }
}

void turnOffLEDs() { setLEDColor(0, 0, 0); }

void handleError() {
  for (byte i = 0; i < 4; i++) {
    statusLED(254);
    delay(100);
    turnOffLEDs();
    delay(100);
  }
  turnOffLEDs();
  mqttClient.publish("esp_motor/status", "Valor invalido");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";

  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];
    if (!isDigit(c)) {
      handleError();
      return;
    }
    message += c;
  }

  int speed = message.toInt();
  if (speed > 0 && speed < 255) {
    if (speed != currentSpeed) {
      currentSpeed = speed;
      statusLED(3);
      ledcWrite(PWM_FORWARD, speed);
      Serial.println(String("Velocidade alterada para: ") + speed);
    }
  }
  else if (speed == 0) {
    Serial.println(String("Velocidade alterada para: ") + speed);
    ledcWrite(PWM_FORWARD, speed);
    turnOffLEDs();
  }
  else {
    handleError();
    statusLED(3);
  }
}