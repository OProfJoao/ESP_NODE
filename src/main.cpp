//!---------------------       Inclusões de bibliotecas ---------------------
 
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>

#include "DHT.h"
#include "Adafruit_Sensor.h"
#include "Ultrasonic.h"
#include "PubSubClient.h"
#include "env.h"

//!---------------------       Definição dos pinos      ---------------------

#define PWM_FREQ 500
#define PWM_RESOLUTION 8

#define FORWARD_DIRECTION_PIN 32
#define BACKWARD_DIRECTION_PIN 33 

#define PWM_FORWARD 0
#define PWM_BACKWARD 1



#define STATUS_LED_R_PIN 25
#define STATUS_LED_G_PIN 26
#define STATUS_LED_B_PIN 27

#define PWM_LED_R 2
#define PWM_LED_G 3
#define PWM_LED_B 4



//TODO: Configurar pinos corretos
#define LDR_PIN 10      
#define DHT_PIN 11
#define ULTRA_ECHO 0
#define ULTRA_TRIGG 1

#define LEDPIN 30


//!---------------------       Definições de variáveis     ---------------------

//ultrasonic
bool detected = false;
unsigned long lastDetection = 0;

//dht
unsigned long lastReading = 0;


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

Ultrasonic ultrasonic(ULTRA_TRIGG,ULTRA_ECHO);
DHT dht(DHT_PIN, DHT11);

String NODE_ID = "NODE_1_";


//!---------------------       Definição dos tópicos        ---------------------

//Publish
const char* topicPresenceSensor = "ferrorama/station/presence";
const char* topicTemperatureSensor = "ferrorama/station/temperature";
const char* topicHumiditySensor = "ferrorama/station/humidity";
const char* topicLuminanceSensor = "ferrorama/station/luminanceStatus";
const char* topicTrainSpeed = "ferrorama/train/speed";


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

  //DHT11
  dht.begin();

  // Status LED
  ledcSetup(PWM_LED_R, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_LED_G, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_LED_B, PWM_FREQ, PWM_RESOLUTION);

  ledcAttachPin(STATUS_LED_R_PIN, PWM_LED_R);
  ledcAttachPin(STATUS_LED_G_PIN, PWM_LED_G);
  ledcAttachPin(STATUS_LED_B_PIN, PWM_LED_B);
  turnOffLEDs();



  nodeIlumination(0);
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

  unsigned long currentTime = millis();

  //TODO: Read luminance sensor data and publish it to the luminance topic
  byte luminanceValue = map(analogRead(LDR_PIN),0,4095,0,100);
  if (luminanceValue < 80){
    mqttClient.publish(topicLuminanceSensor,String("1").c_str());
  }else{
    mqttClient.publish(topicLuminanceSensor, String("0").c_str());
  }

  //TODO: Read temperatura/humidity sensor data and publish it to the temperatura/humidity topic 
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  if ((currentTime - lastReading > 2000) && (!isnan(temperature) || !isnan(humidity))) {
    lastReading = currentTime;
    mqttClient.publish(topicTemperatureSensor, String(temperature).c_str());
    mqttClient.publish(topicHumiditySensor, String(humidity).c_str());
  }

  //TODO: Read distance sensor data and publish it to the presence topic
  long microsec = ultrasonic.timing();
  float distance = ultrasonic.convert(microsec,Ultrasonic::CM);
  

  if(distance < 10 && detected == false && (currentTime - lastDetection >= 3000)){
    mqttClient.publish(topicPresenceSensor, String("1").c_str());
    detected = true;
    lastDetection = currentTime;
  }
  if (distance > 10 && detected == true && (currentTime - lastDetection >= 3000)) {
    detected = false;
    lastDetection = currentTime;
  }


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
  WiFi.begin(WIFI_CONN_SSID, WIFI_CONN_PASSWORD);
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
  mqttClient.setServer(MQTT_BROKER_CONN, MQTT_PORT_CONN);

  while (!mqttClient.connected()) {
    Serial.print("Conectando ao Broker MQTT...");
    String mqtt_id = NODE_ID;
    mqtt_id += String(random(0xffff), HEX);
    if (mqttClient.connect(mqtt_id.c_str(), MQTT_USER_CONN, MQTT_PASSWORD_CONN)) {
      Serial.println("Conectado ao Broker MQTT");


      mqttClient.subscribe(topicLuminanceSensor);
      mqttClient.setCallback(callback);
      Serial.print("Inscrito no tópico: ");
      Serial.print(topicLuminanceSensor);
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
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  bool error = false;
  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];
    if (!isDigit(c)) {
      handleError();
      error = true;
      return;
    }
    message += c;
  }

  if (!error) {
    if (message == "1") {
      nodeIlumination(1); //Acende os leds
    }
    else if (message == "0") {
      nodeIlumination(0); //Apaga os leds
    }
    else {
      handleError();
      statusLED(3);
    }
  }
}

void nodeIlumination(bool status) {
  digitalWrite(LEDPIN, status);
}