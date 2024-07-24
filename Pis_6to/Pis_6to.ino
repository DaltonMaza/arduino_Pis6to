//Librería para conexión WiFi
#include <WiFi.h>

//Librería para conexión con el MQTT
#include <PubSubClient.h>

//Libreria para temporizador del sistema
#include <esp_timer.h>

//Libreria para el sensor DHT11
#include "DHTesp.h"

//Libreria para el sensor MQ135
#include <MQ135.h>

// WiFi
const char* ssid = "MazaAlvarado";
const char* password = "Lz0H42*1";

// MQTT
const char* mqtt_server = "mqtt-broker--5i0tb2p.greencoast-d0c064cb.centralus.azurecontainerapps.io";
const char* mqtt_user = "d9a1f8a6-2aa6-485e-900a-e16b75f13901";
// const char* mqtt_user = "1e6xwo45oef"
// const char* mqtt_user = "0ne00D11313"
const char* mqtt_password = "sMB+ReN1zvpyd1b4YVRIhaxyngEp7R3xgAIoTHUs9+0=";
// const char* mqtt_password = "3uwDwvk56B7+AkNksISsrfuRx06BIq0RqmunlgiWOkQ="
const char* temp_topic = "sensor/temperatura";
const char* hum_topic = "sensor/humedad";
const char* co2_topic = "sensor/co2";

//Inicializamos el controlador del sensor DHT11
DHTesp dht;
const int Dhtpin = 16; //Pin para el Sensor DHT11
const unsigned long periodoLlamadaDHT = 2000000;  // 3 segundos en microsegundos

//Inicializamos el controlador del sensor MQ135
const int MQpin = 34; //Pin para el Sensor MQ135
MQ135 mq135_sensor(MQpin);
const unsigned long periodoLlamadaMQ = 2000000;  // 3 segundos en microsegundos
//Valores ideales para el correcto funcionamiento del MQ135
float mqtemperatura = 21.0;
float mqhumedad = 25.0;

WiFiClient espClient;
PubSubClient client(espClient);

esp_timer_handle_t timer1;
esp_timer_handle_t timer2;

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    // if (client.connect("ESP32Client", mqtt_user, mqtt_password)) {
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      esp_timer_start_periodic(timer1, periodoLlamadaDHT);
      esp_timer_start_periodic(timer2, periodoLlamadaMQ);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void IRAM_ATTR recuperarTemperaturaHumedad(void* arg) {
  // Leer la temperatura y la humedad
  float temperatura = dht.getTemperature();
  float humedad = dht.getHumidity();

  // Publicar resultados en MQTT
  char tempStr[8];
  char humStr[8];
  dtostrf(temperatura, 1, 2, tempStr);
  dtostrf(humedad, 1, 2, humStr);
  client.publish(temp_topic, tempStr);
  client.publish(hum_topic, humStr);

  // Imprimir resultados en el Monitor Serial
  Serial.print("Temperatura: ");
  Serial.print(temperatura);
  Serial.println(" °C");
  Serial.print("Humedad: ");
  Serial.print(humedad);
  Serial.println(" %");
  Serial.println();
}

void IRAM_ATTR recuperarCO2(void* arg) {
  // Leer la temperatura y la humedad
  float rzero = mq135_sensor.getRZero();
  float correctedPPM = mq135_sensor.getCorrectedPPM(mqtemperatura, mqhumedad);

  // Publicar resultados en MQTT
  char co2Str[8];
  dtostrf(correctedPPM, 1, 2, co2Str);
  client.publish(co2_topic, co2Str);

  Serial.print("PPM: ");
  Serial.print(correctedPPM);
  Serial.println("ppm");
  Serial.println();
}

void setup() {
  // Iniciar Serial
  Serial.begin(115200);

  // Conexión WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Configurar MQTT
  client.setServer(mqtt_server, 1883);

  //Temporizador 1 (DHT11)
  esp_timer_create_args_t timer_args1 = {
    .callback = &recuperarTemperaturaHumedad,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "timer1"
  };

  // esp_timer_handle_t timer1;
  esp_timer_create(&timer_args1, &timer1);
  // esp_timer_start_periodic(timer1, periodoLlamadaDHT);

  //Temporizador 2 (MQ135)
  esp_timer_create_args_t timer_args2 = {
    .callback = &recuperarCO2,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "timer2"
  };

  // esp_timer_handle_t timer2;
  esp_timer_create(&timer_args2, &timer2);
  // esp_timer_start_periodic(timer2, periodoLlamadaMQ);

  //DHT11
  // Serial2.begin(115200);
  dht.setup(Dhtpin, DHTesp::DHT11);

  //MQ135
  pinMode(MQpin, INPUT);

}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
