
#include <WiFiEspAT.h>
#include "PubSubClient.h"
#include <ArduinoJson.h>

#include "DHT.h"
#include "GravityTDS.h"
#include <OneWire.h>
#include <DallasTemperature.h>

const char* ssid = "xxxx";
const char* password = "xxxxx";

const char* mqtt_server = "10.0.0.10";
int mqtt_server_port = 1883;

//Callback function header
void callback_fct(char* topic, byte* message, unsigned int length);

//WiFiEspClient espClient;
WiFiClient espClient;
int status = WL_IDLE_STATUS;

PubSubClient client(mqtt_server, mqtt_server_port, callback_fct, espClient);
                   
long lastMsg = 0;
long lastMsg2 = 0;
char msg[50];
unsigned int count_MQTT_connection_failed = 0;

const char* mqtt_light_topic_activation = "hydro/lamp/switch";
const char* mqtt_light_state_topic = "hydro/lamp/s";
const char* mqtt_water_spray_activation_topic = "hydro/water_spray/switch";
const char* mqtt_water_spray_state_topic = "hydro/water_spray/s";

// Define digital PINs setup
#define diode_power_pin 7
#define water_pump 8
#define nutriments peristaltic_pump 9
#define PIN_WATER_SPRAY 4
#define PIN_WATER_TEMP 5
#define PIN_DHT11 6
#define PIN_SEED_TRAY_LED 13

//Define analog PINs setup
#define PIN_water_level_value  0 // Plug water level on pin A0
#define PIN_PHOTORESISTOR A2
#define PIN_TDS A3 // Plug TDS on pin A1
#define PIN_PH A5 // Plug PH on pin A5
#define R_PHOTORESISTOR 10000 // Resistance of photoresistor

//Initialize DHT sensor
DHT dht(PIN_DHT11, DHT11);
//Initialize TDS sensor
GravityTDS gravityTDS;
//Initialize OneWire temparature sensor
OneWire oneWire(PIN_WATER_TEMP);
DallasTemperature water_temp_sensor(&oneWire);


#define VREF 5.0      // analog reference voltage(Volt) of the ADC
#define SCOUNT  30           // sum of sample point
int analogBuffer[SCOUNT];    // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0,copyIndex = 0;
float averageVoltage = 0,tdsValue = 0,temperature = 25;

bool growLightsState = false;

// Arduino software reset
void(* resetFunc) (void) = 0;

void callback_fct(char* topic, byte* message, unsigned int length) {
  
  String messageTemp;
 
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }

  // Switch on grow LEDs
  if (strcmp(topic, mqtt_light_topic_activation) == 0 ) {

   Serial.print("Message arrived on topic: ");
    
    if(messageTemp == "true"){
      digitalWrite(PIN_SEED_TRAY_LED, HIGH);
      client.publish(mqtt_light_state_topic, "true");
    } else if(messageTemp == "false") {
      digitalWrite(PIN_SEED_TRAY_LED, LOW);
      client.publish(mqtt_light_state_topic, "false");
    }
  }

  // Switch on water spray
  if (strcmp(topic, mqtt_water_spray_activation_topic) == 0 ) {

    Serial.print("Message arrived on topic: ");
    Serial.print(messageTemp);
    if (messageTemp == "true") {
      digitalWrite(PIN_WATER_SPRAY, HIGH);
      client.publish(mqtt_water_spray_state_topic, "true");
    } else if(messageTemp == "false") {
      digitalWrite(PIN_WATER_SPRAY, LOW);
     client.publish(mqtt_water_spray_state_topic, "false");
    }
   }
}


void setup() {

  Serial.begin(115200);
  while (!Serial);
 
  // Connect to WIFI
  setup_wifi();

  client.setCallback(callback_fct);
  //client.set
  

  // Set the PIN
  pinMode(PIN_SEED_TRAY_LED, OUTPUT);
  digitalWrite(PIN_SEED_TRAY_LED, LOW); // start with off

  pinMode(PIN_WATER_SPRAY, OUTPUT);
  digitalWrite(PIN_WATER_SPRAY, LOW); // start with off
  
  dht.begin();
  
  gravityTDS.setPin(PIN_TDS);
  gravityTDS.setAref(5.0);
  gravityTDS.setAdcRange(1024);
  gravityTDS.begin();
}  

void setup_wifi() {
  
  WiFi.init(&Serial);
  delay(2000);

   // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  
  while (status != WL_CONNECTED) {
    status = WiFi.begin(ssid, password);
    delay(5000);
  }
 
  delay(500);
}


void reconnect() {

  // Loop until we're reconnected
  while (!client.connected()) {

    // Create a random client ID
    String clientId = "ArduinoClient-";
  
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
           
      // Subscribe to different topics
      client.subscribe(mqtt_light_topic_activation);
      client.loop();
      client.subscribe(mqtt_light_state_topic);
      client.loop();
      client.subscribe(mqtt_water_spray_activation_topic);
      client.loop();
      client.subscribe(mqtt_water_spray_state_topic);
      client.loop();
      client.subscribe("hydro/t/room");
      client.loop();
      client.subscribe("hydro/t/seed_tray");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
      
      //Software restart of arduino after few failures
      //TODO add ESP reset
      count_MQTT_connection_failed++;
      if (count_MQTT_connection_failed > 5) {
        resetFunc();
      }
      
    }
  }
}

int getWaterTemperature() {
  water_temp_sensor.requestTemperatures(); 
  return int(water_temp_sensor.getTempCByIndex(0));
}

float getDHTTempature() {
  return dht.readTemperature();
}

// https://cimpleo.com/blog/simple-arduino-ph-meter/ 
float getPHWater() {
  
    int measurings=0;
    for (int i = 0; i < 10; i++)
    {
      measurings += analogRead(PIN_PH);
      delay(10);
    }

    float voltage = 5.0 / 1024.0 * measurings/10.0;
    //return 7 + ((2.5 - voltage) / 0.18);
    return -5.70 * voltage + 21.28;
}

float getDHTHumidity() {
  return dht.readHumidity();
}

int getWaterQuality() {
  //Set actual water temperature
  gravityTDS.setTemperature(getWaterTemperature());
  gravityTDS.update();
  Serial.print(" ; TDS: ");
  Serial.print(gravityTDS.getTdsValue());
  Serial.println(" ppm.");
  return (int) gravityTDS.getTdsValue();
}


float getLuminosityLevel() {
  int raw = analogRead(PIN_PHOTORESISTOR);
  // Conversion rule
  float Vout = float(raw) * (5.0 / float(1023));// Conversion analog to voltage
  float RLDR = (R_PHOTORESISTOR * (5.0 - Vout))/Vout; // Conversion voltage to resistance
  return int(500/(RLDR/1000)); // Conversion resitance to lumen
}

void loop() {

  if(!client.connected()) {
    reconnect();
  }

  client.loop();
  
    // DEBUG PH sensor, should be 2.5 when short circuit
   //float voltage = 5.0 / 1024.0 * analogRead(PIN_PH);
   //Serial.print("ph Voltage= ");
   //Serial.print(voltage);
   //Serial.print(" ; ph = ");
   //Serial.println(-5.70 * voltage + 21.28);
   
   getWaterQuality();
   delay(100);
  
  long now = millis();
  
  if (now - lastMsg > 100000) {
    
    lastMsg = now;

    StaticJsonDocument<100> doc; 

    doc["s"]["water"]["ppm"] = getWaterQuality();
    doc["s"]["water"]["ph"] = getPHWater();
    doc["s"]["water"]["temp"] = getWaterTemperature();
    doc["s"]["lux"] = getLuminosityLevel();
    
    //Serialize to JSON
    char buffer[100];
    size_t n = serializeJson(doc, buffer);

    client.publish("hydro/t/seed_tray", buffer, n);
    
  }

  if (now - lastMsg2 > 110000) {
  
    lastMsg2 = now;
    StaticJsonDocument<100> docRoom; 

    docRoom["r"]["t"] = getDHTTempature();
    docRoom["r"]["h"] = getDHTHumidity();
    
    //Serialize to JSON
    char bufferRoom[100];
    size_t nRoom = serializeJson(docRoom, bufferRoom);

    client.publish("hydro/t/room", bufferRoom, nRoom);
  }
  
  
}
