#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TridentTD_LineNotify.h>

const char* ssid = "CKTECH_2.4GHz";
const char* password = "0817830056";

//const char* ssid = "ck_iot";
//const char* password = "ck@12345678";

#define LINE_TOKEN  "bpltzZwa6c1UUU6oXlWrBxXDUYWBbRZCLkN7ExVQASb" //ใส่ รหัส TOKEN ที่ได้มาจากข้างบน

const char* mqtt_server = "192.168.1.14";
#define MQTT_PORT     1883

//const char* mqtt_server = "183.88.229.11";
//#define MQTT_PORT     9135

const char* mqtt_user = "esp02";
const char* mqtt_pass = "12345";

String data0;

#define DHTPIN 26
#define DHTTYPE    DHT21     // DHT 11
float t, h;

DHT_Unified dht(DHTPIN, DHTTYPE);

uint32_t delayMS;

#define LED_BUILTIN 2

WiFiClient espClient;
PubSubClient client(espClient);

#include <SoftwareSerial.h>
SoftwareSerial mySerial(21, 22); // RX, TX
unsigned int pm1 = 0;
unsigned int pm2_5 = 0;
unsigned int pm10 = 0;

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  60 //300         Time ESP32 will go to sleep (in seconds) */

RTC_DATA_ATTR int bootCount = 0;

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  //----
  //--------------------------------------------------------------
  readPM();
  readDHT();


  //  connectWiFi();
  client.setServer(mqtt_server, MQTT_PORT);
  client.connect("ESP32Client", mqtt_user, mqtt_pass);
  data0 = "temp=" + String(t) + " ,humidity=" + String(h) + " ,pm2_5=" + String(pm2_5) + "\n";
  Serial.println(data0);
  LINE.notify(data0);

  if (client.publish("MQTT/ESP32", data0.c_str()) == true) {
    Serial.println("Success sending");
    digitalWrite(LED_BUILTIN, HIGH);
  } else {
    Serial.println("Fail sending");
    //    client.disconnect();
    reconnectMQTT();
  }
  digitalWrite(LED_BUILTIN, LOW);
}

void setup() {
  Serial.begin(9600);
  while (!Serial) ;
  mySerial.begin(9600);
  dht.begin();
  LINE.setToken(LINE_TOKEN);

  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  dht.humidity().getSensor(&sensor);
  delayMS = sensor.min_delay / 1000;

  pinMode(LED_BUILTIN, OUTPUT);
  connectWiFi();
  client.setServer(mqtt_server, MQTT_PORT);
  client.connect("ESP32Client", mqtt_user, mqtt_pass);
  dht.begin();
  delay(1000); //Take some time to open up the Serial Monitor
  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  /*
    First we configure the wake up source
    We set our ESP32 to wake up every 5 seconds
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) + " Seconds");

  Serial.println("Going to sleep now");
  delay(1000);
  Serial.flush();
  esp_deep_sleep_start();
  Serial.println("This will never be printed");
}

void connectWiFi() {
  WiFi.begin(ssid, password);
  int timeout = 10;
  while (WiFi.status() != WL_CONNECTED && timeout > 0) {
    delay(1000);
    Serial.print(".");
    timeout--;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi connected");
  } else {
    ESP.restart();
  }
}

void reconnectMQTT() {
  int timeout = 10;
  while (!client.connected() && timeout > 0) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
      if (client.publish("MQTT/ESP32_2", data0.c_str()) == true) {
        Serial.println("Success sending");
        digitalWrite(LED_BUILTIN, HIGH);
      } else {
        Serial.println("Fail sending");
        reconnectMQTT();
      }
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 1 second");
      timeout--;
      delay(1000);

      if (timeout <= 0) {
        ESP.restart();
      }
    }
  }
}

void readPM() {
  int index = 0;
  char value;
  char previousValue;

  while (mySerial.available()) {
    value = mySerial.read();
    if ((index == 0 && value != 0x42) || (index == 1 && value != 0x4d)) {
      Serial.println("Cannot find the data header.");
      break;
    }

    if (index == 4 || index == 6 || index == 8 || index == 10 || index == 12 || index == 14) {
      previousValue = value;
    }
    else if (index == 5) {
      pm1 = 256 * previousValue + value;
      //      Serial.print("{ ");
      //      Serial.print("\"pm1\": ");
      //      Serial.print(pm1);
      //      Serial.print(" ug/m3");
      //      Serial.print(", ");
    }
    else if (index == 7) {
      pm2_5 = 256 * previousValue + value;
      //      Serial.print("\"pm2_5\": ");
      //      Serial.print(pm2_5);
      //      Serial.print(" ug/m3");
      //      Serial.print(", ");
    }
    else if (index == 9) {
      pm10 = 256 * previousValue + value;
      //      Serial.print("\"pm10\": ");
      //      Serial.print(pm10);
      //      Serial.print(" ug/m3");
    } else if (index > 15) {
      break;
    }
    index++;
  }
  while (mySerial.available()) mySerial.read();
  Serial.println(" }");
  delay(1000);
}

void readDHT() {
  delay(delayMS);
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
    t = event.temperature;
  }
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    h = event.relative_humidity;
    Serial.println(F("%"));
  }
}

void loop() {

}
