#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include "SparkFun_MMA8452Q.h"    // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "arduino_secrets.h"


int status = WL_IDLE_STATUS;

WiFiClient client;
PubSubClient mqtt(client);
IPAddress mqtt_server(SECRET_MQTT_SERVER);

Adafruit_BMP280 bmp; // barometer
MMA8452Q accel;      // accelerometer

String chan_config = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String(F("/config"));
String chan_start = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String(F("/run"));

unsigned long tReadoutInterval = 20;    // update readings every 20 milliseconds
unsigned long tStatusInterval  = 10000;  // update status every 5 seconds
unsigned long tReadout = 0;
unsigned long tStatus  = 0;    // timestamp of the last wifi connection status print

bool running = false;

float p0 = 1013.25; // reference pressure, set during start of launch
float maxalt = 0.;  // maximum altitude during run
unsigned long t0 = 0;

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print(F("SSID: "));
  Serial.println(WiFi.SSID());

  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print(F("IP Address: "));
  Serial.println(ip);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print(F("signal strength (RSSI):"));
  Serial.print(rssi);
  Serial.println(F(" dBm"));
}


void reconnect(){
   while (!mqtt.connected()) {
    Serial.print(F("Attempting MQTT connection..."));
    // Attempt to connect
    if (mqtt.connect("esp32logger")) {
      Serial.println(F("connected"));

      // subscribe to config & start channels to recieve instructions
      mqtt.subscribe(chan_config.c_str());
      mqtt.subscribe(chan_start.c_str());
    } else {
      Serial.print(F("failed, rc="));
      Serial.print(mqtt.state());
      Serial.println(F(" try again in 5 seconds"));
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

// do nothing when message recieved
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();


  // Handle configuration payloads
  if (chan_config.equals(topic)){
    Serial.println("Handling configuration message");
    StaticJsonDocument<200> doc;

    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }

    // Fetch values in config payload
    // Most of the time, you can rely on the implicit casts.
    // In other case, you can do doc["time"].as<long>();
    int accel_scale = doc["mma8452q_scale"].as<int>();
    switch ( accel_scale )  {
      case 2:
        Serial.println("Setting accelerometer scale to 2G");
        accel.setScale(SCALE_2G);
        break;
      case 4:
        Serial.println("Setting accelerometer scale to 4G");
        accel.setScale(SCALE_4G);
        break;
      case 8:
        Serial.println("Setting accelerometer scale to 8G");
        accel.setScale(SCALE_8G);
        break;
      default:
        Serial.println("Invalid scale, setting accelerometer scale to default 2G");
        accel.setScale(SCALE_2G);
        break;
    } 

  }


  // Handle messages on the /run topic --> starting & stopping a launch
  if (chan_start.equals(topic)) {
    if ( payload[0] == '1' ) {
      Serial.println("Start the run !");
      running = true;
      t0 = millis();

      // read 10x pressure & average
      p0 = 0.;
      for ( int i = 0; i < 10; i++ )
        p0 += bmp.readPressure();
      p0 /= 10.; // average

      p0 /= 100.; // convert to hPa
    }

    
    if ( payload[0] == '0') {
      Serial.println("Stop the run!");
      running = false;
    }
  }

  return;
}


void setup() {
  Serial.begin(115200);
  Serial.println();
  Wire.begin();

  if (!bmp.begin()) {
    Serial.println(F("Could not find BMP280 sensor, check wiring."));
    while (1) delay(10);
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,    /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X1,    /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X2,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_OFF,     /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_1); /* Standby time. */

  if (! accel.begin() ) {
    Serial.println(F("Could not find MMA8452Q accelerometer, check wiring."));
    while (1) delay(10);
  }
  accel.setScale(SCALE_8G);

  mqtt.setServer(mqtt_server, 1883);
  mqtt.setCallback(callback);

  // attempt to connect to WiFi network:
  while (status != WL_CONNECTED) {
    Serial.print(F("Attempting to connect to SSID: "));
    Serial.println(SECRET_WIFI_SSID);
    // Connect to WPA/WPA2 network. Change this line if using open or WEP network:
    status = WiFi.begin(SECRET_WIFI_SSID, SECRET_WIFI_PASSWORD);

    // wait 15 seconds for connection:
    delay(15000);
  }
  Serial.println(F("Connected to WiFi"));
  printWifiStatus();
}

void loop() {
  
  if (!mqtt.connected()) {
    reconnect();
  } else {
    mqtt.loop();
  }
    
  /* ========================================================================
   *  Readout sensors at interval : 
   *  ==================================================================== */
  if ( running && 
      ( WiFi.status() == WL_CONNECTED ) && 
      ( ( millis() - tReadout ) > tReadoutInterval ) ) {

    float temp = bmp.readTemperature();
    float pres = bmp.readPressure();
    float alt  = bmp.readAltitude(p0); // Adjusted to local forecast!
 
    if ( alt > maxalt ) maxalt = alt;

    float accX = 0.;
    float accY = 0.;
    float accZ = 0.;

    if (accel.available()) {      // Wait for new data from accelerometer
      accX = accel.getCalculatedX();
      accY = accel.getCalculatedY();
      accZ = accel.getCalculatedZ();
    }

    String payload = 
      "{\"timestamp\": " + String(millis()-t0) + 
      ",\"temp_C\": " + String(temp,2) + 
      ",\"pressure_Pa\": " + String(pres,2) +
      ",\"altitude_m\": " + String(alt,2) + 
      ",\"max_altitude_m\": " + String(maxalt,2) + 
      ",\"ax\": " + String(accX,4) +
      ",\"ay\": " + String(accY,4) + 
      ",\"az\": " + String(accZ,4) + "}";
    
    //Serial.print(F("MEASUREMENT: "));
    //Serial.println(payload);

    // transmit
    if ( mqtt.connected() ) {        
      String chan = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String(F("/measurement"));
      if ( ! mqtt.publish( chan.c_str(), payload.c_str()) ) {
        Serial.println(F("Publish failed..."));
      }
    }

    tReadout = millis();
  }
  
  /* ========================================================================
   *  Report status
   *  ==================================================================== */
  if ( ( millis() - tStatus ) > tStatusInterval ) {
    byte ar[6];
    WiFi.macAddress(ar);
    char macStr[18];
    sprintf(macStr, "%02x:%02x:%02x:%02x:%02x:%02x", ar[5], ar[4], ar[3], ar[2], ar[1], ar[0]);
    IPAddress ip = WiFi.localIP();

    // battery voltage is internally wired to pin 35...
    // see : https://www.esp32.com/viewtopic.php?t=10147
    float battV = analogRead(35)/4096.0 * 7.445;

    String payload = \
      "{\"wifiStatus\": " + String(WiFi.status()) +
      ", \"macAddress\": \"" + macStr + "\"" +
      ", \"ipAddress\": \"" + ip[0] + "." + ip[1] + "." + ip[2] + "." + ip[3] + "\""
      ", \"ssid\": \"" + WiFi.SSID() + "\"" +
      ", \"rssi\": " + String(WiFi.RSSI()) +
      ", \"battery_volt\": " + String(battV,2) +
      ", \"mqttStatus\": " + mqtt.connected() + "}";

    Serial.print(F("STATUS: "));
    Serial.println(payload);
    
    // transmit
    if ( mqtt.connected() ) {        
      String chan = SECRET_MQTT_CHANNEL + String("/") + SECRET_DEVUID + String(F("/status"));
      mqtt.publish( chan.c_str(), payload.c_str());
    }

    tStatus = millis();
  }

}