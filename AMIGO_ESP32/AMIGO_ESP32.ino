//This program only support for ESP32
#define DEBUG 1                                        // 1 DEBUG , 0 PRODUCTION
#include <WiFi.h>                                      // WIFI Library
#include <PubSubClient.h>                              // MQTT Library (Version 3.1.0)
#include <Wire.h>                                      // I2C Library
#include <Adafruit_Sensor.h>                           // Adafruit Sensor Library
#include <Adafruit_BME280.h>                           // Library Temperature and Humidity BME280 (Version 1.1.0)
//#include <Adafruit_ADS1015.h>                         // Analog to I2C Library
#include <BH1750.h>                                    // Light Sensor Library
#include <Arduino.h>                                   // Arduino Library to support SHT31  
#include <Wire.h>                                      // I2C Library
#include <Adafruit_SHT31.h>                            // Library Temperature and Humudity SHT31
#include <strings.h>                                   // String Library
#include <RTClib.h>                                    // RTC Library

// DeepSleep Parameter
#define uS_TO_S_FACTOR 1000000                         // Conversion factor for micro seconds to seconds
#define TIME_TO_SLEEP  600                             // Time ESP32 will go to sleep (in seconds)
// Wifi Parameter
const char* ssid = "jeager";                           // Wifi SSID name
const char* password = "123456";                       // Wifi SSID password

// MQTT Parameter
const char* mqttServer = "35.240.228.215";             // MQTT Server Addres 
const int mqttPort = 1883;                             // MQTT Server Port
const char* mqttUser = "jeager";                       // MQTT Server Username
const char* mqttPassword = "Telkom123";                // MQTT Server Password
const char* topic = "environment_sensor";              // MQTT App Topic
char data[1024];
// JSON Parameter ESP32
char* client_id = "5df3145a4ccd8b1af4e5cc80";                         // Jeager App Client ID                     
char* device_id = "5df3145a4ccd8b1af4e5cc80-1589289102231809146";     // Jeager App Device ID
char  times[10];                                                      // Buffer for times parameter
char  date[20];                                                       // Buffer for date parameter
// Timer Parameter 
const char* ntpServer = "pool.ntp.org";                               // NTP Server Address
const long  gmtOffset_sec = 25200;                                    // GMT offset in second (GMT+1 = 3600)(GMT-1 = -3600)(GMT+2=7200)
const int   daylightOffset_sec = 3600;                                // Daylight offet default is 0 or 3600
float temp,hum,lux,adc,dB;                                            // Inisialisasi sensor variable
// Parameter Pin
int pinAnalog = 34;                                                   // Parameter pin Analog for sound sensor
// Parameter Sensor Suara
const int sampleWindow = 50;                                          // Sample window width in mS (50 mS = 20Hz) for sound sensor
unsigned int sample;                                                  // Sample variable for sound sensor
// Parameter Kalibrasi
float kalibrasi_temp = -(0.5);                                        // Variable for calibration temperature      
float kalibrasi_hum = 0;                                              // Variable for callibration humidity
float kalibrasi_lux = 0;                                              // Variable for callibration Sensor Cahaya
// Define Parameter
//Adafruit_ADS1115 ads;                                               // Use this for the 16-bit version
//Adafruit_ADS1015 ads;                                               // Use thi for the 12-bit version 
RTC_DS3231 rtc;                                                       // Call RTC DS3231 as rtv
Adafruit_BME280 bme;                                                  // Call Adafruit BME280 Sensor as bme
Adafruit_SHT31 sht31 = Adafruit_SHT31();                              // Call Adafruit SHT31 as sht31
BH1750 lightMeter;                                                    // Call BH1750 as lightMeter
#define SEALEVELPRESSURE_HPA (1013.25)                                // Sealevel setup parameter

// MQTT PubSub Connection
WiFiClient espClient;                                                 // Insialisasi MQTT
PubSubClient client(espClient);                                       // Using espClient as PubSubClient

// Check Wifi Connection
void wifiConnection(){
  while (WiFi.status() != WL_CONNECTED) {                             // Waiting for wifi connection
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");
}

// Check MQTT Connection
void mqttConnection(){
    while (!client.connected()) {                                     // Waiting for connection
    Serial.println("Connecting to MQTT...");
    if (client.connect(device_id, mqttUser, mqttPassword )) {         // Set credentials for MQTT
      Serial.println("connected");
    } else {
      Serial.println("failed with state ");
      Serial.print(client.state());                                  // Failed for built connection
      delay(2000);
    }
  }
}

// Get decibel sensor from sensor sound module
float decibelSensor()
{
   unsigned long startMillis= millis();                       // Start of sample window
   float peakToPeak = 0;                                      // peak-to-peak level
   unsigned int signalMax = 0;                                // minimum value
   unsigned int signalMin = 1024;                             // maximum value                                                                               
   while (millis() - startMillis < sampleWindow)              // collect data for 50 mS
   {
      sample = analogRead(pinAnalog);                         // get reading from microphone
      if (sample < 1024)                                      // toss out spurious readings
      {
         if (sample > signalMax)
         {
            signalMax = sample;                               // save just the max levels
         }
         else if (sample < signalMin)
         {
            signalMin = sample;                               // save just the min levels
         }
      }
   } 
   peakToPeak = signalMax - signalMin;                        // max - min = peak-peak amplitude
   float db = map(peakToPeak,20,900,44.5,90);                 // calibrate for deciBels
   return db;
}

// Get Sensor Value from BME Sensor
void sensorValue()
{
//    temp = bme.readTemperature() + kalibrasi_temp;          // Get data sensor suhu 
//    hum = bme.readHumidity() + kalibrasi_hum;               // Get data sensor kelembaban
    temp = sht31.readTemperature() + kalibrasi_temp;          // Get data sensor suhu                                          
    hum = sht31.readHumidity() + kalibrasi_hum;               // Get data sensor kelembaban
    lux = lightMeter.readLightLevel() + kalibrasi_lux;        // Get data sensor cahaya
    dB = decibelSensor();                                     // Get data sensor suara

}

// Get Local Time from NTP Server
void getTime()
{
  struct tm timeinfo;
  int jam,menit,detik,tanggal,bulan,tahun;
  if(!getLocalTime(&timeinfo)){                               // Check Localtime Data
    Serial.println("Failed to obtain time");
    return;
  }
  jam =  timeinfo.tm_hour;                                    // Get data jam
  menit  = timeinfo.tm_min;                                   // Get data menit
  detik  = timeinfo.tm_sec;                                   // Get data detik
  tanggal = timeinfo.tm_mday;                                 // Get data tanggal
  bulan = timeinfo.tm_mon + 1;                                // Get data bulan
  tahun = timeinfo.tm_year +1900;                             // Get data Tahun
  sprintf(times,"%02d:%02d:%02d.000", jam, menit, detik);     // Make Time String Format jam:menit:detik
  sprintf(date,"%04d-%02d-%02d", tahun, bulan, tanggal);      // Make Date String Format tahun-bulan-tanggal
}

// Get Time from RTC device 
void RTC(){
  int jam,menit,detik,tanggal,bulan,tahun;                    // Inisialisasi parameter
  DateTime now = rtc.now();                                   // Get data from RTC device
  jam = now.hour();                                           // Get data jam
  menit = now.minute();                                       // Get data menit
  detik = now.second();                                       // Get data detik
  tanggal = now.day();                                        // Get data tanggal
  bulan = now.month();                                        // Get data bulan
  tahun = now.year();                                         // Get data Tahun
  sprintf(times,"%02d:%02d:%02d.000", jam, menit, detik);     // Make Time String Format jam:menit:detik
  sprintf(date,"%04d-%02d-%02d", tahun, bulan, tanggal);      // Make Date String Format tahun-bulan-tanggal
}

// Convert String to Char
void mqttPublish(String topic, String value) {
    client.publish((char * ) topic.c_str(), (char * ) value.c_str());     // Convert String to Char
}

// Formating data to JSON
String jsonData(String client_id, String device_id, String temperature, String humidity, String intensitas_cahaya, String suara, String times, String date){
    String json = "{\"client_id\": \"" + client_id + "\", \"device_id\": \"" + device_id + "\", \"sensors\": [{\"sensor_name\": \"temperatur\", \"value\": "+ temperature +"},{\"sensor_name\": \"kelembapan\", \"value\": " + humidity +"},{\"sensor_name\": \"intensitas_cahaya\", \"value\": " + intensitas_cahaya +"},{\"sensor_name\": \"suara\", \"value\": " + suara +"}], \"time\": \"" + date  +" " + times +" +0700\"}";
    return json ;
}

// Init Function
void setup() {
  #ifdef DEBUG
    Serial.begin(115200);                                                  // Set Baudrate for Serial Communication.
    WiFi.begin(ssid, password);                                            // Set WiFi username and Password
//    ads.begin();                                                           // Inisialisasi ADS1115 I2C to Analog
    bme.begin();                                                           // Inisialisasi sensor BME280
    sht31.begin(0x44);                                                     // Inisialisasi sensor SHT31
    analogReadResolution(10);                                              // Inisialisasi resolusi Analog
    lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);                      // Inisialisasi sensor lightMeter
    wifiConnection();                                                      // Check WiFi connection
    mqttConnection();                                                      // Check MQTT connection
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);              // Konfigurasi untuk NTP Server
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);         // Set timer to deep sleep
  #else
     WiFi.begin(ssid, password);                                           // Set WiFi username and Password
     client.setServer(mqttServer, mqttPort);                               // Make connection with MQTT server
     client.connect(device_id, mqttUser, mqttPassword)                     // Connect to MQTT server with credetials
     ads.begin();                                                          // Inisialisasi ADS1115 I2C to Analog
     bme.begin();                                                          // inisialisasi sensor BME280
     sht31.begin(0x44);                                                    // Inisialisasi sensor SHT31
     analogReadResolution(10);                                             // Inisialisasi resolusi Analog
     lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE);                     // Inisialisasi sensor lightMeter
     configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);             // Konfigurasi untuk NTP Server
     esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);        // Set timer to deep sleep
  #endif
}

// Main Function
void loop() {
  #ifdef DEBUG                                                                                                    // Mode DEBUG is running
      client.loop();                                                                                              // Make sure connection with MQTT broker                                                          
//    RTC();                                                                                                      // Get Time from RTC
      sensorValue();                                                                                              // Get Sensor Value
      getTime();                                                                                                  // Get Local Time From NTP Server
      String payload = jsonData(client_id,device_id,String(temp),String(hum),String(lux),String(dB),times,date);  // Build payload in JSON Format
      mqttPublish(topic, payload);                                                                                // Publish payload to mqtt server
      Serial.println(payload);                                                                                    // Print Payload
      esp_deep_sleep_start();                                                                                     // ESP deep sleep start
  #else                                                                                                           // Mode DEBUG is not running
    client.loop();                                                                                                // Make sure connection with MQTT broker                                                          
    sensorValue();                                                                                                // Get Sensor Value
    getTime();                                                                                                    // Get Local Time From NTP Server
    String payload = jsonData(client_id,device_id,String(temp),String(hum),String(lux),String(dB),times,date);    // Build payload in JSON Format
    mqttPublish(topic, payload);                                                                                  // Publish payload to mqtt server
    esp_deep_sleep_start();                                                                                       // ESP deep sleep start
  #endif
}
