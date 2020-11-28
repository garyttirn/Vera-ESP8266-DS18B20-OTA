
/**
 * ESP8266 12F and DS18B20 Temperature and Humidity Sersor with Vera integration. 
 * 
 * Partly based on 
 *  https://github.com/Ispep/Hemautomation/blob/master/ESP8266/ESP8266%2012E%20TemperaturSensor/ESP8266-12E-TemperaturSensor.ino
 *
 * To be powered by 3xAA or 3xAAA NiMH Cells
 **/

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <ESP8266httpUpdate.h>

// One Wire for DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>
#define ONE_WIRE_BUS 4 //GPIO4

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

// The ESP8266 RTC memory is arranged into blocks of 4 bytes. The access methods read and write 4 bytes at a time,
// so the RTC data structure should be padded to a 4-byte multiple.
struct {
  uint32_t crc32=0;   // 4 bytes
  uint8_t channel=0;  // 1 byte,   5 in total
  uint8_t bssid[6]={0}; // 6 bytes, 11 in total
  uint8_t padding=0;  // 1 byte,  12 in total
} rtcData;

//Read battery voltage
ADC_MODE(ADC_VCC);

#include "config.h"

String ProgramVersion  = "0.1";

float glb_temp=0,glb_batterylevel=0;

WiFiClient client;
HTTPClient http;

uint32_t calculateCRC32( const uint8_t *data, size_t length ) {
  uint32_t crc = 0xffffffff;
  while( length-- ) {
    uint8_t c = *data++;
    for( uint32_t i = 0x80; i > 0; i >>= 1 ) {
      bool bit = crc & 0x80000000;
      if( c & i ) {
        bit = !bit;
      }

      crc <<= 1;
      if( bit ) {
        crc ^= 0x04c11db7;
      }
    }
  }

  return crc;
}

void GoToDeepSleep(int SleepDelay){
  Serial.print("Sleeping " + String(SleepDelay) + " seconds\n");
  ESP.deepSleepInstant(SleepDelay * 1000000,WAKE_RF_DISABLED);
}

void update_started() {
  Serial.println("HTTP update process started");
}

void update_finished() {
  Serial.println("HTTP update process finished");
}

void update_progress(int cur, int total) {
  Serial.printf("HTTP update process at %d of %d bytes...\n", cur, total);
}

void update_error(int err) {
  Serial.printf("HTTP update fatal error code %d\n", err);
}

void checkForUpdates() {
    ESPhttpUpdate.setLedPin(LED_BUILTIN, LOW);

    // Add optional callback notifiers
    ESPhttpUpdate.onStart(update_started);
    ESPhttpUpdate.onEnd(update_finished);
    ESPhttpUpdate.onProgress(update_progress);
    ESPhttpUpdate.onError(update_error);

    Serial.print("\nChecking for update from " + String(UpdateURL) + ". Current FW version " + String(FWVersion) + "\n");
    t_httpUpdate_return ret = ESPhttpUpdate.update(client, String(UpdateURL),String(FWVersion));

    switch (ret) {
      case HTTP_UPDATE_FAILED:
        Serial.printf("HTTP_UPDATE_FAILD Error (%d): %s\n", ESPhttpUpdate.getLastError(), ESPhttpUpdate.getLastErrorString().c_str());
        break;

      case HTTP_UPDATE_NO_UPDATES:
        Serial.println("HTTP_UPDATE_NO_UPDATES");
        break;

      case HTTP_UPDATE_OK:
        Serial.println("HTTP_UPDATE_OK");
        delay(5000);
        ESP.restart();
        break;
    }
}

int connect_wifi (){
  int retries = 0;
  int wifiStatus = WiFi.status();

  WiFi.forceSleepWake();
  delay(1);
  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.config(ip, gateway, subnet);

  // Try to read WiFi settings from RTC memory
  bool rtcValid = false;
  if( ESP.rtcUserMemoryRead( 0, (uint32_t*)&rtcData, sizeof( rtcData ) ) ) {
    // Calculate the CRC of what we just read from RTC memory, but skip the first 4 bytes as that's the checksum itself.
    uint32_t crc = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
    if( crc == rtcData.crc32 ) {
      rtcValid = true;
    }
  }

  if( rtcValid ) {
    // The RTC data was good, make a quick connection
    Serial.printf("[WIFI] ... quick connect, status: %d\n", wifiStatus);
    WiFi.begin( WifiSSID, WifiPass, rtcData.channel, rtcData.bssid, true );
  }
  else {
    // The RTC data was not valid, so make a regular connection
    Serial.printf("[WIFI] ... normal connect, status: %d\n", wifiStatus);
    WiFi.begin( WifiSSID, WifiPass );
  }

  while( wifiStatus != WL_CONNECTED ) {
    retries++;
     if( retries == 100 ) {
       // Quick connect is not working, reset WiFi and try regular connection
       rtcValid = false;
       WiFi.disconnect();
       delay( 10 );
       WiFi.forceSleepBegin();
       delay( 10 );
       WiFi.forceSleepWake();
       delay( 10 );
       WiFi.begin( WifiSSID, WifiPass );
    }
    if( retries >= 600 ) {
      // Giving up after 30 seconds and going back to sleep
      WiFi.disconnect( true );
      delay( 1 );
      WiFi.mode( WIFI_OFF );
      GoToDeepSleep(sleepTimeS);
      return 1; // Not expecting this to be called, the previous call will never return.
    }
    delay( 50 );
    wifiStatus = WiFi.status();
  }

  if( rtcValid == false ) {
    // Write current connection info back to RTC
   rtcData.channel = WiFi.channel();
   memcpy( rtcData.bssid, WiFi.BSSID(), 6 ); // Copy 6 bytes of BSSID (AP's MAC address)
   rtcData.crc32 = calculateCRC32( ((uint8_t*)&rtcData) + 4, sizeof( rtcData ) - 4 );
   ESP.rtcUserMemoryWrite( 0, (uint32_t*)&rtcData, sizeof( rtcData ) );

   //Check for updates
   checkForUpdates();
 }

return wifiStatus;
}

void setup() {
  //switch radio off to save energy
  WiFi.mode(WIFI_OFF);
  WiFi.forceSleepBegin();
  
  //Disable WiFi status LED to save battery
  wifi_status_led_uninstall();
  
  delay(100);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  
   //Initialize sensor
  DS18B20.begin();

  Serial.print("\n" + String(ESPName) + " started\n");
}

int ReadSensor() {
  int readloops=0;
  
  Serial.print("\nRequesting DS18B20 temperature...\n");

  do {
    DS18B20.requestTemperatures(); 
    glb_temp = DS18B20.getTempCByIndex(0);
    delay(100);

    readloops++;
  } while (glb_temp == 85.0 || glb_temp == (-127.0) || readloops <=5);


  // Check if any reads failed and exit early (to try again).
  if (isnan(glb_temp)) {
    Serial.println("Failed to read from DS18B20 sensor after" + String(readloops) + "loops");
    return 1;
  }

  Serial.println("Temp: " + String(glb_temp));
  return 0;
}

float getBatteryStatus(){
  glb_batterylevel=ESP.getVcc();
    
  Serial.println("Battery VCC: " + String(glb_batterylevel*0.001)+ "V");

  //Calculate estimated percentage
  if (glb_batterylevel >= 3400) {
    glb_batterylevel = 100;
    }
  else if ( glb_batterylevel <= 3400 || glb_batterylevel >= 3300 ) {
    glb_batterylevel = 50;
    }
  else {
    glb_batterylevel = 10;
    }

  Serial.println("Battery level: " + String(glb_batterylevel) + "%");
    
  return 0; 
}

int GetHttpURL(String MyURL){
  Serial.print("[HTTP] begin...\n");

  http.begin(client,String(VeraBaseURL) + MyURL);
    
  int httpCode = http.GET();

   if (httpCode > 0) {
     Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    
     if (httpCode == HTTP_CODE_OK) {
        http.writeToStream(&Serial);
        }
  }
  else {
    Serial.printf("[HTTP] GET... failed, error: %s\n", http.errorToString(httpCode).c_str());
    }
 
 return httpCode;
}

void loop() {

  int SensorStatus = ReadSensor(); 

  if (SensorStatus > 0) {
    GoToDeepSleep(sleepTimeS);
  }

  int BatteryStatus = getBatteryStatus();
  
  // Connect WiFi
  connect_wifi();

  http.setReuse(true);

  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:upnp-org:serviceId:TemperatureSensor1&Variable=CurrentTemperature&Value=" + String(glb_temp));
  GetHttpURL("data_request?id=variableset&DeviceNum=" + String(VeraTempDeviceID) + "&serviceId=urn:micasaverde-com:serviceId:HaDevice1&Variable=BatteryLevel&Value=" + String(glb_batterylevel));

  http.end();

  GoToDeepSleep(sleepTimeS);
}
//EOF