/* ESP8266 Config*/

IPAddress ip(192,168,1,23);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

const char* WifiSSID      = "SSID";
const char* WifiPass      = "PASSWORD"; 
uint8_t WifiBSSID[6]      = {0x30,0xD3,0x2D,0x1F,0xF2,0x69};
uint8_t WifiChannel       = 11;
const int sleepTimeS      = 600; // Sleep time in seconds
const int sleepTimeL      = 1800; // Sleep time in seconds
const float tempThreshold  = 1.0; // A change greater than levelThreshold will trigger sleepTimeS

int VeraTempDeviceID = 146;
const char* ESPName = "Vera-DS18B20";
const char* VeraBaseURL = "http://192.168.1.42:3480/";
const char* UpdateURL = "http://192.168.1.254:4080/8266OTA.php";
const char* FWVersion = "06052021";