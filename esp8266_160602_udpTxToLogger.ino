#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

/*
 * v0.3 2016 Jun. 26
 *   - debug print WiFi strength RSSI
 *   - add debug message (Connected at xth try) in WiFi_setup()
 * v0.2 2016 Jun. 26
 *   - increase connection retry from 6 to 10
 *   - increase connection retry from 3 to 6
 *   - disconnect WiFi when WiFi connection failed
 *   - modify WiFi_setup() to avoid endless while loop
 * v0.1 2016 Jun. 02
 *  - add WiFi_txMessage()
 *  - add WiFi_printConnectionInfo()
 *  - add WiFi_setup()
 */

 /*
  * Connect to UDP Logger [esp8266_160521_udpLoggerWithAccessPoint]
  */

static WiFiUDP wifiUdp;
static const int kPortUdp_logger = 7000;

static const char *kWifiSsid = "esp8266";
static const char *kWifiPass = "12345678";

static const char *kLoggerIP = "192.168.79.2";
static bool s_isWiFiConnected = false;

void WiFi_setup()
{  
  WiFi.begin(kWifiSsid, kWifiPass);

  for(int loop = 0; loop < 10; loop++) {
    if (WiFi.status() == WL_CONNECTED) {
      s_isWiFiConnected = true;

      Serial.print("\nConnected at ");
      Serial.print(loop + 1);
      Serial.print("th try with RSSI:");
      Serial.println(WiFi.RSSI());
      break;
    }
    delay(500); // msec
  }
  if (s_isWiFiConnected) {
    wifiUdp.begin(kPortUdp_logger);  
  } else {
    WiFi.disconnect();
  }
}

void WiFi_printConnectionInfo()
{
  Serial.println("\nLocal IP=");
  Serial.println(WiFi.localIP());
  Serial.println("\nLocal port=");
  Serial.println(wifiUdp.localPort());
}

void WiFi_txMessage(char *srcStr)
{
  if (srcStr == NULL) {
    return;
  }
  if (s_isWiFiConnected == false) {
    return;
  }
  
  wifiUdp.beginPacket( kLoggerIP, kPortUdp_logger );
  wifiUdp.write(srcStr);
  wifiUdp.endPacket();  
}


