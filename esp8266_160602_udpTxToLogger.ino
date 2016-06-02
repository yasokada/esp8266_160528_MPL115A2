#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

/*
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

void WiFi_setup()
{
  WiFi.begin(kWifiSsid, kWifiPass);
  while ( WiFi.status() != WL_CONNECTED) {
    delay(500); // msec  
  }
  wifiUdp.begin(kPortUdp_logger);
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
  wifiUdp.beginPacket( kLoggerIP, kPortUdp_logger );
  wifiUdp.write(srcStr);
  wifiUdp.endPacket();  
}


