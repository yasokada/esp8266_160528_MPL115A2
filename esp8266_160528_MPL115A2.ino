#include <Wire.h>
#include "lib_160602_calcAltitude.h"
#include "esp8266_160602_udpTxToLogger.h"

/*
 * v0.9 2016 Jun. 26
 *   - esp8266_160602_udpTxToLogger > debug print WiFi strength RSSI
 *   - esp8266_160602_udpTxToLogger > add debug message (Connected at xth try) in WiFi_setup()
 * v0.8 2016 Jun. 26
 *   - modify UdpTxAltitude() to have bettern handling as csv format
 *   - tweak delay in loop() to have approximately 1 second in total
 * v0.7 2016 Jun. 26
 *   - esp8266_160602_udpTxToLogger > increase connection retry from 3 to 6
 *   - esp8266_160602_udpTxToLogger > modify WiFi_setup() to avoid watchdog reset problem
 * v0.6 2016 Jun. 25
 *   - disable watchdog while setup()
 * v0.5 2016 Jun. 2
 *   - send to udpLogger 
 * v0.4 2016 Jun. 2
 *   - average over 100 measurements
 *   - PrintPressureAndAltitude() takes [prs] argument
 *   - add [esp8266_160602_calcAltitude.ino] 
 * v0.3 2016 Jun. 2
 *   - add calcPressure_hPa()
 *   - rename PrintPressure() to PrintPressureAndAltitude()
 *   - add calcAltitude()
 *   - add [kAltitudeCorrection]
 * v0.2 2016 May 29
 *   - fix bug > coefficients calculation
 * v0.1 2016 May 28
 *   - add PrintPressure()
 *   - add ReadPressureAndTemperature()
 *   - add ReadCoefficient()
 */

#define MPL_ADDR (0x60)

static const int kAltitudeCorrection = 0; // TODO: calibration
static const int kAverageCount = 100;

static float s_a0, s_b1, s_b2, s_c12; // 係数データ
unsigned long iPress, iTemp;
static int s_count = 0;

void setup() {
  wdt_disable();

  Serial.begin(115200);
  Serial.println("");

  Wire.begin();
  delay(3000); // msec
  ReadCoefficient();

  // UDP
  WiFi_setup();
  WiFi_printConnectionInfo();

  wdt_enable(WDTO_8S);
}

bool ReadCoefficient()
{
  int len;
  unsigned int hi, lw;

  Wire.beginTransmission(MPL_ADDR);
  Wire.write(0x04);
  len = Wire.endTransmission();
  if (len == 0) {
    len = Wire.requestFrom(MPL_ADDR, 8);
    if (len == 8) {
      hi = Wire.read();
      lw = Wire.read();
      s_a0 = (hi << 8) + lw;
      s_a0 /= 8.0;
      if (hi & 0x80) s_a0 -= 8192.0;
      hi = Wire.read();
      lw = Wire.read();
      s_b1 = (hi << 8) + lw;
      s_b1 /= 8192.0;
      if (hi & 0x80) s_b1 -= 8.0;
      hi = Wire.read();
      lw = Wire.read();
      s_b2 = (hi << 8) + lw;
      s_b2 /= 16384.0;
      if (hi & 0x80) s_b2 -= 4.0;
      hi = Wire.read();
      lw = Wire.read();
      s_c12 = (hi << 8) + lw;
      s_c12 /= 32768.0;
      if (hi & 0x80) s_c12 -= 2.0;
      s_c12 /= 512.0;

      Serial.println(s_a0);
      Serial.println(s_b1);
      Serial.println(s_b2);
      Serial.println(s_c12);      
      return true;
    }
  }
  return false;
}

bool ReadPressureAndTemperature()
{
  int len;
  unsigned int hi, lw;

  // 1. start conversion
  Wire.beginTransmission(MPL_ADDR);
  Wire.write(0x12);
  Wire.write(0x01);
  len = Wire.endTransmission();
  if (len != 0) return false;
  delay(3); // msec

  // 2. get values
  Wire.beginTransmission(MPL_ADDR);
  Wire.write(0x00);
  len = Wire.endTransmission();
  if (len == 0) {
    len = Wire.requestFrom(MPL_ADDR, 4);
    if (len == 4) {
      hi = Wire.read();
      lw = Wire.read();
      iPress = (hi * 256 + lw) / 64;
      hi = Wire.read();
      lw = Wire.read();
      iTemp = ( hi * 256 + lw) / 64;
      return true;      
    }
  }
  return false;
}

void PrintPressureAndAltitude(float prs)
{
  float alt;

//  prs = calcPressure_hPa(iTemp, iPress);
  Serial.print(s_count);
  Serial.print(", Pressure=");
  Serial.print(prs, 3);
  alt = calcAltitude(prs, kAltitudeCorrection);
  Serial.print(", Altitude=");
  Serial.print(alt);
#if 0 // for debug of loop() timing (1 second loop)
  int msec = millis() % 1000;
  Serial.print(",");
  Serial.print(msec);
#endif
  Serial.println();

}

void UdpTxAltitude(float prs)
{
  float alt;
  alt = calcAltitude(prs, kAltitudeCorrection);

  char szbuf[200];
  int pos = 0;
  int whl, frac; // whole and fractional parts

  // 1. pressure
  whl = (int)prs;
  frac = (int)(prs*100) % 100;
  pos = sprintf(&szbuf[pos],"Pressure(kPa)=,%d.%02d", whl, frac);

  // 2. altitude
  whl = (int)alt;
  frac = (int)(alt*100) % 100;
  pos = sprintf(&szbuf[pos],",Altitude(m)=,%d.%02d\r\n", whl, frac);

  WiFi_txMessage(szbuf);  
}

float calcPressure_hPa(int iTemp, int iPress) 
{
  float prs, f0, alt;
  f0 = s_a0 + ( s_b1 + s_c12 * iTemp) * iPress + s_b2 * iTemp;
  prs = f0 * ( (115.0 - 50.0) / 1023.0 ) + 50.0;
  return prs;
}

void loop() {
  double prs = 0.0;
  int cnt = 0;
  
  for(int loop=0; loop < kAverageCount; loop++) {
    if (ReadPressureAndTemperature()) {
      prs += calcPressure_hPa(iTemp, iPress);
      cnt++;
    }
  }
  if (cnt > 0) {
    prs = prs / (float)cnt;
    PrintPressureAndAltitude(prs);
    UdpTxAltitude(prs);
  }

  s_count++;

  delay(591); // msec // for approximately 1 second loop
}

