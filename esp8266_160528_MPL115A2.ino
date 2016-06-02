#include <Wire.h>

/*
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

static float s_a0, s_b1, s_b2, s_c12; // 係数データ
unsigned long iPress, iTemp;
static int s_count = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("");

  Wire.begin();
  delay(3000); // msec
  ReadCoefficient();
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

void PrintPressureAndAltitude()
{
  float prs, alt;

  prs = calcPressure_hPa(iTemp, iPress);
  Serial.print(s_count);
  Serial.print(", Pressure=");
  Serial.print(prs);
  alt = calcAltitude(prs, kAltitudeCorrection);
  Serial.print(", Altitude=");
  Serial.print(alt);
  Serial.println();
}

float calcPressure_hPa(int iTemp, int iPress) 
{
  float prs, f0, alt;
  f0 = s_a0 + ( s_b1 + s_c12 * iTemp) * iPress + s_b2 * iTemp;
  prs = f0 * ( (115.0 - 50.0) / 1023.0 ) + 50.0;
  return prs;
}

float calcAltitude(float pressure_hPa, int bias)
{
  float ftmp, alt;
  float pressure_kPa = pressure_hPa * 10;

  ftmp = pow(pressure_kPa / 1013.25, 0.190263);
  alt = 44330.8 * (1.0 - ftmp);
  alt = alt + bias;
  return alt;
}

void loop() {
#if 1 // calc  
  float prs = 0.0;
  int cnt = 0;
  
  for(int loop=0; loop<10; loop++) {
    if (ReadPressureAndTemperature()) {
      prs += calcPressure_hPa(iTemp, iPress);
      cnt++;
    }
  }
  if (cnt > 0) {
    prs = prs / (float)cnt;
    PrintPressureAndAltitude();    
  }
#else
  if (ReadPressureAndTemperature()) {
    PrintPressureAndAltitude();
  }
#endif
  s_count++;
  delay(1000); // msec
}

