#include <math.h>

/*
 * v0.1 2016 Jun. 02
 *   - add calcAltitude()
 */

/*
 * Reference: http://home.anadolu.edu.tr/~mcavcar/common/ISAweb.pdf > Equation (7)
 */

static const float kSurfaceTemperature_K = 288.15; // [K]
static const float kLapseRate_K_m = 0.0065; // [K/m]
static const float kPressureFactor = 0.190263;
static const float kStandardPressure_hPa = 1013.25;

float calcAltitude(float pressure_kPa, int bias)
{
  float ftmp, alt;
  float pressure_hPa = pressure_kPa * 10;

  ftmp = pow(pressure_hPa / kStandardPressure_hPa, kPressureFactor);
  alt = kSurfaceTemperature_K / kLapseRate_K_m * (1.0 - ftmp);
  alt = alt + bias;
  return alt;
}

