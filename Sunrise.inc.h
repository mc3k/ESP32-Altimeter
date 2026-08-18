#ifndef SUNRISE_INC_H
#define SUNRISE_INC_H

#include <math.h>

int sunriseHour = 0;
int sunriseMinute = 0;
int sunsetHour = 0;
int sunsetMinute = 0;
bool isSolarCalculated = false;

int getDayOfYear(int year, int month, int day) {
  int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0)) {
    daysInMonth[1] = 29; // FIX: Assign specifically to February index
  }
  int n = 0;
  for (int i = 0; i < month - 1; i++) {
    n += daysInMonth[i];
  }
  n += day;
  return n;
}

void calculateSolarTimes(double latitude, double longitude, int year, int month, int day) {
  int N = getDayOfYear(year, month, day);
  double lngHour = longitude / 15.0;
  
  // 1. Calculate for Sunrise (using baseline estimate of 6:00 AM)
  double t_rise = N + ((6.0 - lngHour) / 24.0);
  double M_rise = (0.9856 * t_rise) - 3.289;
  double M_rise_rad = M_rise * DEG_TO_RAD;
  double L_rise = M_rise + (1.916 * sin(M_rise_rad)) + (0.020 * sin(2.0 * M_rise_rad)) + 282.634;
  while (L_rise < 0)   L_rise += 360.0;
  while (L_rise >= 360) L_rise -= 360.0;
  
  double L_rise_rad = L_rise * DEG_TO_RAD;
  double RA_rise = atan(0.91764 * tan(L_rise_rad)) * RAD_TO_DEG;
  while (RA_rise < 0)   RA_rise += 360.0;
  while (RA_rise >= 360) RA_rise -= 360.0;
  double Lquad_rise  = floor(L_rise / 90.0) * 90.0;
  double RAquad_rise = floor(RA_rise / 90.0) * 90.0;
  RA_rise = (RA_rise + (Lquad_rise - RAquad_rise)) / 15.0;
  
  double sinDec_rise = 0.39782 * sin(L_rise_rad);
  double cosDec_rise = cos(asin(sinDec_rise));
  double latRad = latitude * DEG_TO_RAD;
  double cosH_rise = (cos(90.833 * DEG_TO_RAD) - (sinDec_rise * sin(latRad))) / (cosDec_rise * cos(latRad));

  // 2. Calculate for Sunset (using baseline estimate of 6:00 PM)
  double t_set = N + ((18.0 - lngHour) / 24.0);
  double M_set = (0.9856 * t_set) - 3.289;
  double M_set_rad = M_set * DEG_TO_RAD;
  double L_set = M_set + (1.916 * sin(M_set_rad)) + (0.020 * sin(2.0 * M_set_rad)) + 282.634;
  while (L_set < 0)   L_set += 360.0;
  while (L_set >= 360) L_set -= 360.0;
  
  double L_set_rad = L_set * DEG_TO_RAD;
  double RA_set = atan(0.91764 * tan(L_set_rad)) * RAD_TO_DEG;
  while (RA_set < 0)   RA_set += 360.0;
  while (RA_set >= 360) RA_set -= 360.0;
  double Lquad_set  = floor(L_set / 90.0) * 90.0;
  double RAquad_set = floor(RA_set / 90.0) * 90.0;
  RA_set = (RA_set + (Lquad_set - RAquad_set)) / 15.0;
  
  double sinDec_set = 0.39782 * sin(L_set_rad);
  double cosDec_set = cos(asin(sinDec_set));
  double cosH_set = (cos(90.833 * DEG_TO_RAD) - (sinDec_set * sin(latRad))) / (cosDec_set * cos(latRad));

  if (cosH_rise > 1 || cosH_rise < -1 || cosH_set > 1 || cosH_set < -1) {
    sunriseHour = 0; sunriseMinute = 0;
    sunsetHour = 0; sunsetMinute = 0;
    return;
  }
  
  // Finalise Sunrise
  double H_rise = (360.0 - acos(cosH_rise) * RAD_TO_DEG) / 15.0;
  double T_rise = H_rise + RA_rise - (0.06571 * t_rise) - 6.622;
  double localSunrise = T_rise - lngHour;
  while (localSunrise < 0)  localSunrise += 24.0;
  while (localSunrise >= 24) localSunrise -= 24.0;
  sunriseHour = (int)localSunrise;
  sunriseMinute = (int)((localSunrise - sunriseHour) * 60.0 + 0.5);
  if (sunriseMinute >= 60) { sunriseMinute -= 60; sunriseHour += 1; }

  // Finalise Sunset
  double H_set = (acos(cosH_set) * RAD_TO_DEG) / 15.0;
  double T_set = H_set + RA_set - (0.06571 * t_set) - 6.622;
  double localSunset = T_set - lngHour;
  while (localSunset < 0)  localSunset += 24.0;
  while (localSunset >= 24) localSunset -= 24.0;
  sunsetHour = (int)localSunset;
  sunsetMinute = (int)((localSunset - sunsetHour) * 60.0 + 0.5);
  if (sunsetMinute >= 60) { sunsetMinute -= 60; sunsetHour += 1; }

  isSolarCalculated = true;
}

int displayDim(int currentHour, int currentMinute) {
  long currentAbsMinutes = (currentHour * 60L) + currentMinute;
  long sunriseAbsMinutes = (sunriseHour * 60L) + sunriseMinute;
  long sunsetAbsMinutes = (sunsetHour * 60L) + sunsetMinute;
  
  if (currentAbsMinutes < sunriseAbsMinutes || currentAbsMinutes >= sunsetAbsMinutes) {
      return 0; 
  } 
  else {
      return 255; 
  }
}

#endif
