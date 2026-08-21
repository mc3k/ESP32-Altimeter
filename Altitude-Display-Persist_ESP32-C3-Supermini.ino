/**
 * @file Altitude-Display_ESP32-C3-Supermini.ino
 * @brief ESP32-C3 Supermini GPS Altimeter with Altitude History Graph & Slope Tracking
 * @author Marty Childs <www.childs.be>
 * @date 2026-08-18
 * @version 1.0.44.persist
 * @license MIT
 * 
 * @description
 * An ESP32 powered device that tracks real-time altitude, slope gradient (%) and heading 
 * using an SSD1306 128x64 OLED display over I2C and a standard 9600-baud GPS module.
 * Features automated daily resets and persistent graph history using internal storage.
 *
 * @warning
 * EXPERIMENTAL - constantly writes to NVS flash which is only rated to 100,000 operations
 * which will permanently damage your device
 * 
 * @hardware
 * - Microcontroller: ESP32-C3 Supermini
 * - Display: SSD1306 128x64 I2C OLED Display
 * - GPS Module: NEO-6M / BN-220 or compatible (9600 Baud default)
 * 
 * @pinout
 *   [ESP32-C3 Supermini] <--> [Peripherals]
 *   GPIO 6 (SDA)         <--> OLED SDA
 *   GPIO 7 (SCL)         <--> OLED SCL
 *   GPIO 20 (RX)         <--> GPS TX
 *   GPIO 21 (TX)         <--> GPS RX
 *   GND                  <--> Common Ground
 *   5V / VBUS            <--> GPS VCC (Prevents Brownouts)
 *   3V3                  <--> OLED VCC
 * 
 * @dependencies
 * - U8g2 by olikraus (OLED driver library)
 * - TinyGPS++ by mikalhart (GPS parsing library)
 * - Preferences (Built-in ESP32 core storage library)
 * - Sunset by Peter Buelow (Sunrise/set from GPS)
 */


#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Preferences.h>
#include <TinyGPS++.h>
#include <sunset.h>
#include "Compass_30.inc.h"

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

#define I2C_SDA 6 // C3 Supermini
#define I2C_SCL 7

#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
#define GPS_BAUDRATE 9600

const int graphWidth = 80;
const int altPtsCnt = 15;
const bool resetOnBoot = false;
unsigned long logInterval = 60000;

struct GPSPoint { double lon; double lat; double dist; double alt; };
GPSPoint path[altPtsCnt];

int signalHistory[graphWidth];
int graphPointsCount = 0;
unsigned long lastLogTime = 0;
unsigned long lastAltTime = 0;
unsigned long lastDisplayTime = 0;
uint32_t lastStoredDate = 0;
double slope = 0;

Preferences preferences;
TinyGPSPlus gps;
SunSet sun;

// Brightness parameters (0 to 255)
const uint8_t DAY_BRIGHTNESS = 255;
const uint8_t NIGHT_BRIGHTNESS = 15; 
uint8_t currentBrightness = 15;

void historyGraph(int x, int y, int gWidth=graphWidth, int gHeight=30) {
    if (graphPointsCount == 0) return; 

    int maxAltitude = -32768; 
    int minAltitude = 32767;  

    for (int i = 0; i < graphPointsCount; i++) {
      if (signalHistory[i] > maxAltitude) maxAltitude = signalHistory[i];
      if (signalHistory[i] < minAltitude) minAltitude = signalHistory[i];
    }

    int currentRange = maxAltitude - minAltitude;
    if (currentRange < 100) {
      int paddingNeeded = 100 - currentRange;
      maxAltitude += paddingNeeded; 
    }

    float pointWidth = (float)graphWidth / (float)graphPointsCount;

    for (int i = 0; i < graphPointsCount; i++) {
      int dataHeight = map(signalHistory[i], minAltitude, maxAltitude, 1, gHeight);
      int yStart = y + gHeight - dataHeight;
      
      int startX = x + int(i * pointWidth);
      int endX = x + int((i + 1) * pointWidth);
      int currentBarWidth = max(1, endX - startX);

      u8g2.drawBox(startX, yStart, currentBarWidth, dataHeight);
    }
    
    if (gps.altitude.meters() < 1000) {
      u8g2.setFont(u8g2_font_helvB10_tr);
      u8g2.setCursor(41, 34); 
      u8g2.print(currentRange);
      
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t); 
      u8g2.drawGlyph(29, 34, 84);
      
      u8g2.setFont(u8g2_font_helvB08_tr);
    }
}

double gradientCalc() {
    double sumX = 0, sumY = 0;
    for (int i = 0; i < altPtsCnt; i++) {
      sumX += path[i].dist;
      sumY += path[i].alt;  
    }
    double meanX = sumX / altPtsCnt;
    double meanY = sumY / altPtsCnt;
    double num = 0;
    double den = 0;
    for (int i = 0; i < altPtsCnt; i++) {
      double xDiff = path[i].dist - meanX;
      double yDiff = path[i].alt - meanY;
      num += xDiff * yDiff;
      den += xDiff * xDiff;  
    }
    if (den <= 0.001) return 0; 
    double m = num / den;
    if (gps.speed.mps() < 1 
        || abs(path[0].alt - path[altPtsCnt-1].alt) < 0.5 
        || path[altPtsCnt-1].dist < 25 
        || (path[0].lat == 0 && path[0].lon == 0)) {
      return 0;
    }
    return m * 100;
}

void drawLockStatus(int x, int y, int sat, double hdop) {
  u8g2.setFont(u8g2_font_04b_03_tr);
  
  if (sat >= 4 && hdop < 5.0) {
    u8g2.drawRBox(x, y, 13, 9, 1);
    u8g2.setDrawColor(0); 
    u8g2.drawStr(x + 2, y + 7, "3D");
    u8g2.setDrawColor(1); 
  } 
  else if (sat >= 3) {
    u8g2.drawRFrame(x, y, 13, 9, 1);
    u8g2.drawStr(x + 2, y + 7, "2D");
  } 
  else {
    u8g2.drawRFrame(x, y, 13, 9, 1);
    u8g2.drawStr(x + 2, y + 7, "NO");
  }
}

void drawSigBars(int x, int y, int sat, double hdop) {
  int numBars = 0;
  if (hdop <= 1) numBars = 5;
  else if (hdop <= 2.5) numBars = 4;
  else if (hdop <= 5) numBars = 3;
  else if (hdop <= 10) numBars = 2;
  else numBars = 1;

  int fillBars = 0;
  if (sat > 12) fillBars = 5;
  else if (sat > 7) fillBars = 4;
  else if (sat > 6) fillBars = 3;
  else if (sat > 4) fillBars = 2;
  else if (sat > 0) fillBars = 1;

  for (int i = 0; i < 5; i++) {
    int barHeight = (i + 1) * 4;
    int barX = x + (i * 5);
    int barY = (y + 20) - barHeight;

    if (i < fillBars) {
      u8g2.drawRBox(barX, barY, 4, barHeight, 1);
    }
    if (i < numBars) {
      u8g2.drawRFrame(barX, barY, 4, barHeight, 1);
    }
  }
}

void altDisplay(int x, int y,float altValue) {
    u8g2.setFont(u8g2_font_logisoso28_tr);
    u8g2.setCursor(x - u8g2.getUTF8Width(String(altValue, 0).c_str()) - 10, y);
    u8g2.print(altValue, 0);

    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(x - 8, y - 22, "m");
}

void failDisplay(int x, int y,float failValue) {
    u8g2.setFont(u8g2_font_04b_03_tr);
    u8g2.setFontDirection(1);
    u8g2.setCursor(x - 6, y - 28);
    u8g2.print("ERRORS");
    u8g2.setFontDirection(0);
    u8g2.setFont(u8g2_font_logisoso28_tr);
    u8g2.drawStr(x - u8g2.getUTF8Width(String(failValue, 0).c_str()) - 10, y, String(failValue, 0).c_str());
}

void slopeDisplay(int x, int y, float slopeValue) {
    int arrowGlyph;
    if ((int)slopeValue == 0)   {  arrowGlyph = 78;   }
    else if (slopeValue < 0.0f) {  arrowGlyph = 76;   }
    else                        {  arrowGlyph = 79;   }

    String displayStr = String((int)abs(slopeValue));
    u8g2.setFont(u8g2_font_logisoso28_tf);
    int textWidth = u8g2.getUTF8Width(displayStr.c_str());
    u8g2.setCursor(x - textWidth - 10, y); 
    u8g2.print(displayStr);

    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.setCursor(x - 7, y -28+6); 
    u8g2.print("%");

    if (slopeValue >= 10 || slopeValue <= -10) {
      u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t);
      u8g2.drawGlyph(x - 9, y -13+6, arrowGlyph);
    }
    else {
      u8g2.setFont(u8g2_font_open_iconic_arrow_2x_t);
      u8g2.drawGlyph(x - textWidth - 26, y - 6, arrowGlyph);
    }
    u8g2.setFont(u8g2_font_helvB08_tr);
}

void clearHistoryProfile(uint32_t newDateReference) {
    graphPointsCount = 0;
    memset(signalHistory, 0, sizeof(signalHistory));
    lastStoredDate = newDateReference;

    preferences.begin("gps_history", false);
    preferences.remove("pt_count");
    preferences.remove("history_arr");
    preferences.putUInt("saved_date", lastStoredDate);
    preferences.end();
    
    Serial.print("New day detected! History cleared for date: ");
    Serial.println(lastStoredDate);
}

void displayDim() {
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) {
	sun.setPosition(gps.location.lat(), gps.location.lng(), 0.0);
    sun.setCurrentDate(gps.date.year(), gps.date.month(), gps.date.day());

	int sunriseMin = static_cast<int>(sun.calcSunrise());
    int sunsetMin = static_cast<int>(sun.calcSunset());
    int currentMin = (gps.time.hour() * 60) + gps.time.minute();
    bool isDaytime = (currentMin >= sunriseMin && currentMin < sunsetMin);
    uint8_t targetBrightness = isDaytime ? DAY_BRIGHTNESS : NIGHT_BRIGHTNESS;

	if (currentBrightness != targetBrightness) {
      currentBrightness = targetBrightness;
      u8g2.setContrast(currentBrightness); 
    }
  }
  else {
    u8g2.setContrast(NIGHT_BRIGHTNESS); 
  }
}

void setup(void) {
    Serial.begin(115200);
    delay(500);
    Serial.println(__FILE__); Serial.print(__DATE__); Serial.print(F("\t")); Serial.println(__TIME__);
    
    Wire.begin(I2C_SDA, I2C_SCL);
    Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    u8g2.begin();
    u8g2.setBusClock(400000);
	u8g2.setContrast(currentBrightness);

    // Restore altitude from memory
    if (resetOnBoot) {
      Serial.println("Boot flag active: Purging old data profiles.");
      clearHistoryProfile(0); 
    } else {
      // Standard restore payload behavior
      preferences.begin("gps_history", false);
      lastStoredDate = preferences.getUInt("saved_date", 0);
      graphPointsCount = preferences.getInt("pt_count", 0);
      if (graphPointsCount > 0) {
        preferences.getBytes("history_arr", signalHistory, sizeof(signalHistory));
      }
      preferences.end();
    }
}

void loop(void) {

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }
 
  if (gps.location.isValid()) { 
    
    if (gps.date.isValid() && gps.date.value() > 0) {
      uint32_t currentGpsDate = gps.date.value();
      
      // If no day key exists in flash, establish this as our base target date tracking line
      if (lastStoredDate == 0) {
        lastStoredDate = currentGpsDate;
        preferences.begin("gps_history", false);
        preferences.putUInt("saved_date", lastStoredDate);
        preferences.end();
      } 
      // Clear altitude history if new day
      else if (currentGpsDate != lastStoredDate) {
        clearHistoryProfile(currentGpsDate);
      }
    }

    // Log new alititude for gradient calc
    if ((millis() - lastAltTime) >= 1000) {
      if (path[altPtsCnt - 1].lat == 0 && path[altPtsCnt - 1].lon == 0) {
        for (int i = 0; i < altPtsCnt; i++) {
          path[i].lat = gps.location.lat();
          path[i].lon = gps.location.lng();
          path[i].alt = gps.altitude.meters();
          path[i].dist = 0;
        }
      } else {
        for (int i = 0; i < altPtsCnt - 1; i++) {
          path[i] = path[i + 1];
        }
        path[altPtsCnt - 1].lat = gps.location.lat();
        path[altPtsCnt - 1].lon = gps.location.lng();
        path[altPtsCnt - 1].alt = gps.altitude.meters();
      }

      path[0].dist = 0;
      for (int i = 1; i < altPtsCnt; i++) {
        double segDist = TinyGPSPlus::distanceBetween(path[i].lat, path[i].lon, path[i-1].lat, path[i-1].lon);
        path[i].dist = path[i-1].dist + segDist;
      }
      
      // Calculate gradient
      slope = gradientCalc();
      lastAltTime = millis();
    }

    // Log altitude for graph display
    if ((millis() - lastLogTime) >= logInterval) {
      if (graphPointsCount < graphWidth) {
        signalHistory[graphPointsCount] = gps.altitude.meters();
        graphPointsCount++; 
      } 
      else {
        for (int i = 0; i < graphWidth - 1; i++) {
          signalHistory[i] = signalHistory[i + 1];
        }
        signalHistory[graphWidth - 1] = gps.altitude.meters();
      }
      lastLogTime = millis();
      // Save to ESP
      preferences.begin("gps_history", false);
      preferences.putInt("pt_count", graphPointsCount);
      preferences.putBytes("history_arr", signalHistory, sizeof(signalHistory));
      preferences.putUInt("saved_date", lastStoredDate);
      preferences.end();
    }
  } else {
     lastLogTime = millis();
  }

  // Draw Display
  if (millis() - lastDisplayTime >= 200) {
    lastDisplayTime = millis();

    u8g2.firstPage();
    do {   // When there is gps lock
      if ( (gps.hdop.value() / 100.0) < 5.0 && (gps.satellites.value() > 3) ) {
        u8g2.drawXBMP(0, 0, 32, 30, getCompassBitmap(gps.course.deg()));
        altDisplay(128, 28, gps.altitude.meters());
        slopeDisplay(128, 64, slope);
        historyGraph(0, 34);
      }
      else {    // No gps lock
        u8g2.drawBox(0, 34, (gps.charsProcessed() % 12800)/100, 30);
        u8g2.drawXBMP(0, 0, 32, 30, getCompassBitmap(millis() % 360));
        failDisplay(128, 28, gps.failedChecksum());
        drawSigBars(34, 3, gps.satellites.value(), gps.hdop.value() / 100.0 );
      }
      if ( (gps.hdop.value() / 100.0) > 2.5 ) {
        drawLockStatus(34, 0, gps.satellites.value(), gps.hdop.value() / 100.0 ); 
      }
      displayDim();
    } while (u8g2.nextPage());
  }
}
