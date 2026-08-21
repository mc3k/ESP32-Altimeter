/**
 * @file Altitude-Display_ESP32-C3-Supermini.ino
 * @brief ESP32-C3 Supermini GPS Altimeter with Altitude History Graph & Slope Tracking
 * @author Marty Childs <www.childs.be>
 * @date 2026-08-21
 * @version 1.0.45
 * @license MIT4
 * 
 * @description
 * An ESP32 powered device that tracks real-time altitude, slope gradient (%) and heading 
 * using an SSD1306 128x64 OLED display over I2C and a standard 9600-baud GPS module.
 * Features automated daily resets and persistent graph history using internal storage.
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
 * - Sunset by Peter Buelow (Sunrise/set from GPS)
 */


#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <sunset.h>
#include <LittleFS.h> 

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

#define I2C_SDA 6 // C3 Supermini
#define I2C_SCL 7

#define GPS_RX_PIN 20
#define GPS_TX_PIN 21
#define GPS_BAUDRATE 9600

const int graphWidth = 80;
const int altPtsCnt = 15;
const bool resetOnBoot = true;
unsigned long logInterval = 60000;

const char* HISTORY_FILE = "/history.bin"; 

struct GPSPoint { double lon; double lat; double dist; double alt; };
GPSPoint path[altPtsCnt];

int signalHistory[graphWidth];
int graphPointsCount = 0;
unsigned long lastLogTime = 0;
unsigned long lastAltTime = 0;
unsigned long lastDisplayTime = 0;
uint32_t lastStoredDate = 0; 
bool dayChangeCheckedThisBoot = false; 
double slope = 0;

TinyGPSPlus gps;
SunSet sun;

// Brightness parameters (0 to 255)
const uint8_t DAY_BRIGHTNESS = 255;
const uint8_t NIGHT_BRIGHTNESS = 15; 
uint8_t currentBrightness = 15;

#include "LittleFS.inc.h"
#include "Compass_30.inc.h"
#include "Graphics.inc.h"

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
      int endX = x + int((i+1) * pointWidth);
      int currentBarWidth = max(1, endX - startX);

      u8g2.drawBox(startX, yStart, currentBarWidth, dataHeight);
    }
    
    // if (gps.altitude.meters() < 1000) {
    //   u8g2.setFont(u8g2_font_helvB10_tr);
    //   u8g2.setCursor(41, 34); 
    //   u8g2.print(currentRange);
      
    //   u8g2.setFont(u8g2_font_open_iconic_arrow_1x_t); 
    //   u8g2.drawGlyph(29, 34, 84);
      
    //   u8g2.setFont(u8g2_font_helvB08_tr);
    // }
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
    
    if(!LittleFS.begin(true)){ 
        Serial.println("LittleFS Mount Failed");
    } else {
        loadHistoryFromFS(); 
    }

    Wire.begin(I2C_SDA, I2C_SCL);
    Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    u8g2.begin();
    u8g2.setBusClock(400000);
    u8g2.setContrast(currentBrightness);
}

void loop(void) {

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  // Day-change evaluation step
  if (!dayChangeCheckedThisBoot && gps.date.isValid() && gps.date.value() != 0) {
    uint32_t currentGpsDate = gps.date.value(); 

    if (resetOnBoot) { // Only evaluate date differences if user wants the feature active
        if (lastStoredDate != 0 && lastStoredDate != currentGpsDate) {
            clearHistoryData();
            lastStoredDate = currentGpsDate;
            saveHistoryToFS();
        } else {
            lastStoredDate = currentGpsDate;
        }
    }
    dayChangeCheckedThisBoot = true; 
  }
 
  if (gps.location.isValid()) { 
    
    // Log new altitude for gradient calc
    if ((millis() - lastAltTime) >= 1000) {
      if (path[altPtsCnt-1].lat == 0 && path[altPtsCnt-1].lon == 0) {
        for (int i = 0; i < altPtsCnt; i++) {
          path[i].lat = gps.location.lat();
          path[i].lon = gps.location.lng();
          path[i].alt = gps.altitude.meters();
          path[i].dist = 0;
        }
      } else {
        for (int i = 0; i < altPtsCnt-1; i++) {
          path[i] = path[i+1];
        }
        path[altPtsCnt-1].lat = gps.location.lat();
        path[altPtsCnt-1].lon = gps.location.lng();
        path[altPtsCnt-1].alt = gps.altitude.meters();
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
          signalHistory[i] = signalHistory[i+1];
        }
        signalHistory[graphWidth - 1] = gps.altitude.meters();
      }
      lastLogTime = millis();
      
      saveHistoryToFS();
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
