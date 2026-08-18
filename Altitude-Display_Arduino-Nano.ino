/**
 * @file Altitude-Display_ESP32-C3-Supermini.ino
 * @brief ESP32 / Arduino GPS Altimeter with Altitude History Graph & Slope Tracking
 * @author Marty Childs <www.childs.be>
 * @date 2026-08-18
 * @version 1.0.6
 * @license MIT
 *
 * @description
 * An Nano powered device that tracks real-time altitude, slope gradient (%) and heading
 * using an SSD1306 128x64 OLED display over I2C and a standard 9600-baud GPS module.
 * Features automated daily resets and persistent graph history using internal storage.
 *
 * @additional
 * A port of the full ESP32-C3 version slimmed down to fit a Nano's harware limitations,
 * it still works on the ESP32.
 *
 * @hardware
 * - Microcontroller: Arduino Nano / ESP32-C3 Supermini / ESP32-C3 Mini
 * - Display: SSD1306 128x64 I2C OLED Display
 * - GPS Module: NEO-6M / BN-220 or compatible (9600 Baud default)
 *
 * @pinout
 *   [ESP32-C3 Mini]
 *   GPIO 6 (SDA)    <--> OLED SDA
 *   GPIO 7 (SCL)    <--> OLED SCL
 *   GPIO 20 (RX)    <--> GPS TX
 *   [Arduino Nano]
 *   A4 / D18 (SDA)  <--> OLED SDA
 *   A5 / D19 (SCL)  <--> OLED SCL
 *   D4 (RX)         <--> GPS TX
 *   D5 (TX)         <--> GPS RX (not used)
 *   [Common]
 *   GND             <--> Common Ground
 *   5V              <--> GPS VCC
 *   3V3             <--> OLED VCC
 *
 * @dependencies
 * - U8g2 by olikraus (OLED driver library)
 * - TinyGPS++ by mikalhart (GPS parsing library)
 */


#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include "Compass_30.inc.h"
#include "Sunrise.inc.h"

#if defined(ARDUINO_AVR_NANO)
  #include <SoftwareSerial.h>
#endif

U8G2_SSD1306_128X64_NONAME_2_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);

#if defined(ARDUINO_ARCH_ESP32)
  #define I2C_SDA 6 
  #define I2C_SCL 7
  #define GPS_RX_PIN 20
  #define GPS_TX_PIN 21
  #define GPS_BAUDRATE 9600
#elif defined(ARDUINO_AVR_NANO)
  #define GPS_RX_PIN 4
  #define GPS_TX_PIN 5
  #define GPS_BAUDRATE 9600
  SoftwareSerial mygps(GPS_RX_PIN, GPS_TX_PIN); 
#endif

const uint8_t graphWidth = 80;   
const uint8_t altPtsCnt = 15;    
unsigned long logInterval = 60000;

struct GPSPoint { float lon; float lat; float dist; float alt; }; 
GPSPoint path[altPtsCnt];

int8_t signalHistory[graphWidth]; 
uint8_t graphPointsCount = 0;     
unsigned long lastLogTime = 0;
unsigned long lastAltTime = 0;
unsigned long lastDisplayTime = 0;
double slope = 0;

TinyGPSPlus gps;

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
    
    if (gps.altitude.meters() < 10000) {
      u8g2.setFont(u8g2_font_helvB10_tn);
      u8g2.setCursor(41, 34); 
      u8g2.print(currentRange);
    }
}

double gradientCalc() {
    double sumX = 0, sumY = 0;
    for (uint8_t i = 0; i < altPtsCnt; i++) {
      sumX += path[i].dist;
      sumY += path[i].alt;  
    }
    double meanX = sumX / altPtsCnt;
    double meanY = sumY / altPtsCnt;
    double num = 0;
    double den = 0;
    for (uint8_t i = 0; i < altPtsCnt; i++) {
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

void altDisplay(uint8_t x, uint8_t y, float altValue) {
    u8g2.setFont(u8g2_font_logisoso28_tr);
    
    char buf[8];
    dtostrf(altValue, 0, 0, buf);
    
    u8g2.setCursor(x - u8g2.getUTF8Width(buf) - 10, y);
    u8g2.print(buf);

    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.drawStr(x - 8, y - 22, "m");
}

void slopeDisplay(uint8_t x, uint8_t y, float slopeValue) {
    char buf[6]; 
    int intSlope = (int)slopeValue;

    if (intSlope > 0) {
        buf[0] = '+';
        itoa(intSlope, buf + 1, 10);
    } else if (intSlope < 0) {
        buf[0] = '-';
        itoa(abs(intSlope), buf + 1, 10);
    } else {
        buf[0] = '0';
        buf[1] = '\0';
    }
    
    u8g2.setFont(u8g2_font_logisoso28_tn);
    int textWidth = u8g2.getUTF8Width(buf);
    u8g2.setCursor(x - textWidth - 10, y); 
    u8g2.print(buf);

    u8g2.setFont(u8g2_font_helvB08_tr);
    u8g2.setCursor(x - 7, y - 28 + 6); 
    u8g2.print(F("%"));
}

void setup(void) {
    #if defined(ARDUINO_ARCH_ESP32)
      Wire.begin(I2C_SDA, I2C_SCL);
      Serial1.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    #elif defined(ARDUINO_AVR_NANO)
      mygps.begin(GPS_BAUDRATE);
    #endif
    
    u8g2.begin();
    u8g2.setBusClock(400000);
    u8g2.setContrast(0);
}

void loop(void) {
  #if defined(ARDUINO_ARCH_ESP32)
    while (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
  #elif defined(ARDUINO_AVR_NANO)
    while (mygps.available() > 0) {
      gps.encode(mygps.read());
    }
  #endif
 
  if (gps.location.isValid() && gps.date.isValid() && gps.time.isValid()) { 
    if ((millis() - lastAltTime) >= 1000) {
      if (path[altPtsCnt - 1].lat == 0 && path[altPtsCnt - 1].lon == 0) {
        for (uint8_t i = 0; i < altPtsCnt; i++) {
          path[i].lat = gps.location.lat();
          path[i].lon = gps.location.lng();
          path[i].alt = gps.altitude.meters();
          path[i].dist = 0;
        }
      } else {
        for (uint8_t i = 0; i < altPtsCnt - 1; i++) {
          path[i] = path[i + 1];
        }
        path[altPtsCnt - 1].lat = gps.location.lat();
        path[altPtsCnt - 1].lon = gps.location.lng();
        path[altPtsCnt - 1].alt = gps.altitude.meters();
      }

      path[0].dist = 0;
      for (uint8_t i = 1; i < altPtsCnt; i++) {
        double segDist = TinyGPSPlus::distanceBetween(path[i].lat, path[i].lon, path[i-1].lat, path[i-1].lon);
        path[i].dist = path[i-1].dist + segDist;
      }
      
      slope = gradientCalc();
      calculateSolarTimes( gps.location.lat(), gps.location.lng(), gps.date.year(), gps.date.month(), gps.date.day() );
      u8g2.setContrast(displayDim(gps.time.hour(), gps.time.minute()));
      lastAltTime = millis();
    }

    if ((millis() - lastLogTime) >= logInterval) {
      long rawAlt = round(gps.altitude.meters());
      
      if (rawAlt > 5000) rawAlt = 5000;
      if (rawAlt < -5000) rawAlt = -5000;
      
      int8_t scaledAlt = (int8_t)(rawAlt / 40);

      if (graphPointsCount < graphWidth) {
        signalHistory[graphPointsCount] = scaledAlt;
        graphPointsCount++; 
      } 
      else {
        for (uint8_t i = 0; i < graphWidth - 1; i++) {
          signalHistory[i] = signalHistory[i + 1];
        }
        signalHistory[graphWidth - 1] = scaledAlt;
      }
      lastLogTime = millis();
    }
  } else {
     lastLogTime = millis();
  }

  if (millis() - lastDisplayTime >= 200) {
    lastDisplayTime = millis();

    u8g2.firstPage();
    do {   
      if ( (gps.hdop.value() / 100.0) < 5.0 && (gps.satellites.value() > 3) ) {
        u8g2.drawXBMP(0, 0, 32, 30, getCompassBitmap(gps.course.deg()));
        altDisplay(128, 28, gps.altitude.meters());
        slopeDisplay(128, 64, slope);
        historyGraph(0, 34);
      }
      else {    
        u8g2.drawBox(0, 34, (gps.charsProcessed() % 12800)/100, 30);
        u8g2.drawXBMP(0, 0, 32, 30, getCompassBitmap(millis() % 360));
      }
    } while (u8g2.nextPage());
  }
}
