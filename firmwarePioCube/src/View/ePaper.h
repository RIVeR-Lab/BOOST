/**
 * Based on these libraries:
 * https://github.com/adafruit/Adafruit-GFX-Library.git
 * https://github.com/ZinggJM/GxEPD.git
 * https://github.com/olikraus/U8g2_for_Adafruit_GFX.git
 */

#ifndef _SRC_EPAPER_H
#define _SRC_EPAPER_H

#include "Adafruit_GFX.h"
#include "MantaCommon/src/utils/log.h"
#include "View/IView.h"
#include "images_bmps/logo-inkbit-black-20-60.h"
#include "images_bmps/logo-inkbit-black-30-90.h"
#include "pins.h"
#include "version.h"
#include <Arduino.h>
#include <SPI.h>

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMono12pt7b.h>
#include <Fonts/FreeMono18pt7b.h>
#include <Fonts/FreeMono24pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeSans12pt7b.h>
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans24pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeSansBold12pt7b.h>
#include <Fonts/FreeSansBold18pt7b.h>
#include <Fonts/FreeSansBold24pt7b.h>
#include <Fonts/FreeSansBold9pt7b.h>

#define ePaper_RST_0 (digitalWrite(EPD_RESET, LOW))
#define ePaper_RST_1 (digitalWrite(EPD_RESET, HIGH))
#define ePaper_CS_0 (digitalWrite(EPD_CS, LOW))
#define ePaper_CS_1 (digitalWrite(EPD_CS, HIGH))
#define ePaper_DC_0 (digitalWrite(EPD_DC, LOW))
#define ePaper_DC_1 (digitalWrite(EPD_DC, HIGH))

#define GxEPD_WHITE 0x00
#define GxEPD_BLACK 0xFF

/* ------------------------------------------------------------- */
// Display Selection
#define CFAP128296C0_0290 true
#define CFAP122250A2_0213 false
/* ------------------------------------------------------------- */

/* ------------------------------------------------------------- */
#if CFAP128296C0_0290
// 2.9in Display
// https://www.crystalfontz.com/product/cfap128296c00290-128x296-epaper-display-eink
#define EPAPER_WIDTH 128
#define EPAPER_HEIGHT 296
#define EPAPER_BUFFER_SIZE                                                     \
  (uint32_t(EPAPER_WIDTH) * uint32_t(EPAPER_HEIGHT) / 8)
// divisor for AVR, should be factor of EPAPER_HEIGHT
#define EPAPER_PAGES 8
#define EPAPER_PAGE_HEIGHT (EPAPER_HEIGHT / EPAPER_PAGES)
#define EPAPER_PAGE_SIZE (EPAPER_BUFFER_SIZE / EPAPER_PAGES)
/* ------------------------------------------------------------- */

/* ------------------------------------------------------------- */
#elif CFAP122250A2_0213
// 2.13in Display
// https://www.crystalfontz.com/product/cfap122250a20213-122x250-epaper-display-module
#define CFAP122250A2_0213_WIDTH 122
#define CFAP122250A2_0213_HEIGHT 250
#define CFAP122250A2_0213_BUFFER_SIZE                                          \
  (uint32_t(CFAP122250A2_0213_WIDTH) * uint32_t(CFAP122250A2_0213_HEIGHT) / 8)
// divisor for AVR, should be factor of CFAP122250A2_0213_HEIGHT
#define CFAP122250A2_0213_PAGES 8
#define CFAP122250A2_0213_PAGE_HEIGHT                                          \
  (CFAP122250A2_0213_HEIGHT / CFAP122250A2_0213_PAGES)
#define CFAP122250A2_0213_PAGE_SIZE                                            \
  (CFAP122250A2_0213_BUFFER_SIZE / CFAP122250A2_0213_PAGES)
/* ------------------------------------------------------------- */
#else
#error "No Display Selected"
#endif

class ePaper : public IView, public Adafruit_GFX {
public:
  ePaper(SPIClass &spi)
      : Adafruit_GFX(EPAPER_WIDTH, EPAPER_HEIGHT),
        canvas(height() - (marginPx * 2), width() - (marginPx * 2)),
        displayPort(spi), _current_page(-1) {
    setRotation(3);
  }
  ~ePaper() {}

  bool initialize() override;

  bool displayError(const char *errorStr) override { return true; }

  bool updateDisplay(State_t mdl) override;

private:
  template <typename T> static inline void swap(T &a, T &b) {
    T t = a;
    a = b;
    b = t;
  }

  // The number of pixels around the display boarder to use as margin.
  static const uint32_t marginPx = 5;

  // Where we draw the display contents so that we can have margin around the
  // edges. We first draw all contents on this intermediary convas which is
  // slighty smaller than the actually display size. Then we draw this convas
  // onto the display.
  GFXcanvas1 canvas;

  SPIClass &displayPort;
  static const uint32_t displayReadyTimeout = 5000;
  SPISettings spiSettings = SPISettings(2000000, MSBFIRST, SPI_MODE0);

  uint8_t _buffer[EPAPER_BUFFER_SIZE];
  int16_t _current_page;
  void drawPixel(int16_t x, int16_t y, uint16_t color);

  void powerON() { writeCMD(0x04); }

  void powerOff() {
    writeCMD(0x02);
    writeCMD(0x03);
    writeData(0x00);
  }

  void setOTPLUT() {
    // set panel setting to call LUTs from OTP
    writeCMD(0x00);
    writeData(0x93);
  }

  bool waitEPDReady(uint32_t timeout_ms) {
    bool success = true;
    uint32_t startTime = millis();
    while (success && LOW == digitalRead(EPD_READY)) {
      if (millis() - startTime > timeout_ms) {
        LOGERROR("Display not ready timedout");
        success = false;
      }
    }
    return success;
  }

  // this function will take in a byte and send it to the display with the
  // command bit low for command transmission
  void writeCMD(uint8_t command) {
    SPI.beginTransaction(spiSettings);
    ePaper_DC_0;
    ePaper_CS_0;
    SPI.transfer(command);
    ePaper_CS_1;
    SPI.endTransaction();
  }

  // this function will take in a byte and send it to the display with the
  // command bit high for data transmission
  void writeData(uint8_t data) {
    SPI.beginTransaction(spiSettings);
    ePaper_DC_1;
    ePaper_CS_0;
    SPI.transfer(data);
    ePaper_CS_1;
    SPI.endTransaction();
  }
};

#endif // _SRC_EPAPER_H
