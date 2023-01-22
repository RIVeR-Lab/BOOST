#include "ePaper.h"

bool ePaper::initialize() {
  bool success = true;
  LOGEVENT("Initializing ePaper...");

  pinMode(EPD_CS, OUTPUT);
  ePaper_CS_1;
  pinMode(EPD_RESET, OUTPUT);
  ePaper_RST_0;
  pinMode(EPD_DC, OUTPUT);
  ePaper_DC_0;
  pinMode(EPD_READY, INPUT);

  SPI.beginTransaction(spiSettings);
  SPI.begin();

  // reset driver
  ePaper_RST_0;
  delay(20);
  ePaper_RST_1;
  delay(20);

  ePaper_RST_0;
  delay(20);
  ePaper_RST_1;
  delay(20);

  ePaper_RST_0;
  delay(20);
  ePaper_RST_1;
  delay(20);

  // Power Setting
  writeCMD(0x01);
  writeData(0x03);
  writeData(0x00);
  writeData(0x2B);
  writeData(0x2B);

  // Booster Soft Start
  writeCMD(0x06);
  writeData(0x17);
  writeData(0x17);
  writeData(0x17);

  // Power On
  writeCMD(0x04);
  // wait until powered on
  success = success && waitEPDReady(displayReadyTimeout);

  if (!success) {
    LOGERROR("FAILED to power on ePaper.");
  } else {
    // Panel Setting
    setOTPLUT();

    // setRegisterLUT();

    // TODO: change explictly define all these command bytes
    // PLL Control
    writeCMD(0x30);
    writeData(0x3a);

    // Resolution
    writeCMD(0x61);
    writeData(0x01); // first half of 2 bytes:  256
    writeData(0x90); // second half of 2 bytes: 144 -> first half plus second
                     // half = 400
    writeData(0x01); // first half of 2 bytes:  256
    writeData(0x2c); // second half of 2 bytes: 44  -> first half plus second
                     // half = 300

    // VCM DC Settings
    writeCMD(0x82);
    writeData(0x12);

    // Vcom and data interval setting
    writeCMD(0x50);
    writeData(0x07);
  }

  if (!success) {
    LOGERROR("FAILE to initialize ePaper.");
  } else {
    LOGEVENT("ePaper initialized SUCCESSFULLY.");
  }

  return success;
}

bool ePaper::updateDisplay(State_t mdl) {
  bool success = true;
  LOGEVENT("Updating display...");
  uint16_t x, y;

  powerON();
  fillScreen(GxEPD_WHITE); // Reset display buffer.
  canvas.fillScreen(GxEPD_WHITE);

  // Set the voltage level in upper left corner.
  int16_t vlTbx, vlTby;
  uint16_t vlTbw, vlTbh;
  canvas.setTextWrap(true);
  canvas.setFont(&FreeSans18pt7b);
  canvas.setTextColor(GxEPD_BLACK);
  canvas.setTextSize(1);
  canvas.getTextBounds(mdl.selectedVoltage.name, 0, 0, &vlTbx, &vlTby, &vlTbw,
                       &vlTbh);
  y = -vlTby; // height() - vlTbh - vlTby;
  x = -vlTbx; // + (width()/2) - (vlTbw/2);
  canvas.setCursor(x, y);
  canvas.print(mdl.selectedVoltage.name);

  // InkBit Logo in upper right corner
  canvas.drawXBitmap(canvas.width() - logo_inkbit_black_20_60_width,
                     canvas.height() - logo_inkbit_black_20_60_height,
                     logo_inkbit_black_20_60, logo_inkbit_black_20_60_width,
                     logo_inkbit_black_20_60_height, 0xffff);

  // Put commit SHA in lower left corner
  int16_t shaTbx, shaTby;
  uint16_t shalTbw, shalTbh;
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(GxEPD_BLACK);
  canvas.setTextSize(1);
  canvas.getTextBounds(mdl.selectedVoltage.name, 0, 0, &shaTbx, &shaTby, &shalTbw, &shalTbh);
  y = canvas.height() - shalTbh - shaTby;
  x = 0 - shaTbx + 2;
  canvas.setCursor(x, y);
  std::string commitTxt = "";
  commitTxt.append("ver:").append(Version::getGitCommitSha1().substr(0, 7));
  canvas.print(commitTxt.c_str());

  // Get state's string.
  int16_t tbx, tby, tx, ty;
  uint16_t tbw, tbh;
  std::string textStr = mdl.generateTextString();
  const char *text = textStr.c_str();
  LOGEVENT("Displaying text:\n\r```\n\r%s\n\r```", text);
  canvas.setTextWrap(true);
  canvas.setFont(&FreeSansBold12pt7b);
  canvas.setTextColor(GxEPD_BLACK);
  canvas.setTextSize(1);
  canvas.getTextBounds(text, 0, 0, &tbx, &tby, &tbw, &tbh);

  // Move text up if there is additional info to display.
  int additionalMargin = 0;
  if(mdl.additionalInfo.length() > 0) {
    additionalMargin = -10;
  }

  // Center text in middle of display.
  tx = -tbx + (canvas.width() / 2) - (tbw / 2);
  ty = -tby + (canvas.height() / 2) - (tbh / 2) + additionalMargin;
  canvas.setCursor(tx, ty);
  canvas.print(text);

  // Display additional info
  int16_t abx, aby, ax, ay;
  uint16_t abw, abh;
  std::string additionalInfoTextStr = mdl.additionalInfo;
  const char *additionalInfoText = additionalInfoTextStr.c_str();
  LOGEVENT("Displaying text:\n\r```\n\r%s\n\r```", additionalInfoText);
  canvas.setTextWrap(true);
  canvas.setFont(&FreeSans9pt7b);
  canvas.setTextColor(GxEPD_BLACK);
  canvas.setTextSize(1);
  canvas.getTextBounds(additionalInfoText, 0, 0, &abx, &aby, &abw, &abh);
  // Put text below state string text.
  ax = -abx + (canvas.width() / 2) - (abw / 2);
  ay = tbh - aby + (canvas.height() / 2) - (abh / 2) + 5;
  canvas.setCursor(ax, ay);
  canvas.print(additionalInfoText);

  // Draw the canvas onto the display.
  drawBitmap(marginPx, marginPx, canvas.getBuffer(), canvas.width(),
             canvas.height(), 0xffff);

  // Write buffer.
  writeCMD(0x13);
  for (uint32_t i = 0; i < EPAPER_BUFFER_SIZE; i++) {
    writeData((i < sizeof(_buffer)) ? ~_buffer[i] : 0xFF);
  }

  // refresh the display
  writeCMD(0x12);
  success = success && waitEPDReady(displayReadyTimeout);
  powerOff();
  LOGEVENT("Display updated: %s", success ? "SUCCESS" : "FAILED");
  return success;
}

void ePaper::drawPixel(int16_t x, int16_t y, uint16_t color) {
  if ((x < 0) || (x >= width()) || (y < 0) || (y >= height()))
    return;

  // check rotation, move pixel around if necessary
  switch (getRotation()) {
  case 1:
    swap(x, y);
    x = EPAPER_WIDTH - x - 1;
    break;
  case 2:
    x = EPAPER_WIDTH - x - 1;
    y = EPAPER_HEIGHT - y - 1;
    break;
  case 3:
    swap(x, y);
    y = EPAPER_HEIGHT - y - 1;
    break;
  }
  uint16_t i = x / 8 + y * EPAPER_WIDTH / 8;
  if (_current_page < 1) {
    if (i >= sizeof(_buffer))
      return;
  } else {
    y -= _current_page * EPAPER_PAGE_HEIGHT;
    if ((y < 0) || (y >= EPAPER_PAGE_HEIGHT))
      return;
    i = x / 8 + y * EPAPER_WIDTH / 8;
  }

  if (!color)
    _buffer[i] = (_buffer[i] | (1 << (7 - x % 8)));
  else
    _buffer[i] = (_buffer[i] & (0xFF ^ (1 << (7 - x % 8))));
}