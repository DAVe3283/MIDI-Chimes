// -----------------------------------------------------------------------------
// Master.ino
//
// The master Teensy that runs the MIDI chimes.
// This teensy is responsible for running the display, USB and physical MIDI,
// address auto-assignment, etc.
//
// TODO:
// *****************************************************************************
// * USB MIDI locks up (including host PC playback application) when a slave
// * doesn't work as expected (i.e. unplugged.)
// * Find out why, and make the master handle this gracefully, so the host PC
// * doesn't have to deal with a hung MIDI device.
// * This could be an I2C timeout, which I am not handling.
// *****************************************************************************
// * Velocity map (MIDI is 0-127, I need a PWM value ~50%-100%)
//   * Per-note velocity map? This would make the chimes perfectly linear
//   * Or a velocity formula (per note?) might be better?
// * Actually implement master volume
// * Physical MIDI support
//   * When a note doesn't match the current channel, pass it to MIDI out
//   * Software MIDI "thru" support (pass all inputs to the output when thru is on)
//   * Or do we want hardware MIDI THRU, which is just a transistor?
// * Implement I2C timeout (properly) & handle I2C errors
//   * Probably need a very short timeout (1ms or less) to avoid messing up song
//     * Or should we just throw a giant error and abort playback?
// * Double-check pin config for SPI
//   * If I can move the data/command pin off a hardware CS pin, I can probably
//     use that hardware pin for the touch screen or SD card (speed things up)
// * Make the display do useful things
//   * Show errors
//   * Show input source (USB or physical MIDI)
//   * Master volume
//     * Working as proof of concept, but I want to implement this better
//   * Options
//     * Override velocity to master volume vs. scale velocity 0-master
//       * Default should probably be scale
//     * Physical MIDI port is OUT or THRU
//     * Pass USB MIDI to physical MIDI OUT? (or should I always just do this?)
// * Read from SD card
//   * Basic file browser?
// * Playback of MIDI files (from SD card)
//   * Play doorbel tone from sd "doorbell" folder :P
// * Save options to EEPROM (or FLASH or whatever the Teensy has)
//   * Or save them to the SD card?
// -----------------------------------------------------------------------------

// Comment this line out if using the resistive touchscreen layer
//#define CAPACITIVE_TS

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

// Core I/O (display, slaves, SD card)
#include <SPI.h>
#include <i2c_t3.h>
#include <ILI9341_t3.h>
#include <SdFat.h>

extern "C"
{
  #include "ugui.h"
}

// Touchscreen driver
#ifdef CAPACITIVE_TS
  #include <Adafruit_FT6206.h>
#else
  #include <Adafruit_STMPE610.h>
#endif

// Fonts
#include <font_LiberationMono.h>
#include <font_LiberationSans.h>
// #include <font_LiberationSansBold.h>
// #include <font_LiberationSansItalic.h>
// #include <font_LiberationSansBoldItalic.h>
#include <font_AwesomeF000.h>

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I2C config
const uint8_t slave_addresses[] =
{
  'A', // Slave 0
  'B', // Slave 1
  'C', // Slave 2
};
const size_t num_slaves(sizeof(slave_addresses)/sizeof(*slave_addresses));
// TODO: auto-assignment of slave addresses

// Note mapping
const size_t notes_per_slave(10);
// WARNING: make sure this table is big enough!
const uint8_t note_map[num_slaves][notes_per_slave] =
{
  // Slave 0
  // G3  G3# A3  A3# B3  C4  C4# D4  D4# E4
  {  55, 56, 57, 58, 59, 60, 61, 62, 63, 64 },

  // Slave 1
  // F4  F4# G4  G4# A4  A4# B4  C5  C5# D5
  {  65, 66, 67, 68, 69, 70, 71, 72, 73, 74 },

  // Slave 2
  // D5# E5  F5  F5# G5  <--- Not Used --->
  {  75, 76, 77, 78, 79,  0,  0,  0,  0,  0 },
};
// TODO: convert this from a reverse-lookup table to a lookup table.
// It won't be as easy to read, but it will be faster. Or do we need the speed?
// TODO: use real data once the chimes are hooked up

// Duty Cycle Settings
const uint8_t pwm_bits(12); // 0 - 4095
const float minimum_pwm(0.55); // 55%, lowest reliable impact to produce a chime
const uint16_t maximum_dc((1 << pwm_bits) - 1);
const uint16_t minimum_dc(minimum_pwm * maximum_dc);

// Pins
const uint8_t spi_sck_pin(14);
const uint8_t spi_mosi_pin(11);
const uint8_t spi_miso_pin(12);
const uint8_t led_pin(13);
const uint8_t sd_cs_pin(15);

// Touch Screen
// Pins
const uint8_t lcd_reset_pin(7);
const uint8_t lcd_cs_pin(10);
const uint8_t lcd_dc_pin(9);
const uint8_t touch_cs_pin(8);
// Calibration
const uint16_t ts_min_x( 150);
const uint16_t ts_max_x(3800);
const uint16_t ts_min_y( 130);
const uint16_t ts_max_y(4000);

// Timeouts
const uint16_t blink_time(20000); // microseconds

// Graphics settings
const uint16_t background_color(ILI9341_BLUE);
const uint16_t text_color(ILI9341_WHITE);
const uint16_t button_color(ILI9341_WHITE);
const uint16_t button_border(ILI9341_BLACK);
const uint16_t button_text(button_border);
const uint16_t button_text_disabled(ILI9341_LIGHTGREY);
const  int16_t button_radius(5);
const uint16_t volume_color(ILI9341_RED);

// DEBUG: volume button sizes (until I make a class and do this right)
const int16_t button_vol_d_x(  5);
const int16_t button_vol_d_y(185);
const int16_t button_vol_u_x(265);
const int16_t button_vol_u_y(185);
const int16_t button_vol_w(50);
const int16_t button_vol_h(50);


// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity); // Not needed
// TODO: handle more MIDI stuff?
// void OnVelocityChange(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnControlChange(uint8_t channel, uint8_t control, uint8_t value);
// void OnProgramChange(uint8_t channel, uint8_t program);
// void OnAfterTouch(uint8_t channel, uint8_t pressure);
// void OnPitchChange(uint8_t channel, int pitch);

// Scale a MIDI velocity (7-bit) to our 12-bit velocity
// TODO: calibrate for linear sound output, calibrate per chime, etc.
uint16_t scale_midi_velocity(const uint8_t& midi_velocity);

// Send a chime strike command to a slave
void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity);

// Gets the slave address and channel on the slave for a given MIDI note
bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out);

// Master Volume
void draw_master_volume();
void update_master_volume();

// µGUI
void UserPixelSetFunction(UG_S16 x, UG_S16 y, UG_COLOR c);
// µGUI Hardware Acceleration
UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);
UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);

// DEBUG / TESTING
void glyph_test();
bool hit_test(const int16_t& x, const int16_t& y, const int16_t& _x, const int16_t& _y, const int16_t& _w, const int16_t& _h);


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// User settings
uint8_t our_channel(0); // Current channel (1-16, 0 means play all)
uint8_t master_volume(100); // Master volume (0-100, 0 means mute, steps of 10)
bool override_velocity(false); // Override velocity to master volume (true), or scale it by master volume (false)

// Timers
elapsedMicros message_blink_timer;

// Serial IO
// usb_serial_class  usb = usb_serial_class();
HardwareSerial    ser = HardwareSerial();
// TODO: hardware MIDI

// SD Card
SdFat sd;
SdFile file;
SdFile dirFile;

// Touch Screen
ILI9341_t3 tft = ILI9341_t3(lcd_cs_pin, lcd_dc_pin, lcd_reset_pin, spi_mosi_pin, spi_sck_pin, spi_miso_pin); // TFT LCD
#ifdef CAPACITIVE_TS
  Adafruit_FT6206 ts = Adafruit_FT6206();  // Using default Arduino I2C pins?
#else
  Adafruit_STMPE610 ts = Adafruit_STMPE610(touch_cs_pin); // Touch sensor
#endif

// µGUI
UG_GUI gui;
#define MAX_OBJECTS 10
// ^^ temp #define
UG_WINDOW window0;
UG_BUTTON button0;
UG_BUTTON button1;
UG_BUTTON button2;
UG_OBJECT obj_buff_window0[MAX_OBJECTS];
void window0_callback(UG_MESSAGE* msg)
{
  if ((msg->type == MSG_TYPE_OBJECT) &&
      (msg->id == OBJ_TYPE_BUTTON) &&
      (msg->event == BTN_EVENT_CLICKED))
  {
    switch (msg->sub_id)
    {
    case BTN_ID_0:
      UG_DriverEnable(DRIVER_DRAW_LINE);
      UG_DriverEnable(DRIVER_FILL_FRAME);
      UG_ButtonSetBackColor(&window0, BTN_ID_0, C_GREEN);
      UG_ButtonSetBackColor(&window0, BTN_ID_1, UG_WindowGetBackColor(&window0));
      break;

    case BTN_ID_1:
      UG_DriverDisable(DRIVER_DRAW_LINE);
      UG_DriverDisable(DRIVER_FILL_FRAME);
      UG_ButtonSetBackColor(&window0, BTN_ID_0, UG_WindowGetBackColor(&window0));
      UG_ButtonSetBackColor(&window0, BTN_ID_1, C_GREEN);
      break;

    case BTN_ID_2:
      UG_WindowHide(&window0);
      UG_WindowShow(&window0);
      break;

    default:
      break;
    }
  }
}

// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  // Configure serial (debug)
  ser.begin(1500000);

  // Configure TFT
  tft.begin();

  // Initialize display
  tft.setRotation(1); // Landscape
  // Un-burn-in the LCD somewhat
  tft.fillScreen(ILI9341_BLACK);
  delay(100);
  tft.fillScreen(ILI9341_RED);
  delay(100);
  tft.fillScreen(ILI9341_GREEN);
  delay(100);
  tft.fillScreen(ILI9341_BLUE);
  delay(100);
  tft.fillScreen(ILI9341_WHITE);
  delay(200);
  tft.fillScreen(background_color);

  // Test glyphs
  //glyph_test();

  // Testing master volume!
  draw_master_volume();

  // Configure I2C
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);

  tft.setFontAdafruit();
  tft.setCursor(0, 0);
  tft.setTextColor(text_color);
  if (!ts.begin())
  {
    tft.println("Unable to start touchscreen.");
  }
  else
  {
    tft.println("Touchscreen started.");
  }

  //////////////////////////////////////////////////////////////////////////////
  // Directory list SD card
  //////////////////////////////////////////////////////////////////////////////

  // Init SD
  tft.print("SdFat version: ");
  tft.println(SD_FAT_VERSION);
  if (!sd.begin(sd_cs_pin))
  {
    sd.initErrorHalt(&tft);
  }

  // Display card info
  const uint32_t cardSize(sd.card()->cardSize());
  if (cardSize == 0) {
    sd.errorHalt(&tft, "cardSize failed");
  }
  tft.print("Card type: ");
  switch (sd.card()->type()) {
  case SD_CARD_TYPE_SD1:
    tft.println("SD1");
    break;

  case SD_CARD_TYPE_SD2:
    tft.println("SD2");
    break;

  case SD_CARD_TYPE_SDHC:
    if (cardSize < 70000000) {
      tft.println("SDHC");
    } else {
      tft.println("SDXC");
    }
    break;

  default:
    tft.println("Unknown");
  }

  // CID Dump
  cid_t cid;
  if (!sd.card()->readCID(&cid)) {
    sd.errorHalt(&tft, "readCID failed");
  }
  tft.print("Manufacturer ID: 0x");
  tft.println(static_cast<int>(cid.mid), HEX);
  tft.print("OEM ID: 0x");
  tft.print(cid.oid[0], HEX);
  tft.println(cid.oid[1], HEX);
  // tft.print("Product: ");
  // for (uint8_t i = 0; i < 5; i++) {
  //   tft.print(cid.pnm[i]);
  // }
  // tft.println();
  // tft.print("Version: ");
  // tft.print(static_cast<int>(cid.prv_n));
  // tft.print(".");
  // tft.println(static_cast<int>(cid.prv_m));
  tft.print("Serial number: 0x");
  tft.println(cid.psn, HEX);
  tft.print("Manufacturing date: ");
  tft.print(static_cast<int>(cid.mdt_month));
  tft.print("/");
  tft.println(2000 + cid.mdt_year_low + 10 * cid.mdt_year_high);
  tft.println("File listing:");

  //tft.println("SD card found! File listing:");
  // tft.print("FreeStack: ");
  // tft.println(FreeStack());
  // tft.println();

  // List files in root directory.
  if (!dirFile.open("/", O_READ))
  {
    sd.errorHalt(&tft, "open root failed");
  }
  uint16_t files_found(0);
  const uint16_t nMax(13); // Max files to list
  while (files_found < nMax && file.openNext(&dirFile, O_READ))
  {
    // Skip directories and hidden files.
    if (!file.isSubDir() && !file.isHidden())
    {
      // Save dirIndex of file in directory.
      //dirIndex[files_found] = file.dirIndex();

      // Print the file number and name.
      files_found++;
      //tft.print(files_found++);
      //tft.print(' ');
      tft.print(file.dirIndex());
      tft.print(" ");
      file.printName(&tft);
      tft.println();
    }
    file.close();
  }

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Configure USB MIDI
  usbMIDI.setHandleNoteOn(OnNoteOn);
  // usbMIDI.setHandleNoteOff(OnNoteOff); // Not needed
  // TODO: handle more MIDI stuff?

  // GUI Tests
  UG_Init(&gui, UserPixelSetFunction, tft.width(), tft.height());
  UG_DriverRegister(DRIVER_DRAW_LINE, (void*)_HW_DrawLine);
  UG_DriverRegister(DRIVER_FILL_FRAME, (void*)_HW_FillFrame);
  // UG_FontSelect(&FONT_8X12);
  // UG_PutString(0, 0, "Test");
  // Window Tests
  UG_WindowCreate(&window0, obj_buff_window0, MAX_OBJECTS, window0_callback);
  UG_WindowResize(&window0, 20, 20, 319-20, 239-20);
  UG_WindowSetTitleText(&window0, "uGUI Test Window");
  UG_WindowSetTitleTextFont(&window0, &FONT_8X12);
  UG_ButtonCreate(&window0, &button0, BTN_ID_0, 10, 10, 100,  60);
  UG_ButtonSetFont(&window0, BTN_ID_0, &FONT_8X12);
  UG_ButtonSetText(&window0, BTN_ID_0, "H/W Acc\nON");
  UG_ButtonSetBackColor(&window0, BTN_ID_0, C_GREEN);
  UG_ButtonCreate(&window0, &button1, BTN_ID_1, 10, 70, 100, 130);
  UG_ButtonSetFont(&window0, BTN_ID_1, &FONT_8X12);
  UG_ButtonSetText(&window0, BTN_ID_1, "H/W Acc\nOFF");
  UG_ButtonCreate(&window0, &button2, BTN_ID_2, 110, 10, 200, 60);
  UG_ButtonSetFont(&window0, BTN_ID_2, &FONT_8X12);
  UG_ButtonSetText(&window0, BTN_ID_2, "Redraw");
  UG_WindowShow(&window0);
}

// Main program loop
void loop()
{
  // Handle USB MIDI messages
  usbMIDI.read();

  // Handle GUI & Touch
  const bool touched(ts.touched());
#ifdef CAPACITIVE_TS
  if (touched)
  {
    // Retrieve a point
    TS_Point p = ts.getPoint();

    // Rotate the screen
    const int y(p.x);
    const int x(tft.width() - p.y);

    // New GUI Test
    UG_TouchUpdate(x, y, TOUCH_STATE_PRESSED);
  }
  else
  {
    UG_TouchUpdate(-1, -1, TOUCH_STATE_RELEASED);
  }
#else // Resistive touch screen
  if (!ts.bufferEmpty())
  {
    // Retrieve a point
    TS_Point p = ts.getPoint();

    // Scale using the calibration #'s and rotate coordinate system
    p.x = map(p.x, ts_min_y, ts_max_y, 0, tft.height());
    p.y = map(p.y, ts_min_x, ts_max_x, 0, tft.width());
    const int x(p.y);
    const int y(tft.height() - p.x);

    // New GUI Test
    UG_TouchUpdate(x, y, touched ? TOUCH_STATE_PRESSED : TOUCH_STATE_RELEASED);
  }
  else
  {
    if (!touched)
    {
      UG_TouchUpdate(-1, -1, TOUCH_STATE_RELEASED);
    }
  }
#endif



  UG_Update();
}

void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
  // MIDI spec allows turning a note off by sending a note on with velocity = 0
  // We don't care about note off ;)
  if (velocity == 0)
  {
    return;
  }

  // Are we handling this channel?
  if ((our_channel == 0) || (our_channel == channel))
  {
    // Lookup note
    uint8_t slave_address, slave_channel;
    if (get_slave_and_channel(note, slave_address, slave_channel))
    {
      digitalWrite(led_pin, HIGH); // diagnostics
      send_chime(slave_address, slave_channel, scale_midi_velocity(velocity));
      digitalWrite(led_pin, LOW); // diagnostics
    }
  }
}

uint16_t scale_midi_velocity(const uint8_t& midi_velocity)
{
  // TODO: take master volume (& override) into account
  // TODO: scale MIDI velocity to chime volume somehow

  // Don't worry about MIDI velocity == 0, that must be taken care of before we
  // try and send a message at all.

  // For now, just linearly scaling chime velocity between minimum_dc and
  // maximum_dc.

  const float requested_velocity(static_cast<float>(midi_velocity & 0x7F) / 127.0);
  const float range(maximum_dc - minimum_dc);
  const uint16_t velocity((range * requested_velocity) + minimum_dc);
  return velocity;
}

void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity)
{
  // Packetize message
  uint8_t buffer[sizeof(uint8_t) + sizeof(uint16_t)];
  memcpy(buffer + 0, &channel, sizeof(uint8_t));
  memcpy(buffer + 1, &velocity, sizeof(uint16_t));

  // Send I2C message
  Wire.beginTransmission(address);
  Wire.write(buffer, sizeof(buffer));
  Wire.endTransmission();
}

bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out)
{
  for (uint8_t slave(0); slave < num_slaves; ++slave)
  {
    for (uint8_t channel(0); channel < notes_per_slave; ++channel)
    {
      if (note_map[slave][channel] == midi_note)
      {
        slave_address_out = slave_addresses[slave];
        slave_channel_out = channel;
        return true;
      }
    }
  }
  return false;
}

// Draws the master volume section (assumes blank screen)
void draw_master_volume()
{
  // Down button
  tft.fillRoundRect(button_vol_d_x, button_vol_d_y, button_vol_w, button_vol_h, button_radius, button_color);  // Button
  tft.drawRoundRect(button_vol_d_x, button_vol_d_y, button_vol_w, button_vol_h, button_radius, button_border); // Border

  // Up button
  tft.fillRoundRect(button_vol_u_x, button_vol_u_y, button_vol_w, button_vol_h, button_radius, button_color);  // Button
  tft.drawRoundRect(button_vol_u_x, button_vol_u_y, button_vol_w, button_vol_h, button_radius, button_border); // Border

  // Label
  tft.setFont(LiberationSans_12);
  tft.setTextColor(text_color);
  // Text is 12px tall, and 107px wide (I measured it). We want to center it.
  const int16_t text_w = 107, text_h = 12;
  const int16_t text_x = (320 - text_w) / 2, text_y = 240 - 5 - 50 - 5 - text_h;
  tft.drawFastHLine(0, text_y + (text_h / 2), text_x - 5, text_color);
  tft.setCursor(text_x, text_y);
  tft.print("Master Volume");
  tft.drawFastHLine(text_x + text_w + 5, text_y + (text_h / 2), 320 - text_x - text_w - 5, text_color);

  // Volume bar
  update_master_volume();
}

// Updates the master volume bar (assumes draw_master_volume() was run)
void update_master_volume()
{
  // Text offsets, because this font system sucks
  const int16_t offset_d_x = 10, offset_d_y = 18;
  const int16_t offset_u_x = 15, offset_u_y = 17;

  // Button font
  tft.setFont(AwesomeF000_24);

  // Down button
  if (master_volume > 0)
  {
    tft.setTextColor(button_text);
  }
  else
  {
    tft.setTextColor(button_text_disabled);
  }
  tft.setCursor(
    button_vol_d_x + (button_vol_w / 2) - offset_d_x,
    button_vol_d_y + (button_vol_h / 2) - offset_d_y);
  tft.print('\x27');

  // Up button
  if (master_volume < 100)
  {
    tft.setTextColor(button_text);
  }
  else
  {
    tft.setTextColor(button_text_disabled);
  }
  tft.setCursor(
    button_vol_u_x + (button_vol_w / 2) - offset_u_x,
    button_vol_u_y + (button_vol_h / 2) - offset_u_y);
  tft.print('\x28');

  // Buttons are 50 x 50, offset 5 from edge of screen
  // Bar should be between them, 5 padding from buttons
  const int16_t bar_x =  60, bar_y = 185;
  const int16_t bar_w = 200, bar_h =  50;
  const int16_t vol_w = master_volume * bar_w / 100;

  // Draw background
  if (master_volume < 100)
  {
    tft.fillRoundRect(bar_x, bar_y, bar_w, bar_h, button_radius, button_color);
  }

  // Draw volume level
  if (master_volume > 0)
  {
    tft.fillRoundRect(bar_x, bar_y, vol_w, bar_h, button_radius, volume_color);
    // TODO: square up right edges?
  }

  // Draw border
  tft.drawRoundRect(bar_x, bar_y, bar_w, bar_h, button_radius, button_border);

  // Current volume
  int16_t num_chars(3);
  if (master_volume < 10)
  {
    num_chars = 2;
  }
  else if (master_volume >= 100)
  {
    num_chars = 4;
  }
  tft.setFont(LiberationMono_32);
  tft.setTextColor(button_text);
  // Text is 32px tall, and characters are 26px wide (I measured it).
  // There ios a vertical offset of 2 pixels needed.
  const int16_t char_w = 26, char_h = 32;
  const int16_t text_w = num_chars * char_w;
  const int16_t text_x = (320 - text_w) / 2, text_y = 240 - 5 - 25 - (char_h / 2);
  // int x = 10, y = 10;
  // tft.drawFastHLine(0, y, 320, ILI9341_GREEN);
  // tft.drawFastHLine(0, y + char_h, 320, ILI9341_GREEN);
  // tft.drawFastVLine(x, 0, 240, ILI9341_GREEN);
  // tft.drawFastVLine(x + char_w, 0, 240, ILI9341_GREEN);
  // tft.drawFastVLine(x + 2 * char_w, 0, 240, ILI9341_GREEN);
  // tft.drawFastVLine(x + 3 * char_w, 0, 240, ILI9341_GREEN);
  // tft.drawFastVLine(x + 4 * char_w, 0, 240, ILI9341_GREEN);
  // tft.setCursor(x, y + 2);
  // tft.print("888%");

  // tft.drawFastHLine(0, text_y + (text_h / 2), text_x - 5, text_color);
  tft.setCursor(text_x, text_y + 2); // Offset because this font setup blows
  tft.print(master_volume);
  tft.print("%");
}

void UserPixelSetFunction(UG_S16 x, UG_S16 y, UG_COLOR c)
{
  tft.drawPixel(x, y, c);
}

UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
  // Vertical line
  if (x1 == x2)
  {
    tft.drawFastVLine(x1, y1, y2 - y1 + 1, c);
    return UG_RESULT_OK;
  }

  // Horizontal line
  if (y1 == y2)
  {
    tft.drawFastHLine(x1, y1, x2 - x1 + 1, c);
    return UG_RESULT_OK;
  }

  // Other lines can't be accelerated
  return UG_RESULT_FAIL;
}

UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c)
{
  tft.fillRect(x1, y1, x2 - x1 + 1, y2 - y1 + 1, c);
  return UG_RESULT_OK;
}

void glyph_test()
{
  // Test printing crap
  tft.setTextColor(ILI9341_YELLOW);
  // uint8_t padding(4);
  // uint8_t font_height(12);
  // tft.setCursor(padding, padding + 0 * (font_height + padding));
  // tft.setFont(LiberationSans_12);
  // tft.print("LiberationSans");
  // tft.setCursor(padding, padding + 1 * (font_height + padding));
  // tft.setFont(LiberationSans_12_Bold);
  // tft.print("LiberationSans Bold");
  // tft.setCursor(padding, padding + 2 * (font_height + padding));
  // tft.setFont(LiberationSans_12_Italic);
  // tft.print("LiberationSans Italic");
  // tft.setCursor(padding, padding + 3 * (font_height + padding));
  // tft.setFont(LiberationSans_12_Bold_Italic);
  // tft.print("LiberationSans Bold Italic");

  // tft.setFont(AwesomeF000_24);
  // tft.setCursor(10, 160);
  // tft.print('\x27'); // Volume Down Icon
  // tft.setCursor(160, 160);
  // tft.print((char)17); // Power Icon
  // tft.setCursor(220, 160);
  // tft.print((char)40); // Volume Up Icon

  // Learning character sizes
  tft.setFont(AwesomeF000_24);
  int16_t x, y = 100;

  // Line at y=100
  tft.drawFastHLine(0, y, 320, ILI9341_WHITE);

  // Test glyph alignment
  char glyph('\x7C');
  uint16_t w = 35, h = 26; // Claimed size
  int16_t offset_x = 0, offset_y = 3;

  // In box, offset
  x = 50;
  tft.drawFastVLine(x, 0, 240, ILI9341_WHITE);
  tft.drawRect(x, y, w, h, ILI9341_RED);
  tft.setCursor(x - offset_x, y - offset_y);
  tft.print(glyph);

  // In box, centered
  x = 100;
  offset_x += w / 2; // 0
  offset_y += h / 2; // 4
  tft.drawFastVLine(x, 0, 240, ILI9341_WHITE);
  tft.drawRect(x - (w / 2), y - (h / 2), w, h, ILI9341_RED);
  tft.setCursor(x - offset_x, y - offset_y);
  tft.print(glyph);

  // Centered
  x = 150;
  tft.drawFastVLine(x, 0, 240, ILI9341_WHITE);
  tft.setCursor(x - offset_x, y - offset_y);
  tft.print(glyph);

  // Display final offset
  tft.setFontAdafruit();
  tft.setCursor(0, 0);
  tft.print("Offset: ");
  tft.print(offset_x);
  tft.print(",");
  tft.println(offset_y);
}

bool hit_test(const int16_t& x, const int16_t& y, const int16_t& _x, const int16_t& _y, const int16_t& _w, const int16_t& _h)
{
  bool hit(true);
  if ((x < _x) || (x > (_x + _w)))
    hit = false;
  if ((y < _y) || (y > (_y + _h)))
    hit = false;

  // // DEBUG
  // tft.setTextColor(ILI9341_BLACK);
  // tft.setFontAdafruit();
  // tft.print("Hit testing (");
  // tft.print(x);
  // tft.print(",");
  // tft.print(y);
  // tft.println(")");
  // tft.print("against button (");
  // tft.print(_x);
  // tft.print(",");
  // tft.print(_y);
  // tft.print(") to (");
  // tft.print(_x + _w);
  // tft.print(",");
  // tft.print(_y + _h);
  // tft.println("):");
  // tft.println(hit);

  return hit;
}
