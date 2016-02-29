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
#define CAPACITIVE_TS

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------

// Core I/O (display, slaves, SD card)
#include <SPI.h>
#include <i2c_t3.h>
#include <ILI9341_t3.h>
#include <SdFat.h>

// GUI
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
// #include <font_LiberationMono.h>
// #include <font_LiberationSans.h>
// #include <font_LiberationSansBold.h>
// #include <font_LiberationSansItalic.h>
// #include <font_LiberationSansBoldItalic.h>
// #include <font_AwesomeF000.h>
#include "FontAwesome_mod_50X40.h"

// Blue Screen of Death :P
#include "bsod_win10.h"

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
const uint16_t console_bg(0x0000); // Windows 98+ #000000 --> RGB565
const uint16_t console_fg(0xC618); // Windows XP+ #C0C0C0 --> RGB565 (98 used #A8A8A8)
const uint16_t background_color(ILI9341_YELLOW);
const uint16_t button_text_disabled(C_GRAY);
const uint16_t selected_color(C_GREEN);

// Icons
const char fa_icon_settings[] = "u";       // Sliders
const char fa_icon_vol_dn[] = "F";         // Volume Down
const char fa_icon_vol_up[] = "G";         // Volume Up
const char fa_icon_level_up[] = "]";       // Level Up
const char fa_icon_level_down[] = "^";     // Level Down
const char fa_icon_folder_closed[] = "_";  // Folder, Closed (solid)
const char fa_icon_folder_open[] = "a";    // Folder, Open (solid)
const char fa_icon_file_generic[] = "d";   // File, Generic (outline)
const char fa_icon_file_picture[] = "k";   // File, Picture / Image (outline)
const char fa_icon_file_sound[] = "m";     // File, Sound / Music (outline)
const char fa_icon_file_config[] = "o";    // File, Code / Config (outline)

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

// Adjust Master Volume
void adjust_master_volume(int8_t change);

// µGUI
void UserPixelSetFunction(UG_S16 x, UG_S16 y, UG_COLOR c);
// µGUI Hardware Acceleration
UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);
UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);

// Main Window
void draw_main_window();
void main_callback(UG_MESSAGE* msg);

// Settings Window
void draw_settings_window();
void settings_callback(UG_MESSAGE* msg);

// File Browse window
void draw_fb_window();
void fb_window_callback(UG_MESSAGE* msg);
void fb_draw_highlight(int16_t line);
void update_file_list();


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// User settings
uint8_t our_channel(0); // Current channel (1-16, 0 means play all)
int8_t master_volume(100); // Master volume (0-100, 0 means mute, steps of 10)
bool override_velocity(false); // Override velocity to master volume (true), or scale it by master volume (false)
int16_t fb_selected_line(0); // Which line in the file browse list is currently highlighted

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
// Main Window
UG_WINDOW main_window;
UG_BUTTON main_window_button_settings;
UG_BUTTON main_window_button_browse;
UG_BUTTON main_window_button_vol_dn;
UG_BUTTON main_window_button_vol_up;
UG_PROGRESSBAR main_window_prb_volume;
UG_OBJECT main_window_buffer[5];
char volume_text_buffer[5] = { 0 };

// File Browse window
#define FB_LIST_SIZE 7
#define MAX_NAME 32
#define MAX_PATH 256
UG_WINDOW fb_window;
UG_BUTTON fb_up_button;
UG_BUTTON fb_down_button;
UG_BUTTON fb_cancel_button;
UG_BUTTON fb_select_button;
UG_TEXTBOX fb_current_path;
UG_TEXTBOX fb_file_list[FB_LIST_SIZE];
UG_OBJECT fb_window_buffer[FB_LIST_SIZE+6];
char selected_file_buffer[MAX_NAME] = { 0 };
char selected_path_buffer[MAX_PATH] = "/";
char file_list_buffer[FB_LIST_SIZE][MAX_NAME] = { 0 };

// Settings Window
UG_WINDOW settings_window;
UG_BUTTON settings_hw_accel_on_button;
UG_BUTTON settings_hw_accel_off_button;
UG_BUTTON settings_redraw_button;
UG_BUTTON settings_close_button;
UG_OBJECT obj_buff_settings_window[4];

// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  // Configure serial (debug)
  ser.begin(1500000);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Debug trigger
  digitalWrite(led_pin, HIGH);

  // Configure TFT
  tft.begin();
  tft.setRotation(1); // Landscape
  tft.fillScreen(background_color);

  digitalWrite(led_pin, LOW);
  // Configure GUI
  UG_Init(&gui, UserPixelSetFunction, tft.width(), tft.height());
  UG_DriverRegister(DRIVER_DRAW_LINE, (void*)_HW_DrawLine);
  UG_DriverRegister(DRIVER_FILL_FRAME, (void*)_HW_FillFrame);
  UG_FontSelect(&FONT_8X12); // Default font

  // Setup Console
  UG_ConsoleSetBackcolor(console_bg);
  UG_ConsoleSetForecolor(console_fg);
  UG_ConsolePutString("MIDI Chimes Booting\n");
  // TODO: put FW version or git hash or something?

  // Initialize Touch Screen
  UG_ConsolePutString("Starting touchscreen...");
  ser.print("ts.begin()");
  digitalWrite(led_pin, HIGH);
  if (!ts.begin())
  {
    draw_BSOD(tft);
    tft.println("Unable to start touchscreen.");
    while (1) { yield(); }
  }
  digitalWrite(led_pin, LOW);
  UG_ConsolePutString("done.\n");

  // Configure I2C
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  // TODO: check for fails

  // Initialize SD Card
  UG_ConsolePutString("Starting SD Card...");
  ser.print("sd.begin()");
  digitalWrite(led_pin, HIGH);
  if (!sd.begin(sd_cs_pin))
  {
    draw_BSOD(tft);
    sd.initErrorHalt(&tft);
  }
  digitalWrite(led_pin, LOW);
  UG_ConsolePutString("done.\n");

  // TODO: read config file

  // Configure USB MIDI
  usbMIDI.setHandleNoteOn(OnNoteOn);
  // usbMIDI.setHandleNoteOff(OnNoteOff); // Not needed
  // TODO: handle more MIDI stuff?

  // Draw Main Window
  draw_main_window();

  // file browse window
  draw_fb_window();

  // Settings window
  draw_settings_window();

  // Start GUI at main window
  UG_WindowShow(&main_window);
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

  // Update the GUI
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

// Change the master volume by the amount specified
void adjust_master_volume(int8_t change)
{
  // Adjust the volume
  master_volume += change;

  // Get current button setup
  const UG_COLOR w_fg(UG_WindowGetForeColor(&main_window));
  const UG_COLOR d_fg(UG_ButtonGetForeColor(&main_window, BTN_ID_1));
  const UG_U8    d_st(UG_ButtonGetStyle(&main_window, BTN_ID_1));
  const UG_COLOR u_fg(UG_ButtonGetForeColor(&main_window, BTN_ID_2));
  const UG_U8    u_st(UG_ButtonGetStyle(&main_window, BTN_ID_2));

  // Limit range to 0-100, enable/disable buttons
  if (master_volume >= 100)
  {
    master_volume = 100;

    // Disable volume up button
    if (u_fg != button_text_disabled) { UG_ButtonSetForeColor(&main_window, BTN_ID_2, button_text_disabled); }
    if (u_st != BTN_STYLE_2D)         { UG_ButtonSetStyle(&main_window, BTN_ID_2, BTN_STYLE_2D); }
  }
  else if (master_volume <= 0)
  {
    master_volume = 0;

    // Disable volume down button
    if (d_fg != button_text_disabled) { UG_ButtonSetForeColor(&main_window, BTN_ID_1, button_text_disabled); }
    if (d_st != BTN_STYLE_2D)         { UG_ButtonSetStyle(&main_window, BTN_ID_1, BTN_STYLE_2D); }
  }
  else
  {
    // Enable both buttons
    if (d_fg != w_fg         ) { UG_ButtonSetForeColor(&main_window, BTN_ID_1, w_fg); }
    if (d_st != BTN_STYLE_3D ) { UG_ButtonSetStyle(&main_window, BTN_ID_1, BTN_STYLE_3D); }
    if (u_fg != w_fg         ) { UG_ButtonSetForeColor(&main_window, BTN_ID_2, w_fg); }
    if (u_st != BTN_STYLE_3D ) { UG_ButtonSetStyle(&main_window, BTN_ID_2, BTN_STYLE_3D); }
  }

  // Update the text
  const UG_U8 last_vol(UG_ProgressbarGetValue(&main_window, PRB_ID_0));
  if (last_vol != master_volume)
  {
    sprintf(volume_text_buffer, "%d%%", master_volume);
    UG_ProgressbarSetText(&main_window, PRB_ID_0, volume_text_buffer);
    UG_ProgressbarSetValue(&main_window, PRB_ID_0, master_volume);
  }
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

void draw_main_window()
{
  // Window
  UG_WindowCreate(&main_window, main_window_buffer, sizeof(main_window_buffer) / sizeof(*main_window_buffer), main_callback);
  UG_WindowSetTitleTextAlignment(&main_window, ALIGN_CENTER);
  UG_WindowSetTitleText(&main_window, "MIDI Chimes");

  // Get usable size
  const uint16_t width(UG_WindowGetInnerWidth(&main_window));
  const uint16_t height(UG_WindowGetInnerHeight(&main_window));
  const uint16_t padding(5);
  const uint16_t button_size(50);

  // Settings button
  UG_ButtonCreate(&main_window, &main_window_button_settings, BTN_ID_0,
    padding,                // top-left x
    padding,                // top-left y
    padding + button_size,  // bottom-right x
    padding + button_size); // bottom-right y
  UG_ButtonSetFont(&main_window, BTN_ID_0, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&main_window, BTN_ID_0, fa_icon_settings);

  // Volume Down button
  UG_ButtonCreate(&main_window, &main_window_button_vol_dn, BTN_ID_1,
    padding,
    height - padding - button_size,
    padding + button_size,
    height - padding);
  UG_ButtonSetFont(&main_window, BTN_ID_1, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&main_window, BTN_ID_1, fa_icon_vol_dn);

  // Volume Up button
  UG_ButtonCreate(&main_window, &main_window_button_vol_up, BTN_ID_2,
    width - padding - button_size,
    height - padding - button_size,
    width - padding,
    height - padding);
  UG_ButtonSetFont(&main_window, BTN_ID_2, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&main_window, BTN_ID_2, fa_icon_vol_up);

  // Browse button
  UG_ButtonCreate(&main_window, &main_window_button_browse, BTN_ID_3,
    width - padding - button_size,
    padding,
    width - padding,
    padding + button_size);
  UG_ButtonSetFont(&main_window, BTN_ID_3, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&main_window, BTN_ID_3, fa_icon_folder_open);

  // Volume progress bar
  UG_ProgressbarCreate(&main_window, &main_window_prb_volume, PRB_ID_0,
    padding + button_size + padding,
    height - padding - button_size,
    width - padding - button_size - padding,
    height - padding);
  UG_ProgressbarSetFont(&main_window, PRB_ID_0, &FONT_24X40);
  UG_ProgressbarSetAlignment(&main_window, PRB_ID_0, ALIGN_CENTER);
  UG_ProgressbarSetBarColor(&main_window, PRB_ID_0, C_RED);
  adjust_master_volume(0);
}

void main_callback(UG_MESSAGE* msg)
{
  // TODO: Handle press and hold for volume keys?
  // I would need to do a rate-limit of some kind
  if ((msg->type == MSG_TYPE_OBJECT) &&
      (msg->id == OBJ_TYPE_BUTTON) &&
      (msg->event == BTN_EVENT_CLICKED))
  {
    switch (msg->sub_id)
    {
    // Settings
    case BTN_ID_0:
      UG_WindowShow(&settings_window);
      break;

    // Volume Down
    case BTN_ID_1:
      adjust_master_volume(-10);
      break;

    // Volume Up
    case BTN_ID_2:
      adjust_master_volume(10);
      break;

    // Browse button
    case BTN_ID_3:
      UG_WindowShow(&fb_window);
      break;

    default:
      break;
    }
  }
  // TODO: look into owner-drawing the volume text box using the pre-draw events or post-draw events
  // This might let me do the progress bar again
}

void draw_settings_window()
{
  UG_WindowCreate(&settings_window, obj_buff_settings_window, sizeof(obj_buff_settings_window) / sizeof(*obj_buff_settings_window), settings_callback);
  UG_WindowResize(&settings_window, 20, 20, 319-20, 239-20);
  UG_WindowSetTitleText(&settings_window, "\xE6GUI Test Window");
  UG_WindowSetTitleTextFont(&settings_window, &FONT_8X12);
  UG_ButtonCreate(&settings_window, &settings_hw_accel_on_button, BTN_ID_0, 10, 10, 100,  60);
  UG_ButtonSetFont(&settings_window, BTN_ID_0, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_0, "H/W Acc\nON");
  UG_ButtonSetBackColor(&settings_window, BTN_ID_0, C_GREEN);
  UG_ButtonCreate(&settings_window, &settings_hw_accel_off_button, BTN_ID_1, 10, 70, 100, 130);
  UG_ButtonSetFont(&settings_window, BTN_ID_1, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_1, "H/W Acc\nOFF");
  UG_ButtonCreate(&settings_window, &settings_redraw_button, BTN_ID_2, 110, 10, 200, 60);
  UG_ButtonSetFont(&settings_window, BTN_ID_2, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_2, "Redraw");
  UG_ButtonCreate(&settings_window, &settings_close_button, BTN_ID_3, 110, 70, 200, 130);
  UG_ButtonSetFont(&settings_window, BTN_ID_3, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_3, "Close");
}

void settings_callback(UG_MESSAGE* msg)
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
      UG_ButtonSetBackColor(&settings_window, BTN_ID_0, selected_color);
      UG_ButtonSetBackColor(&settings_window, BTN_ID_1, UG_WindowGetBackColor(&settings_window));
      break;

    case BTN_ID_1:
      UG_DriverDisable(DRIVER_DRAW_LINE);
      UG_DriverDisable(DRIVER_FILL_FRAME);
      UG_ButtonSetBackColor(&settings_window, BTN_ID_0, UG_WindowGetBackColor(&settings_window));
      UG_ButtonSetBackColor(&settings_window, BTN_ID_1, selected_color);
      break;

    case BTN_ID_2:
      UG_WindowHide(&settings_window);
      UG_WindowShow(&settings_window);
      break;

    case BTN_ID_3:
      UG_WindowHide(&settings_window);
      break;

    default:
      break;
    }
  }
}

void draw_fb_window()
{
  // Window setup
  const uint16_t window_padding(10);
  UG_WindowCreate(&fb_window, fb_window_buffer, sizeof(fb_window_buffer) / sizeof(*fb_window_buffer), fb_window_callback);
  UG_WindowResize(&fb_window,
    window_padding,
    window_padding*2,
    UG_WindowGetOuterWidth(&main_window) - window_padding,
    UG_WindowGetOuterHeight(&main_window) - window_padding);
  UG_WindowSetTitleTextAlignment(&fb_window, ALIGN_CENTER);
  UG_WindowSetTitleText(&fb_window, "Select File");
  UG_WindowSetTitleTextFont(&fb_window, &FONT_8X12);

  // UI layout variables
  const uint16_t width(UG_WindowGetInnerWidth(&fb_window));
  const uint16_t height(UG_WindowGetInnerHeight(&fb_window));
  const uint16_t padding(5);
  const uint16_t button_size(50);

  // Scroll up button (top right)
  UG_ButtonCreate(&fb_window, &fb_up_button, BTN_ID_0,
    width - padding - button_size,
    padding,
    width - padding,
    padding + button_size);
  UG_ButtonSetFont(&fb_window, BTN_ID_0, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&fb_window, BTN_ID_0, fa_icon_level_up);

  // Scroll down button (bottom right)
  UG_ButtonCreate(&fb_window, &fb_down_button, BTN_ID_1,
    width - padding - button_size,
    height - padding - button_size,
    width - padding,
    height - padding);
  UG_ButtonSetFont(&fb_window, BTN_ID_1, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&fb_window, BTN_ID_1, fa_icon_level_down);

  // Cancel/close button (top left)
  UG_ButtonCreate(&fb_window, &fb_cancel_button, BTN_ID_2,
    padding,
    padding,
    padding + button_size,
    padding + button_size);
  UG_ButtonSetFont(&fb_window, BTN_ID_2, &FONT_24X40);
  UG_ButtonSetText(&fb_window, BTN_ID_2, "X");

  // Select button (center right)
  UG_ButtonCreate(&fb_window, &fb_select_button, BTN_ID_3,
    width - padding - button_size,
    padding + button_size + padding,
    width - padding,
    height - padding - button_size - padding);
  UG_ButtonSetFont(&fb_window, BTN_ID_3, &FONT_24X40);
  UG_ButtonSetText(&fb_window, BTN_ID_3, ">");

  // Get the file list from the SD card
  update_file_list();

  const uint16_t line_height = 16;  // Height of a single line in the file list. Depends on font
  // File list (main area)
  for (int i = 0; i < FB_LIST_SIZE; i++)
  {
    uint8_t txb_id = i;
    UG_TextboxCreate(&fb_window, &fb_file_list[i], txb_id,
      padding,
      padding + button_size + padding + (i*line_height),
      width - padding - button_size - padding,
      padding + button_size + padding + ((i+1)*line_height));
    UG_TextboxSetFont(&fb_window, txb_id, &FONT_10X16);
    UG_TextboxSetAlignment(&fb_window, txb_id, ALIGN_TOP_LEFT);
    UG_TextboxSetBackColor(&fb_window, txb_id, C_WHITE);
    UG_TextboxSetForeColor(&fb_window, txb_id, C_BLACK);

    // Debug output
    // sprintf(file_list_buffer[i], "file%d.mid", txb_id);
  }

  // Current path text box (top center)
  UG_TextboxCreate(&fb_window, &fb_current_path, (FB_LIST_SIZE+1),
    padding + button_size + padding,
    padding,
    width - padding - button_size - padding,
    padding + button_size);
  UG_TextboxSetFont(&fb_window, (FB_LIST_SIZE+1), &FONT_10X16);
  UG_TextboxSetAlignment(&fb_window, (FB_LIST_SIZE+1), ALIGN_CENTER);
  UG_TextboxSetBackColor(&fb_window, (FB_LIST_SIZE+1), C_WHITE);
  UG_TextboxSetForeColor(&fb_window, (FB_LIST_SIZE+1), C_BLACK);
  UG_TextboxSetText(&fb_window, (FB_LIST_SIZE+1), "active_file.mid");

  // Highlight the first line
  fb_draw_highlight(0);
}

void fb_window_callback(UG_MESSAGE* msg)
{
  if ((msg->type == MSG_TYPE_OBJECT) &&
      (msg->id == OBJ_TYPE_BUTTON) &&
      (msg->event == BTN_EVENT_CLICKED))
  {
    switch (msg->sub_id)
    {
    case BTN_ID_0:  // up button
      fb_draw_highlight(fb_selected_line - 1);
      break;
      
    case BTN_ID_1:  // down button
      fb_draw_highlight(fb_selected_line + 1);
      break;
      
    case BTN_ID_2:  // Close window
      UG_WindowHide(&fb_window);
      break;

    case BTN_ID_3:  // Select file/folder
      UG_TextboxSetText(&fb_window, (FB_LIST_SIZE+1), UG_TextboxGetText(&fb_window, fb_selected_line));
      break;
      
    default:
      break;
    }
  }
}

void fb_draw_highlight(int16_t selected_line)
{
  // Deselect the previously selected line
  UG_TextboxSetBackColor(&fb_window, fb_selected_line, C_WHITE);

  // Change the selectionto the newly selected line
  fb_selected_line = selected_line;

  // Limit the selection max and min.
  if (fb_selected_line <= 0)
  { 
    // TODO: Scroll through directory entries
    update_file_list();
    fb_selected_line = 0;
  }
  if (fb_selected_line >= FB_LIST_SIZE-1)
  {
    // TODO: Scroll through directory entries
    update_file_list();
    fb_selected_line = FB_LIST_SIZE-1;
  }

  // Redraw highlight line in new location
  UG_TextboxSetBackColor(&fb_window, fb_selected_line, selected_color);

  // Update file list text
  for(int i = 0; i < FB_LIST_SIZE; i++)
  {
    UG_TextboxSetText(&fb_window, i, file_list_buffer[i]);
  }

  // Debug output
  sprintf(selected_file_buffer, "sel: %d", fb_selected_line); 
  UG_TextboxSetText(&fb_window, (FB_LIST_SIZE+1), selected_file_buffer);
}

void update_file_list()
{
  // List files in directory.
  ser.println("Calling dirFile.open()"); // DEBUG
  if (!dirFile.isOpen() && !dirFile.open("/", O_READ))
  {
    draw_BSOD(tft);
    sd.errorHalt(&tft, "open root failed");
  }

  uint16_t files_found(0);
  const uint16_t nMax(FB_LIST_SIZE); // Max files to list
  while (files_found < nMax && file.openNext(&dirFile, O_READ))
  {
    // Skip directories and hidden files.
    if (file.isHidden())
    {
      // skip
    }
    else if(file.isSubDir())
    {
      // TODO: Show entry,, but with an icon or something
    }
    else
    {
      file.getName(file_list_buffer[files_found], MAX_NAME);
    }

    files_found++;
    file.close();
  }
}
