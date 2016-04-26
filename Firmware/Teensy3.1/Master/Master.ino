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
// * This could from an I2C timeout, which I am not yet handling.
// *****************************************************************************
// * POST
//   * After assigning slaves I2C addresses, verify the channels are all as expected:
//     * Enable PS & let it settle
//     * Tell slaves to begin normal operation (so they self-test)
//     * Wait for that to complete (fixed delay, polling, ???)
//     * Read slave status
//     * Make sure only NC channels are "channel_disconnected" (1)
//     * All other channels must be "channel_working" (0)
// * Velocity map (MIDI is 0-127, I need a PWM value ~50%-100%)
//   * Working, but I should calibrate PWM value to chime volume with a SPL meter
//   * Per-note velocity map? This would make the chimes perfectly linear
//   * Or a velocity formula (per note?) might be better?
// * Actually implement master volume
// * See Notes/MIDI.md for channel, program, and OUT/THRU information.
// * Implement I2C timeout (properly) & handle I2C errors
//   * Probably need a very short timeout (1ms or less) to avoid messing up song
//     * Or should we just throw a giant error and abort playback?
//       We can only do this for SD card playback, though.
// * Implement settings
//   * Store them to the SD card
// * Handle MIDI program change events
//   * We are technically Program 0xE "Tubular Bells"
//   * We might want to handle other things, or everything on the channel?
// * Handle MIDI Control Change messages
//   * 0x07 is channel volume. Use that to adjust our volume
// * Handle some System Exclusive messages
//   * Master volume (F0 7F 7F 04 01 00 nn F7 where nn is volume 0-127)
//     * This should probably be a setting, but it seems pretty safe.
// * Make the display do useful things
//   * Show errors
//   * Show input source (USB or physical MIDI)
//   * Options
//     * Override velocity to master volume vs. scale velocity 0-master
//       * Default should probably be scale
//     * Physical MIDI port is OUT or THRU
//     * Pass USB MIDI to physical MIDI OUT? (or should I always just do this?)
// * Read from SD card
//   * Basic file browser?
// * Playback of MIDI files (from SD card)
//   * Play doorbel tone from sd "doorbell" folder :P
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
#include "FontAwesome_mod_50X40.h"

// Blue Screen of Death :P
#include "bsod_win10.h"

// Config file
#include "config_file.h"

using namespace midi_chimes;

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I2C config
const uint8_t i2c_slave_base_address(0x10); // Starting address for slaves
const uint8_t i2c_slave_ack(0x06); // ACK sentinel expected from the slaves

// Settings file
const char ini_filename[] = "/Settings.ini";

// Note map (loaded from config file)
// MIDI spec only allows a maximum of 128 notes, so we will just build a map of
// all possibilities
slave_note_map note_map[128] = {};
const size_t notes_per_slave(10);

// Duty Cycle Settings
// This matches the slaves, and isn't directly used by the master
const uint8_t pwm_bits(12); // 0 - 4095
const float minimum_pwm(0.55); // 55%, lowest reliable impact to produce a chime
const uint16_t maximum_dc((1 << pwm_bits) - 1);
const uint16_t minimum_dc(minimum_pwm * maximum_dc);

// Pins
const uint8_t spi_mosi_pin(11);
const uint8_t spi_miso_pin(12);
const uint8_t spi_sck_pin(14);
const uint8_t sd_cs_pin(21);
const uint8_t doorbell_pin(2);
const uint8_t ps_en_pin(8);
const uint8_t addr_latch_pin(10);
const uint8_t led_pin(13);

// Touch Screen
// Pins
const uint8_t lcd_reset_pin(17);
const uint8_t lcd_cs_pin(15);
const uint8_t lcd_dc_pin(9);
const uint8_t touch_cs_pin(20);
// Backlight
const uint8_t lcd_backlight_pin(23);
const uint8_t lcd_bl_pwm_bits(16);
const uint32_t lcd_bl_pwm_freq(549); // Hz
#ifndef CAPACITIVE_TS
// Resistive Touch Screen Calibration
const uint16_t ts_min_x( 150);
const uint16_t ts_max_x(3800);
const uint16_t ts_min_y( 130);
const uint16_t ts_max_y(4000);
#endif

// Timeouts
const uint32_t blink_time(20000); // microseconds
const uint32_t ps_en_toggle_time(1000); // microseconds
const uint32_t ps_settle_time(2500); // milliseconds
const uint32_t sleep_time(5 * 1000); // milliseconds

// Backlight brightness
const uint16_t lcd_bl_pwm_max(0xFFFF);
const uint16_t lcd_bl_pwm_min(0x0C00);
const uint16_t lcd_bl_pwm_step((lcd_bl_pwm_max - lcd_bl_pwm_min) / ps_settle_time);
// We want to ramp up the backlight while we wait for the power supply to settle.

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
const char fa_icon_sleep_leaf[] = "C";     // Leaf (sleep)

// MIDI Commands
const uint8_t midi_cmd_note_off(0x80);
const uint8_t midi_cmd_note_on(0x90);
const uint8_t midi_cmd_aftertouch(0xA0);
const uint8_t midi_cmd_control_change(0xB0);
const uint8_t midi_cmd_program_change(0xC0);
const uint8_t midi_cmd_aftertouch_mono(0xD0);
const uint8_t midi_cmd_pitch_bend(0xE0);
const uint8_t midi_cmd_system_exclusive(0xF0);

// MIDI Programs (Instruments)
const uint8_t midi_prog_tubular_bells(0xE);

// Power state of master
enum power_state_t
{
  awake,
  sleep,
  awake_to_sleep_transition,
  sleep_to_awake_transition,
} power_state;

// Channel states of slave channels
enum channel_state_t : uint8_t
{
  channel_working = 0,
  channel_disconnected = 1,
  channel_failed_short = 2,
  channel_failed_open = 3,
};


// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
// void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity); // Not needed
void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnVelocityChange(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnControlChange(uint8_t channel, uint8_t control, uint8_t value);
void OnProgramChange(uint8_t channel, uint8_t program);
// void OnAfterTouch(uint8_t channel, uint8_t pressure);
// void OnPitchChange(uint8_t channel, int pitch);

// Scale a MIDI velocity (7-bit) to our 12-bit velocity
uint16_t scale_midi_velocity(const uint8_t& midi_velocity, const uint8_t& note);

// Send a chime strike command to a slave
void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity);

// Gets the slave address and channel on the slave for a given MIDI note
bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out);

// Adjust Master Volume
void adjust_master_volume(int8_t change);

// Auto-assign I2C addresses to slaves
uint8_t i2c_addr_auto_assign();
bool i2c_check_ack(const uint8_t& address);

void print_channel_state_string(Print& out, const channel_state_t& state);

// Sleep mode functions
void power_activity(); // Activity detected, exit sleep mode & reset timer
void power_state_update(); // Update power state, display brightness, etc.
bool backlight_up(); // Increases backlight
bool backlight_down(); // Decreases backlight
void begin_sleep(); // Begin sleep process
void begin_wake(); // Begin wakeup process
void wake_complete(); // Finish wakeup process

// µGUI
void UserPixelSetFunction(UG_S16 x, UG_S16 y, UG_COLOR c);
// µGUI Hardware Acceleration
UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);
UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR c);

// Main window
void draw_main_window();
void main_callback(UG_MESSAGE* msg);

// Settings window
void draw_settings_window();
void settings_callback(UG_MESSAGE* msg);

// File Browse window
void draw_fb_window();
void fb_window_callback(UG_MESSAGE* msg);
void fb_draw_highlight(int16_t line);

// Sleep window
void draw_sleep_window();
void sleep_callback(UG_MESSAGE* msg);


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// User settings
config_file settings = config_file(ini_filename); // Hardware settings
uint8_t our_channel(0); // Current channel (0-15)
int8_t master_volume(100); // Master volume (0-100, 0 means mute, steps of 10)
bool override_velocity(false); // Override velocity to master volume (true), or scale it by master volume (false)
bool play_all_programs(false); // Play all programs (instruments), or just Tubular Bells?

int16_t fb_selected_line(0); // Which line in the file browse list is currently highlighted

// Power supply
bool ps_enabled(true); // Is the power supply enabled?
bool ps_en_high(false); // Is the PS_EN pin high currently?

// Backlight
uint16_t backlight_pwm(lcd_bl_pwm_max);

// MIDI state
bool play_this_program(true); // Do we play notes for the current program?

// Timers
elapsedMicros ps_en_timer;
elapsedMillis sleep_timer;

// Serial IO
HardwareSerial midi = HardwareSerial();
HardwareSerial3 ser = HardwareSerial3(); // For debug

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
UG_WINDOW fb_window;
UG_BUTTON fb_up_button;
UG_BUTTON fb_down_button;
UG_BUTTON fb_cancel_button;
UG_BUTTON fb_select_button;
UG_TEXTBOX fb_current_path;
UG_TEXTBOX fb_file_list[FB_LIST_SIZE];
UG_OBJECT fb_window_buffer[FB_LIST_SIZE+6];
char selected_file_buffer[32] = { 0 };
char file_list_buffer[FB_LIST_SIZE][32] = { 0 };

// Settings Window
UG_WINDOW settings_window;
UG_BUTTON settings_ps_on_button;
UG_BUTTON settings_ps_off_button;
UG_BUTTON settings_redraw_button;
UG_BUTTON settings_close_button;
UG_CHECKBOX settings_override_velocity_checkbox;
UG_CHECKBOX settings_play_all_programs_checkbox;
UG_OBJECT obj_buff_settings_window[6];

// Sleep window
UG_WINDOW sleep_window;
UG_TEXTBOX sleep_icon;
UG_TEXTBOX sleep_text;
UG_OBJECT obj_buff_sleep_window[2];

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

  // Configure address latch pin
  pinMode(addr_latch_pin, OUTPUT);
  digitalWrite(addr_latch_pin, LOW);

  // Configure PS_EN
  pinMode(ps_en_pin, OUTPUT);
  digitalWrite(ps_en_pin, LOW);
  ps_en_high = false;

  // Configure LCD backlight
  pinMode(lcd_backlight_pin, OUTPUT);
  analogWriteResolution(lcd_bl_pwm_bits);
  analogWriteFrequency(lcd_backlight_pin, lcd_bl_pwm_freq);
  analogWrite(lcd_backlight_pin, backlight_pwm);

  // Configure TFT
  tft.begin();
  tft.setRotation(1); // Landscape
  tft.fillScreen(background_color);

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
  if (!ts.begin())
  {
    draw_BSOD(tft);
    tft.println("Unable to start touchscreen.");
    halt_system();
  }
  UG_ConsolePutString("done.\n");

  // Configure I2C
  Wire1.begin(I2C_MASTER, 0, I2C_PINS_29_30, I2C_PULLUP_EXT, I2C_RATE_1000);

  // Initialize SD Card
  UG_ConsolePutString("Starting SD Card...");
  if (!sd.begin(sd_cs_pin))
  {
    draw_BSOD(tft);
    sd.initErrorHalt(&tft);
  }
  UG_ConsolePutString("done.\n");

  // Read config file
  UG_ConsolePutString("Opening Settings.ini...");
  if (!settings.open())
  {
    draw_BSOD(tft);
    tft.print("Could not open INI file ");
    tft.println(ini_filename);
    halt_system();
  }
  if (!settings.validate())
  {
    draw_BSOD(tft);
    tft.print("INI file ");
    tft.print(ini_filename);
    tft.print(" not valid: ");
    settings.print_ini_error_message(tft);
    halt_system();
  }
  UG_ConsolePutString("done.\n");

  // Load configuration
  UG_ConsolePutString("Loading settings...");
  // How many slaves should we have?
  int8_t slaves_expected(0);
  if (!settings.get_slave_count(slaves_expected))
  {
    draw_BSOD(tft);
    tft.println("Could not find any slave configuration lines!");
    tft.println("Please verify the INI file.");
    halt_system();
  }
  // Load note map
  for (int8_t slave(0); slave < slaves_expected; ++slave)
  {
    int8_t notes[notes_per_slave];
    if (!settings.get_slave_notes(slave, notes))
    {
      draw_BSOD(tft);
      tft.print("Unable to process slave note settings for Slave");
      tft.print(slave);
      tft.println(".");
      tft.println("Verify note assignments in the INI file.");
      halt_system();
    }
    for (uint8_t channel(0); channel < notes_per_slave; ++channel)
    {
      const int8_t note(notes[channel]);
      if (note == -1)
      {
        // Nothing on this channel
        continue;
      }
      note_map[note].slave_address = i2c_slave_base_address + slave;
      note_map[note].channel = channel;
      // Load calibration
      float calibration;
      if (settings.get_note_calibration(note, calibration))
      {
        note_map[note].calibration = calibration;
      }
    }
  }
  UG_ConsolePutString("done.\n");

  // Configure slave addresses
  UG_ConsolePutString("Detecting slave boards...");
  const uint8_t slaves_found(i2c_addr_auto_assign());
  if (slaves_found != slaves_expected)
  {
    draw_BSOD(tft);
    tft.println("Incorrect number of slave boards found!");
    tft.print("Expected: ");
    tft.println(slaves_expected);
    tft.print("Found:    ");
    tft.println(slaves_found);
    tft.println();
    tft.println("Check the cables are attached & oriented correctly.");
    tft.println("Check the INI file has the correct settings.");
    tft.println("Remove all power before retrying.");
    // halt_system();
    // ^^^ TODO: disabled for debug, re-enable when I have all the slaves
  }
  UG_ConsolePutString("done.\n");

  // POST slaves
  UG_ConsolePutString("Performing POST on slave boards...");
  // needs a timer
  // TODO: Start power supply to allow slaves to complete POST
  // TODO: Get slave status
  //auto slave_status[slaves_found];
  UG_ConsolePutString("done.\n");

  // Verify all channels are working
  UG_ConsolePutString("Verifying channels are working...");
  // Check all mapped notes to ensure they are working
  for (uint8_t note(0); note < (sizeof(note_map) / sizeof(note_map[0])); ++note)
  {
    // Only check notes that are connected
    if (note_map[note].slave_address)
    {
      // Get slave & channel
      const uint8_t slave_no(note_map[note].slave_address - i2c_slave_base_address);
      const uint8_t channel(note_map[note].channel);

      // TODO: read slave_status[slave_no], get channel
      channel_state_t state(channel_working); // TODO: really get this
      if (state != channel_working)
      {
        draw_BSOD(tft);
        tft.println("Channel malfunction found!");
        tft.print("Slave:   ");
        tft.println(slave_no);
        tft.print("Channel: ");
        tft.println(channel + 1);
        tft.print("Status:  ");
        tft.print(static_cast<uint8_t>(state), DEC);
        tft.print(" (");
        print_channel_state_string(tft, state);
        tft.println(")");
        tft.println();
        tft.println("Check the solenoid cable is attached correctly.");
        tft.println("Check the INI file has the correct settings.");
        tft.println("Remove all power before retrying.");
        halt_system();
      }
    }
  }
  // Check all connected channels for shorts (even ones we aren't expecting to be connected)
  for (uint8_t slave(0); slave < slaves_found; ++slave)
  {
    for (int channel(0); channel < notes_per_slave; ++channel)
    {
      // TODO: read slave_status[slave_no], get channel
      channel_state_t state(channel_working); // TODO: really get this

      // Verify the channel is working
      if (state == channel_failed_short)
      {
        draw_BSOD(tft);
        tft.println("Shorted channel found!");
        tft.print("Slave:   ");
        tft.println(slave_no);
        tft.print("Channel: ");
        tft.println(channel + 1);
        tft.print("Status:  ");
        tft.print(static_cast<uint8_t>(state), DEC);
        tft.print(" (");
        print_channel_state_string(tft, state);
        tft.println(")");
        tft.println();
        tft.println("Even though this channel isn't mapped, it must not");
        tft.println("be shorted, as that can damage the power supply and");
        tft.println("coil.");
        tft.println();
        tft.println("Check the INI file has the correct settings.");
        tft.println("Remove all power before retrying.");
        halt_system();
      }
    }
  }
  UG_ConsolePutString("done.\n");

  // Configure USB MIDI
  // usbMIDI.setHandleNoteOff(OnNoteOff); // Not needed (might for OUT/THRU?)
  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleProgramChange(OnProgramChange);
  // TODO: handle more MIDI stuff?

  // Configure hardware MIDI
  midi.begin(31250, SERIAL_8N1);

  // Draw Main Window
  draw_main_window();

  // file browse window
  draw_fb_window();

  // Settings window
  draw_settings_window();

  // Sleep window
  draw_sleep_window();

  // Start GUI at main window
  UG_WindowShow(&main_window);

  // Set initial power state
  power_state = awake;
  sleep_timer = 0;
}

// Main program loop
void loop()
{
  // Manage power state (sleep)
  power_state_update();

  // Handle power supply
  if (ps_en_timer > ps_en_toggle_time)
  {
    // Just reset the timer. A bit of jitter isn't a problem, but wind-up is.
    ps_en_timer = 0;

    // Toggle PS_EN state?
    if (ps_enabled)
    {
      ps_en_high = !ps_en_high;
    }
    else
    {
      ps_en_high = false;
    }

    // Write to the pin
    digitalWrite(ps_en_pin, ps_en_high ? HIGH : LOW);
  }

  // Handle USB MIDI messages
  usbMIDI.read();

  // Handle hardware MIDI messages
  int midi_bytes_available;
  while ((midi_bytes_available = midi.available()) > 0)
  {
    power_activity();

    const uint8_t status(midi.peek());

    // Drop non-command data until the buffer starts with a command
    if (status < 0x80) // All command bytes start with a 1
    {
      midi.read(); // Drop the byte from the buffer
      continue;
    }

    // Parse
    const uint8_t command(status & 0xF0); // Mask off channel
    const uint8_t channel(status & 0x0F); // Mask off command

    // Find how many data bytes we need
    int required_data_bytes(0);
    switch (command)
    {
    // 2 byte commands
    case midi_cmd_note_off:
    case midi_cmd_note_on:
    case midi_cmd_aftertouch:
    case midi_cmd_control_change:
    case midi_cmd_pitch_bend:
      required_data_bytes = 2;
      break;

    // 1 byte commands
    case midi_cmd_program_change:
    case midi_cmd_aftertouch_mono:
      required_data_bytes = 1;
      break;

    // Unknown commands
    default:
      break;
    }

    // Verify we have enough data
    if (midi_bytes_available <= required_data_bytes)
    {
      break;
    }

    // Read command & data
    midi.read(); // Drop command from buffer, we already know what it is
    uint8_t midi_buffer[required_data_bytes];
    bool valid(true);
    for (int i(0); i < required_data_bytes; ++i)
    {
      // Validate byte is not a command (data values all start with 0)
      if (midi.peek() >= 0x80)
      {
        // We didn't get the correct number of data bytes! Drop what we have so
        // far and start again on the next command
        valid = false;
        break;
      }

      // Read the data byte
      midi_buffer[i] = midi.read();
    }
    if (valid)
    {
      // Call appropriate function
      switch (command)
      {
      // Note On
      case midi_cmd_note_on:
        OnNoteOn(channel, midi_buffer[0], midi_buffer[1]);
        break;

      case midi_cmd_program_change:
        OnProgramChange(channel, midi_buffer[0]);
        break;

      // Not implemented
      case midi_cmd_note_off:
      case midi_cmd_aftertouch:
      case midi_cmd_control_change:
      case midi_cmd_aftertouch_mono:
      case midi_cmd_pitch_bend:
        break;

      // Unknown commands
      default:
        break;
      }
    }
  }

  // Handle GUI & Touch
  const bool touched(ts.touched());
#ifdef CAPACITIVE_TS
  if (touched)
  {
    power_activity();

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
    power_activity();

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
  power_activity();

  // MIDI spec allows turning a note off by sending a note on with velocity = 0
  // We don't care about note off ;)
  if (velocity == 0)
  {
    return;
  }

  // Are we handling this channel & program (instrument)?
  if ((channel == our_channel) && play_this_program)
  {
    // Lookup note
    uint8_t slave_address, slave_channel;
    if (get_slave_and_channel(note, slave_address, slave_channel))
    {
      digitalWrite(led_pin, HIGH); // diagnostics
      send_chime(slave_address, slave_channel, scale_midi_velocity(velocity, note));
      digitalWrite(led_pin, LOW); // diagnostics
    }
  }
  else
  {
    // TODO: MIDI out/thru
  }
}

void OnProgramChange(uint8_t channel, uint8_t program)
{
  power_activity();

  if (channel == our_channel)
  {
    play_this_program = play_all_programs || (program == midi_prog_tubular_bells);
  }
  else
  {
    // TODO: MIDI out/thru
  }
}

uint16_t scale_midi_velocity(const uint8_t& midi_velocity, const uint8_t& note)
{
  // Don't worry about MIDI velocity == 0, that must be taken care of before we
  // try and send a message at all.

  // TODO: it might be worth re-working all the math as int32, as it might be
  //       faster than floats. But it might not matter.

  // Calculate requested velocity
  const float range(maximum_dc - minimum_dc);
  float requested_volume(static_cast<float>(midi_velocity & 0x7F) / 127.0);

  // If we are overriding velocity, use the current volume level
  if (override_velocity)
  {
    requested_volume = static_cast<float>(master_volume) / 100.0;
  }

  // Handle calibration
  const float& calibration(note_map[note].calibration);

  // Calculate final velocity DC value
  const uint16_t velocity(minimum_dc + (range * requested_volume * calibration));
  return velocity;
}

void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity)
{
  // Packetize message
  uint8_t buffer[sizeof(uint8_t) + sizeof(uint16_t)];
  memcpy(buffer + 0, &channel, sizeof(uint8_t));
  memcpy(buffer + 1, &velocity, sizeof(uint16_t));

  // Send I2C message
  Wire1.beginTransmission(address);
  Wire1.write(buffer, sizeof(buffer));
  Wire1.endTransmission();
}

bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out)
{
  // Sanity check input
  if (midi_note > (sizeof(note_map) / sizeof(note_map[0])))
  {
    return false;
  }

  // Lookup values
  slave_address_out = note_map[midi_note].slave_address;
  slave_channel_out = note_map[midi_note].channel;

  // Valid if slave address is non-zero
  return (slave_address_out != 0);
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

uint8_t i2c_addr_auto_assign()
{
  const uint16_t slave_latch_delay(100); // Microseconds
  uint8_t slaves_found(0);
  uint8_t slave_address(i2c_slave_base_address);

  // Assign first slave's address (directly attached)
  Wire1.beginTransmission(0);
  Wire1.write(0x00); // Command = Set Address
  Wire1.write(slave_address);
  Wire1.endTransmission();

  // Latch address
  digitalWrite(addr_latch_pin, HIGH);
  delayMicroseconds(slave_latch_delay);
  digitalWrite(addr_latch_pin, LOW);

  // Check for ACK
  bool gotSlave(i2c_check_ack(slave_address++));
  if (gotSlave)
  {
    slaves_found++;
  }

  // Handle remaining slaves (daisy chained)
  while (gotSlave)
  {
    // Send next address
    Wire1.beginTransmission(0);
    Wire1.write(0x00); // Command = Set Address
    Wire1.write(slave_address);
    Wire1.endTransmission();

    // Ask previous slave to latch the current slave for us
    Wire1.beginTransmission(slave_address - 1);
    Wire1.write(0x01); // Command = Set ADDR_LATCH
    Wire1.write(HIGH);
    Wire1.endTransmission();
    delayMicroseconds(slave_latch_delay);
    Wire1.beginTransmission(slave_address - 1);
    Wire1.write(0x01); // Command = Set ADDR_LATCH
    Wire1.write(LOW);
    Wire1.endTransmission();

    // Check for ACK
    gotSlave = i2c_check_ack(slave_address++);
    if (gotSlave)
    {
      slaves_found++;
    }
  };

  // Tell slaves startup is complete (allow them to begin their POST)
  for (int i(0); i < slaves_found; ++i)
  {
    Wire1.beginTransmission(i2c_slave_base_address + i);
    Wire1.write(0x02); // Command = Startup Complete
    Wire1.write(0x00); // Don't care
    Wire1.endTransmission();
  }

  // Done!
  return slaves_found;
}

bool i2c_check_ack(const uint8_t& address)
{
  // Look for a single ACK byte
  if (Wire1.requestFrom(address, 1, I2C_STOP, 100) == 1) // We got 1 byte
  {
    // Is the byte an ACK?
    return (Wire1.read() == i2c_slave_ack);
  }

  return false;
}

void print_channel_state_string(Print& out, const channel_state_t& state)
{
  switch (state)
  {
  case channel_working:
    tft.print("Working Normally");
    break;

  case channel_disconnected:
    tft.print("Disconnected / Solenoid Not Found");
    break;

  case channel_failed_short:
    tft.print("Short-Circuited");
    break;

  case channel_failed_open:
    tft.print("Failed Open-Circuit");
    break;

  default:
    tft.print("Unknown / Invalid Channel Status");
    break;
  }
}

void power_activity()
{
  // Wake system
  if ((power_state != awake) && (power_state != sleep_to_awake_transition))
  {
    begin_wake();
  }

  // Reset sleep timer
  sleep_timer = 0;
}

void power_state_update()
{
  // Awake & idle
  if ((power_state == awake) && (sleep_timer >= sleep_time))
  {
    begin_sleep();
  }

  // Falling asleep
  if (power_state == awake_to_sleep_transition)
  {
    // Dim backlight
    if (sleep_timer >= 1)
    {
      sleep_timer -= 1;
      if (!backlight_down())
      {
        // We are at the minimum, so we are fully asleep
        power_state = sleep;
        sleep_timer = 0;
        UG_TextboxSetText(&sleep_window, TXB_ID_1, "Standby Mode\nChime Power Off");
      }
    }
  }

  // Waking up
  else if (power_state == sleep_to_awake_transition)
  {
    // Turn on power supply (it needs to stabilize)
    ps_enabled = true;

    // Raise backlight & wait for PS to stabilize
    if (sleep_timer >= 1)
    {
      sleep_timer -= 1;
      if (!backlight_up())
      {
        // We are maxed out, so we are fully awake
        wake_complete();
      }
    }
  }
}

bool backlight_up()
{
  // Are we at the limit?
  if (backlight_pwm == lcd_bl_pwm_max)
  {
    return false;
  }

  // Range-check new value
  uint32_t new_val(static_cast<uint32_t>(backlight_pwm) + lcd_bl_pwm_step);
  if (new_val >= static_cast<uint32_t>(lcd_bl_pwm_max))
  {
    backlight_pwm = lcd_bl_pwm_max;
  }
  else
  {
    backlight_pwm = new_val;
  }
  analogWrite(lcd_backlight_pin, backlight_pwm);
  return true;
}
bool backlight_down()
{
  // Are we at the limit?
  if (backlight_pwm == lcd_bl_pwm_min)
  {
    return false;
  }

  // Range-check new value
  uint16_t new_val(backlight_pwm - lcd_bl_pwm_step);
  if (lcd_bl_pwm_step >= backlight_pwm)
  {
    // Would have gone below 0 on an unsigned number!
    new_val = 0;
  }
  if (new_val <= lcd_bl_pwm_min)
  {
    backlight_pwm = lcd_bl_pwm_min;
  }
  else
  {
    backlight_pwm = new_val;
  }
  analogWrite(lcd_backlight_pin, backlight_pwm);
  return true;
}

void begin_sleep()
{
  // Transition to sleep
  power_state = awake_to_sleep_transition;
  sleep_timer = 0;

  // Disable power supply
  ps_enabled = false;

  // Show sleep window
  UG_TextboxSetText(&sleep_window, TXB_ID_1, "Sleeping...");
  UG_WindowShow(&sleep_window);
}
void begin_wake()
{
  // Transition to wake
  power_state = sleep_to_awake_transition;
  sleep_timer = 0;

  // Enable power supply
  ps_enabled = true;

  // Update sleep window
  UG_TextboxSetText(&sleep_window, TXB_ID_1, "Waking Up\nPower Supply\nStabilizing...");
}
void wake_complete()
{
  // We are now fully awake
  power_state = awake;
  sleep_timer = 0;

  // Return to main window
  UG_WindowHide(&sleep_window);
  UG_WindowShow(&main_window);
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
  UG_ButtonCreate(&settings_window, &settings_ps_on_button, BTN_ID_0, 10, 10, 100,  60);
  UG_ButtonSetFont(&settings_window, BTN_ID_0, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_0, "Power\nSupply\nON");
  UG_ButtonSetBackColor(&settings_window, BTN_ID_0, C_GREEN);
  UG_ButtonCreate(&settings_window, &settings_ps_off_button, BTN_ID_1, 10, 70, 100, 120);
  UG_ButtonSetFont(&settings_window, BTN_ID_1, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_1, "Power\nSupply\nOFF");
  UG_ButtonCreate(&settings_window, &settings_redraw_button, BTN_ID_2, 110, 10, 200, 60);
  UG_ButtonSetFont(&settings_window, BTN_ID_2, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_2, "Redraw");
  UG_ButtonCreate(&settings_window, &settings_close_button, BTN_ID_3, 110, 70, 200, 120);
  UG_ButtonSetFont(&settings_window, BTN_ID_3, &FONT_8X12);
  UG_ButtonSetText(&settings_window, BTN_ID_3, "Close");
  UG_CheckboxCreate(&settings_window, &settings_override_velocity_checkbox, CHB_ID_0, 10, 130, 200, 150);
  UG_CheckboxSetFont(&settings_window, CHB_ID_0, &FONT_8X12);
  UG_CheckboxSetText(&settings_window, CHB_ID_0, "Override Velocity");
  UG_CheckboxSetChecked(&settings_window, CHB_ID_0, static_cast<UG_U8>(override_velocity));
  UG_CheckboxCreate(&settings_window, &settings_play_all_programs_checkbox, CHB_ID_1, 10, 160, 200, 180);
  UG_CheckboxSetFont(&settings_window, CHB_ID_1, &FONT_8X12);
  UG_CheckboxSetText(&settings_window, CHB_ID_1, "Play All Programs");
  UG_CheckboxSetChecked(&settings_window, CHB_ID_1, static_cast<UG_U8>(play_all_programs));
}

void settings_callback(UG_MESSAGE* msg)
{
  if ((msg->type == MSG_TYPE_OBJECT) &&
      (msg->id == OBJ_TYPE_BUTTON) &&
      (msg->event == BTN_EVENT_CLICKED))
  {
    switch (msg->sub_id)
    {
    case BTN_ID_0: // settings_ps_on_button
      UG_ButtonSetBackColor(&settings_window, BTN_ID_0, selected_color);
      UG_ButtonSetBackColor(&settings_window, BTN_ID_1, UG_WindowGetBackColor(&settings_window));
      break;

    case BTN_ID_1: // settings_ps_off_button
      UG_ButtonSetBackColor(&settings_window, BTN_ID_0, UG_WindowGetBackColor(&settings_window));
      UG_ButtonSetBackColor(&settings_window, BTN_ID_1, selected_color);
      break;

    case BTN_ID_2: // settings_redraw_button
      UG_WindowHide(&settings_window);
      UG_WindowShow(&settings_window);
      break;

    case BTN_ID_3: // settings_close_button
      UG_WindowHide(&settings_window);
      break;

    default:
      break;
    }
  }
  else if ((msg->type == MSG_TYPE_OBJECT) &&
           (msg->id == OBJ_TYPE_CHECKBOX) &&
           (msg->event == CHB_EVENT_CLICKED))
  {
    switch (msg->sub_id)
    {
    case CHB_ID_0: // settings_override_velocity_checkbox
      override_velocity = (UG_CheckboxGetChecked(&settings_window, CHB_ID_0) != 0);
      break;

    case CHB_ID_1: // settings_play_all_programs_checkbox
      play_all_programs = (UG_CheckboxGetChecked(&settings_window, CHB_ID_1) != 0);
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
    sprintf(file_list_buffer[i], "file%d.mid", txb_id);
    UG_TextboxSetText(&fb_window, txb_id, file_list_buffer[i]);
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
  // TODO: Scroll through directory entries
  if (fb_selected_line <= 0) { fb_selected_line = 0; }
  if (fb_selected_line >= FB_LIST_SIZE-1) { fb_selected_line = FB_LIST_SIZE-1; }

  // Redraw highlight line in new location
  UG_TextboxSetBackColor(&fb_window, fb_selected_line, selected_color);

  // Debug output
  sprintf(selected_file_buffer, "sel: %d", fb_selected_line); 
  UG_TextboxSetText(&fb_window, (FB_LIST_SIZE+1), selected_file_buffer);
}

void draw_sleep_window()
{
  // Window layout
  UG_WindowCreate(&sleep_window, obj_buff_sleep_window, sizeof(obj_buff_sleep_window) / sizeof(*obj_buff_sleep_window), sleep_callback);
  UG_WindowResize(&sleep_window, 49, 49, 319-50, 239-50);
  UG_WindowSetTitleTextAlignment(&sleep_window, ALIGN_CENTER);
  UG_WindowSetTitleText(&sleep_window, "Power Saving");
  UG_WindowSetBackColor(&sleep_window, C_FOREST_GREEN);
  UG_WindowSetForeColor(&sleep_window, C_WHITE);

  // UI layout variables
  const uint16_t width(UG_WindowGetInnerWidth(&sleep_window));
  const uint16_t height(UG_WindowGetInnerHeight(&sleep_window));

  // Sleep icon
  UG_TextboxCreate(&sleep_window, &sleep_icon, TXB_ID_0, 0, 0, width, 50);
  UG_TextboxSetFont(&sleep_window, TXB_ID_0, &font_FontAwesome_mod_50X40);
  UG_TextboxSetAlignment(&sleep_window, TXB_ID_0, ALIGN_CENTER);
  UG_TextboxSetText(&sleep_window, TXB_ID_0, fa_icon_sleep_leaf);

  // Sleep text
  UG_TextboxCreate(&sleep_window, &sleep_text, TXB_ID_1, 0, 50, width, height);
  UG_TextboxSetFont(&sleep_window, TXB_ID_1, &FONT_12X20);
  UG_TextboxSetAlignment(&sleep_window, TXB_ID_1, ALIGN_CENTER);
}

void sleep_callback(UG_MESSAGE* msg)
{}
