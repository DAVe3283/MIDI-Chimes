// -----------------------------------------------------------------------------
// Master.ino
//
// The master Teensy that runs the MIDI chimes.
// This Teensy is responsible for running the display, USB and physical MIDI,
// address auto-assignment, etc.
//
// TODO:
// * Limit max simultaneous chime strikes to avoid overloading PSU
// * See Notes/MIDI.md for channel, program, and OUT/THRU information.
// * Implement settings
//   * Storing them in EEPROM is fine
// * Implement channel selection
// * Implement MIDI OUT and THRU
// * Handle per-channel volume?
// * Make the display do useful things
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
// * Stop other playback while doorbell is playing?
// -----------------------------------------------------------------------------

#define VERSION_MAJOR 0
#define VERSION_MINOR 5
#define VERSION_REVISION 1

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

// Slave status message parsing
#include "slave_status.h"

using namespace midi_chimes;

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I²C config
const uint8_t i2c_slave_base_address(0x10); // Starting address for slaves
const uint8_t i2c_slave_ack(0x06); // ACK sentinel expected from the slaves

// Settings file
const char ini_filename[] = "/Settings.ini";

// Note map (loaded from config file)
// MIDI spec only allows a maximum of 128 notes, so we will just build a map of
// all possibilities
slave_note_map note_map[128] = {};
const size_t notes_per_slave(10);

// Power Supply
const float minimum_ps_voltage(10.0f); // Minimum voltage to consider PS working
const size_t max_notes_active(10); // Maximum simultaneous notes to not overload PS

// Duty Cycle Settings
const uint8_t pwm_bits(12); // 0 - 4095 <-- must match slaves!
const uint16_t maximum_dc((1 << pwm_bits) - 1);

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
const uint32_t ps_settle_time(500); // milliseconds
const uint32_t note_strike_time(90); // milliseconds <-- must match slaves!
const uint32_t slave_post_timeout(3000); // milliseconds (after ps_settle_time)
const uint32_t slave_status_interval(1000); // milliseconds
const uint32_t sleep_time(5 * 60 * 1000); // milliseconds

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
const char fa_icon_settings[] = "u";      // Sliders
const char fa_icon_vol_dn[] = "F";        // Volume Down
const char fa_icon_vol_up[] = "G";        // Volume Up
const char fa_icon_level_up[] = "]";      // Level Up
const char fa_icon_level_down[] = "^";    // Level Down
const char fa_icon_folder_closed[] = "_"; // Folder, Closed (solid)
const char fa_icon_folder_open[] = "a";   // Folder, Open (solid)
const char fa_icon_file_generic[] = "d";  // File, Generic (outline)
const char fa_icon_file_picture[] = "k";  // File, Picture / Image (outline)
const char fa_icon_file_sound[] = "m";    // File, Sound / Music (outline)
const char fa_icon_file_config[] = "o";   // File, Code / Config (outline)
const char fa_icon_sleep_leaf[] = "C";    // Leaf (sleep)
const char fa_icon_bell[] = "Y";          // Bell
const char fa_icon_music[] = "D";         // Music Note (Beamed)
const char fa_icon_usb[] = "y";           // USB Logo
const char fa_icon_info[] = "S";          // Information (in circle)
const char fa_icon_exclaim[] = "!";       // Exclamation (in circle)
const char fa_icon_question[] = "?";      // Question (in circle)

// MIDI Commands
const uint8_t midi_cmd_note_off(0x80);
const uint8_t midi_cmd_note_on(0x90);
const uint8_t midi_cmd_aftertouch(0xA0);
const uint8_t midi_cmd_control_change(0xB0);
const uint8_t midi_cmd_program_change(0xC0);
const uint8_t midi_cmd_aftertouch_mono(0xD0);
const uint8_t midi_cmd_pitch_bend(0xE0);
const uint8_t midi_cmd_system_exclusive(0xF0);
const uint8_t midi_cmd_end_exclusive(0xF7);

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

// Doorbell song (hard coded for now, TODO: play off SD card)
// E5 C5 D5 G4, G4 D5 E5 C5 works on these chimes, but one octave lower might too
const uint8_t  doorbell_song_notes[] = {  76,  72,  74,  67,   67,  74,  76,  72, }; // or 64,60,62,55,55,62,64,60
const uint32_t doorbell_note_delay[] = {   0, 500, 500, 500, 1000, 500, 500, 500, }; // milliseconds before starting this note
const size_t   doorbell_song_length(sizeof(doorbell_song_notes) / sizeof(*doorbell_song_notes));

// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
// void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity); // Not needed
void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnVelocityChange(uint8_t channel, uint8_t note, uint8_t velocity);
void OnControlChange(uint8_t channel, uint8_t control, uint8_t value);
void OnProgramChange(uint8_t channel, uint8_t program);
// void OnAfterTouch(uint8_t channel, uint8_t pressure);
// void OnPitchChange(uint8_t channel, int pitch);
void OnSystemExclusive(const uint8_t* data, uint16_t length, bool complete);
// void OnRealTimeSystem(uint8_t realtimebyte);
// void OnTimeCodeQuarterFrame(uint16_t data);

// How many notes are available without exceeding max_notes_active?
size_t notes_available();

// Scale a MIDI velocity (7-bit) to our 12-bit velocity
uint16_t scale_midi_velocity(const uint8_t& midi_velocity, const uint8_t& note);

// Send a chime strike command to a slave
void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity);

// Gets the slave address and channel on the slave for a given MIDI note
bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out);

// Adjust Master Volume
void adjust_master_volume(int8_t change); // Adjust up/down by the specified amount
void set_master_volume(const int8_t& volume); // Set volume the specified value (0 - 100)

// I²C functions
uint8_t i2c_addr_auto_assign(); // Auto-assign I²C addresses to slaves
bool i2c_check_ack(const uint8_t& address); // Check for ack from slave
void i2c_check_result(const uint8_t& i2c_result, const uint8_t& address, const int& line);

// Doorbell Functions
void doorbell_isr(); // Doorbell physically pressed
void doorbell_update(); // Update doorbell software state, start chiming, etc.

// Get slave status
bool get_slave_status(const uint8_t& address, slave_status& state);
void validate_slave_config(const uint8_t& slave, const slave_status& status);
void validate_slave_status(const uint8_t& slave, const slave_status& status);
void next_slave(); // Update current_slave to the next slave

// Power management functions
void ps_update(); // Toggle PS_EN when required to activate power supply
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

// About window
void draw_about_window();
void about_window_callback(UG_MESSAGE* msg);

// Sleep window
void draw_sleep_window();
void sleep_callback(UG_MESSAGE* msg);

// Doorbell window
void draw_doorbell_window();
void doorbell_callback(UG_MESSAGE* msg);


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// User settings
uint8_t our_channel(0); // Current channel (0 = all, 1-16)
int8_t master_volume(100); // Master volume (0-100, 0 means mute, steps of 10)
bool override_velocity(false); // Override velocity to master volume (true), or scale it by master volume (false)
bool play_all_programs(false); // Play all programs (instruments), or just Tubular Bells?

// Slave status
uint8_t slaves_found(0); // How many slaves we found during POST
uint8_t current_slave(0); // The current slave we are checking the status of
size_t notes_active(0); // How many notes are currently being struck

// Power supply
bool ps_enabled(true); // Is the power supply enabled?
bool ps_en_high(false); // Is the PS_EN pin high currently?

// Backlight
uint16_t backlight_pwm(lcd_bl_pwm_max);

// MIDI state
bool play_this_program[17]; // Do we play notes for the current program?
bool midi_master_volume_14bit(false); // Is the master volume message 14-bit? (or just 7-bit)
bool midi_receiving_sysex(false); // Are we currently receiving a system exclusive message?
uint8_t midi_sysex_buffer[8] = {}; // Buffer long enough to hold the longest message we will handle
const size_t midi_sysex_buffer_size(sizeof(midi_sysex_buffer) / sizeof(*midi_sysex_buffer));
size_t midi_sysex_message_length(0); // Length of the last received system exclusive message (including start & end marker)

// Doorbell
volatile bool doorbell_pressed(false); // Set to true in ISR when the doorbell is pressed
bool doorbell_playing(false); // True while we are playing the doorbell song
size_t doorbell_song_index(0); // Used to play through the hard coded song, will be removed when we switch to MIDI files

// Timers
elapsedMicros ps_en_timer;
elapsedMillis sleep_timer;
elapsedMillis doorbell_timer;
elapsedMillis slave_status_timer;
elapsedMillis note_strike_timer[max_notes_active];

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
  Adafruit_FT6206 ts = Adafruit_FT6206();  // Using default Arduino I²C pins?
#else
  Adafruit_STMPE610 ts = Adafruit_STMPE610(touch_cs_pin); // Touch sensor
#endif

// µGUI
UG_GUI gui;

// Main Window
UG_WINDOW main_window;
UG_BUTTON main_window_button_settings;
UG_BUTTON main_window_button_about;
UG_BUTTON main_window_button_vol_dn;
UG_BUTTON main_window_button_vol_up;
UG_PROGRESSBAR main_window_prb_volume;
UG_OBJECT main_window_buffer[5];
char volume_text_buffer[5] = { 0 };

// About window
UG_WINDOW about_window;
UG_TEXTBOX about_icon_midi;
UG_TEXTBOX about_icon_usb;
UG_TEXTBOX about_title;
UG_TEXTBOX about_text;
UG_OBJECT obj_buff_about_window[4];

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

// Doorbell window
UG_WINDOW doorbell_window;
UG_TEXTBOX doorbell_icon;
UG_TEXTBOX doorbell_text;
UG_OBJECT obj_buff_doorbell_window[2];


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

  // Configure doorbell
  pinMode(doorbell_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(doorbell_pin), doorbell_isr, RISING);

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

  // Configure I²C
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
  config_file settings = config_file(ini_filename);
  UG_ConsolePutString("Opening settings file...");
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
  if (!settings.load_globals())
  {
    draw_BSOD(tft);
    tft.print("INI file ");
    tft.print(ini_filename);
    tft.println(" is missing critical data.");
    tft.println("Ensure the INI file has all sections and required values.");
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
      tft.print(slave + 1);
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
      settings.get_note_calibration(
        note,
        note_map[note].calibration_min,
        note_map[note].calibration_max);
    }
  }
  UG_ConsolePutString("done.\n");

  // Configure slave addresses
  UG_ConsolePutString("Detecting slave boards...");
  slaves_found = i2c_addr_auto_assign();
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
    halt_system();
  }
  UG_ConsolePutString("done.\n");

  // POST slaves
  UG_ConsolePutString("Performing slave POST...");
  elapsedMillis post_timer;
  bool got_slave_status[slaves_found];
  for (uint8_t i(0); i < slaves_found; ++i)
  {
    got_slave_status[i] = false;
  }
  bool post_complete(false);
  while (!post_complete)
  {
    // Enable power supply
    ps_update();

    // Request slave status
    if (!got_slave_status[current_slave])
    {
      slave_status status;
      got_slave_status[current_slave] = get_slave_status(
        current_slave + i2c_slave_base_address,
        status);

      // Validate slave status
      if (got_slave_status[current_slave])
      {
        validate_slave_config(current_slave, status);
        validate_slave_status(current_slave, status);
      }
    }
    next_slave();

    // Determine if we have all the statuses
    post_complete = true;
    for (uint8_t i(0); i < slaves_expected; ++i)
    {
      post_complete &= got_slave_status[i];
    }

    // Handle timeout
    if (post_timer > (ps_settle_time + slave_post_timeout))
    {
      draw_BSOD(tft);
      tft.println("Timed out waiting for slaves to POST!");
      for (uint8_t slave(0); slave < slaves_found; ++slave)
      {
        if (!got_slave_status[slave])
        {
          tft.print("Slave ");
          tft.print(slave + 1);
          tft.println(" did not return a status.");
        }
      }
      tft.println();
      tft.println("Please verify all cables are connected securely, &");
      tft.println("the correct firmware is loaded on the slaves.");
      halt_system();
    }
  }
  UG_ConsolePutString("done.\n");

  // Initialize MIDI variables
  for (int channel(0); channel <= 16; ++channel)
  {
    // Assume everything is tubular bells until told otherwise
    play_this_program[channel] = true;
  }

  // Configure USB MIDI
  // usbMIDI.setHandleNoteOff(OnNoteOff); // Not needed (might for OUT/THRU?)
  usbMIDI.setHandleNoteOn(OnNoteOn);
  // usbMIDI.setHandleVelocityChange(OnVelocityChange); // Not needed (might for OUT/THRU?)
  usbMIDI.setHandleControlChange(OnControlChange);
  usbMIDI.setHandleProgramChange(OnProgramChange);
  // usbMIDI.setHandleAfterTouch(OnAfterTouch); // Not needed (might for OUT/THRU?)
  // usbMIDI.setHandlePitchChange(OnPitchChange); // Not needed (might for OUT/THRU?)
  usbMIDI.setHandleSysEx(OnSystemExclusive);
  // usbMIDI.setHandleRealTimeSystem(OnRealTimeSystem); // Not needed (might for OUT/THRU?)
  // usbMIDI.setHandleTimeCodeQuarterFrame(OnTimeCodeQuarterFrame); // Not needed (might for OUT/THRU?)

  // Configure hardware MIDI
  midi.begin(31250, SERIAL_8N1);

  // Configure µGUI
  UG_ConsolePutString("Initializing \xE6GUI...");
  // Draw windows
  draw_main_window();
  draw_about_window();
  draw_settings_window();
  draw_sleep_window();
  draw_doorbell_window();
  // Start GUI at main window
  UG_WindowShow(&main_window);
  UG_ConsolePutString("done.\n");

  // Set initial power state
  power_state = awake;
  sleep_timer = 0;
}

// Main program loop
void loop()
{
  // Manage power state (sleep)
  power_state_update();

  // Handle doorbell
  doorbell_update();

  // Handle power supply
  ps_update();

  // Check slave status (only while awake)
  if ((power_state == awake) && (slave_status_timer >= slave_status_interval))
  {
    // Reset the timer
    slave_status_timer = 0;

    // Determine what slave to work with
    const uint8_t address(i2c_slave_base_address + current_slave);
    slave_status status;
    if (!get_slave_status(address, status))
    {
      draw_BSOD(tft);
      tft.println("Lost communication with slave board!");
      tft.print(  "Slave: ");
      tft.println(current_slave + 1);
      tft.println();
      tft.println("Please verify all cables are connected securely.");
      halt_system();
    }
    validate_slave_status(current_slave, status);
    next_slave();
  }

  // Handle USB MIDI messages
  usbMIDI.read();

  // Handle hardware MIDI messages
  int midi_bytes_available;
  while ((midi_bytes_available = midi.available()) > 0)
  {
    power_activity();

    const uint8_t status(midi.peek());

    // Handle MIDI system exclusive data
    if (midi_receiving_sysex)
    {
      midi.read(); // Drop the byte from the buffer

      // Look for end of message
      if (status == midi_cmd_end_exclusive)
      {
        midi_receiving_sysex = false;
        const bool have_full_message(midi_sysex_message_length <= midi_sysex_buffer_size);
        OnSystemExclusive(
          midi_sysex_buffer,
          have_full_message ? midi_sysex_message_length : midi_sysex_buffer_size,
          have_full_message);
        continue;
      }

      // Store the message until the buffer is full
      if (midi_sysex_message_length < midi_sysex_buffer_size)
      {
        midi_sysex_buffer[midi_sysex_message_length] = status;
      }
      midi_sysex_message_length++;

      // Move on to the next byte
      continue;
    }

    // Drop non-command data until the buffer starts with a command
    if (status < 0x80) // All command bytes start with a 1
    {
      midi.read(); // Drop the byte from the buffer
      continue;
    }

    // Parse
    const uint8_t command( status & 0xF0);      // Mask off command
    const uint8_t channel((status & 0x0F) + 1); // Mask off channel, add 1 to match usbMIDI

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

    // System Exclusive
    case midi_cmd_system_exclusive:
      required_data_bytes = 0; // We start dumping it to a buffer
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

      // Control Change (channel volume)
      case midi_cmd_control_change:
        OnControlChange(channel, midi_buffer[0], midi_buffer[1]);
        break;

      // Program change
      case midi_cmd_program_change:
        OnProgramChange(channel, midi_buffer[0]);
        break;

      // System Exclusive
      case midi_cmd_system_exclusive:
        midi_receiving_sysex = true;
        midi_sysex_message_length = 0;
        break;

      // Not implemented
      case midi_cmd_note_off:
      case midi_cmd_aftertouch:
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

    // Handle touch event
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

    // Handle touch event
    UG_TouchUpdate(x, y, touched ? TOUCH_STATE_PRESSED : TOUCH_STATE_RELEASED);
  }
  else
  {
    if (!touched)
    {
      // Indicate touch released
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

  // Verify the channel is valid
  if (channel > 16)
  {
    return;
  }

  // Are we handling this channel and/or program (instrument)?
  const bool channel_valid((our_channel == 0) || (channel == our_channel));
  const bool play_channel(play_all_programs || play_this_program[channel]);
  if (channel_valid && play_channel)
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

void OnControlChange(uint8_t channel, uint8_t control, uint8_t value)
{
  // TODO: MIDI out/thru

  // Skip other channels
  if (our_channel && (channel != our_channel))
  {
    return;
  }

  // Handle control change messages
  // See http://nickfever.com/music/midi-cc-list for more messages
  switch (control)
  {
  // Adjust system volume
  case   7: // Volume
  case  11: // Expression
    set_master_volume(static_cast<int>(value) * 100 / 127);
    break;

  // Reset
  case 121: // Reset All Controllers
    // TODO: what needs reset?
    break;

  // Not implemented
  default:
    break;
  }
}

void OnProgramChange(uint8_t channel, uint8_t program)
{
  power_activity();

  // Verify the channel is valid
  if ((channel < 1) || (channel > 16))
  {
    return;
  }

  play_this_program[channel] = (program == midi_prog_tubular_bells);
  // TODO: MIDI out/thru
}

void OnSystemExclusive(const uint8_t* data, uint16_t length, bool complete)
{
  if (!complete)
  {
    // Can't do anything with part of a message
    // We don't buffer larger messages; the buffer is sized for the largest message we expect
    return;
  }

  // Handle Master Volume message
  // http://www.recordingblogs.com/sa/tabid/88/Default.aspx?topic=MIDI+Master+Volume+message
  if ((length == 6) &&      // Expected length is 6 + start & end commands
      (data[0] == 0x7F) &&  // Realtime
      (data[1] == 0x7F) &&  // SysEx channel, 0x7F means all/disregard
      (data[2] == 0x04) &&  // Sub-ID - Device Control
      (data[3] == 0x01))    // Sub-ID - Master Volume
  {
    // const uint8_t system_exclusive_channel(data[1]);
    const uint16_t volume_lower_bits(data[4] & 0x7F);
    const uint16_t volume_upper_bits(data[5] & 0x7F);

    // Check for 14-bit resolution
    if (volume_lower_bits)
    {
      midi_master_volume_14bit = true;
    }

    // Set volume
    if (midi_master_volume_14bit)
    {
      // 14-bit
      const uint16_t volume_14bit((volume_upper_bits << 7) & volume_lower_bits);
      set_master_volume(volume_14bit * 100 / ((1 << 14) - 1));
    }
    else
    {
      // 7- bit
      set_master_volume(volume_upper_bits * 100 / ((1 << 7) - 1));
    }
  }
}

size_t notes_available()
{
  // Check for completed strikes
  for (size_t note(0); note < notes_active; )
  {
    if (note_strike_timer[note] >= note_strike_time)
    {
      // Skip the last note
      if (note < (notes_active - 1))
      {
        // Move the last timer to this spot
        note_strike_timer[note] = note_strike_timer[notes_active - 1];
      }
      notes_active--;
    }
    else
    {
      note++;
    }
  }

  // Return how many notes are available
  return max_notes_active - notes_active;
}

uint16_t scale_midi_velocity(const uint8_t& midi_velocity, const uint8_t& note)
{
  // Don't worry about MIDI velocity == 0, that must be taken care of before we
  // try and send a message at all.

  // TODO: it might be worth re-working all the math as int32, as it might be
  //       faster than floats. But it might not matter.

  // Handle calibration for this note
  const float& cal_min(note_map[note].calibration_min);
  const float& cal_max(note_map[note].calibration_max);
  const uint16_t minimum_dc(cal_min * static_cast<float>(maximum_dc));
  const float range((cal_max * static_cast<float>(maximum_dc)) - static_cast<float>(minimum_dc));

  // Calculate requested velocity
  float requested_volume(static_cast<float>(midi_velocity & 0x7F) / 127.0f);

  // Handle master volume
  if (override_velocity)
  {
    // If we are overriding velocity, use the current volume level
    requested_volume = static_cast<float>(master_volume) / 100.0f;
  }
  else
  {
    // Scale the volume by the master volume
    requested_volume *= static_cast<float>(master_volume) / 100.0f;
  }

  // Calculate final velocity DC value
  const uint16_t velocity(minimum_dc + (range * requested_volume));
  return velocity;
}

void send_chime(const uint8_t& address, const uint8_t& channel, const uint16_t& velocity)
{
  // Verify we aren't striking too many chimes at once
  if (notes_available())
  {
    // Start strike timer
    note_strike_timer[notes_active++] = 0;
  }
  else
  {
    // We are striking the max number of notes, ignore this request
    return;
  }

  // Packetize message
  uint8_t buffer[sizeof(uint8_t) + sizeof(uint16_t)];
  memcpy(buffer + 0, &channel, sizeof(uint8_t));
  memcpy(buffer + 1, &velocity, sizeof(uint16_t));

  // Send I²C message
  Wire1.beginTransmission(address);
  Wire1.write(buffer, sizeof(buffer));
  uint8_t i2c_result(Wire1.endTransmission(I2C_STOP, 500));
  i2c_check_result(i2c_result, address, __LINE__);
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
  set_master_volume(master_volume + change);
}

void set_master_volume(const int8_t& volume)
{
  // Get current button setup
  const UG_COLOR w_fg(UG_WindowGetForeColor(&main_window));
  const UG_COLOR d_fg(UG_ButtonGetForeColor(&main_window, BTN_ID_1));
  const UG_U8    d_st(UG_ButtonGetStyle(&main_window, BTN_ID_1));
  const UG_COLOR u_fg(UG_ButtonGetForeColor(&main_window, BTN_ID_2));
  const UG_U8    u_st(UG_ButtonGetStyle(&main_window, BTN_ID_2));

  // Limit range to 0-100, enable/disable buttons
  if (volume >= 100)
  {
    master_volume = 100;

    // Enable volume down button
    if (d_fg != w_fg         )        { UG_ButtonSetForeColor(&main_window, BTN_ID_1, w_fg); }
    if (d_st != BTN_STYLE_3D )        { UG_ButtonSetStyle(&main_window, BTN_ID_1, BTN_STYLE_3D); }
    // Disable volume up button
    if (u_fg != button_text_disabled) { UG_ButtonSetForeColor(&main_window, BTN_ID_2, button_text_disabled); }
    if (u_st != BTN_STYLE_2D)         { UG_ButtonSetStyle(&main_window, BTN_ID_2, BTN_STYLE_2D); }
  }
  else if (volume <= 0)
  {
    master_volume = 0;

    // Disable volume down button
    if (d_fg != button_text_disabled) { UG_ButtonSetForeColor(&main_window, BTN_ID_1, button_text_disabled); }
    if (d_st != BTN_STYLE_2D)         { UG_ButtonSetStyle(&main_window, BTN_ID_1, BTN_STYLE_2D); }
    // Enable volume up button
    if (u_fg != w_fg         )        { UG_ButtonSetForeColor(&main_window, BTN_ID_2, w_fg); }
    if (u_st != BTN_STYLE_3D )        { UG_ButtonSetStyle(&main_window, BTN_ID_2, BTN_STYLE_3D); }
  }
  else
  {
    master_volume = volume;

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
  Wire1.endTransmission(I2C_STOP, 300);

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
    Wire1.endTransmission(I2C_STOP, 300);

    // Ask previous slave to latch the current slave for us
    Wire1.beginTransmission(slave_address - 1);
    Wire1.write(0x01); // Command = Set ADDR_LATCH
    Wire1.write(HIGH);
    Wire1.endTransmission(I2C_STOP, 300);
    delayMicroseconds(slave_latch_delay);
    Wire1.beginTransmission(slave_address - 1);
    Wire1.write(0x01); // Command = Set ADDR_LATCH
    Wire1.write(LOW);
    Wire1.endTransmission(I2C_STOP, 300);

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
    uint8_t i2c_result(Wire1.endTransmission(I2C_STOP, 300));
    i2c_check_result(i2c_result, i2c_slave_base_address + i, __LINE__);
  }

  // Done!
  return slaves_found;
}

bool i2c_check_ack(const uint8_t& address)
{
  // Look for a single ACK byte
  if (Wire1.requestFrom(address, 1, I2C_STOP, 200) == 1) // We got 1 byte
  {
    // Is the byte an ACK?
    return (Wire1.read() == i2c_slave_ack);
  }

  return false;
}

void i2c_check_result(const uint8_t& i2c_result, const uint8_t& address, const int& line)
{
  if (i2c_result)
  {
    draw_BSOD(tft);
    tft.println("I\xFC""C transmission failed!");
    tft.print("Address: 0x");
    tft.print(address, HEX);
    tft.print(" (Slave ");
    tft.print(address - i2c_slave_base_address);
    tft.println(")");
    tft.print("Result: ");
    tft.println(i2c_result);
    tft.print("Line:   ");
    tft.println(line);
    halt_system();
  }
}

void doorbell_isr()
{
  doorbell_pressed = true;
}

void doorbell_update()
{
  // Look for doorbell pressed event
  if (doorbell_pressed)
  {
    // Wake up the chimes if necessary
    power_activity();

    // Once awake, trigger the doorbell
    if (power_state == awake)
    {
      doorbell_pressed = false;
      if (!doorbell_playing)
      {
        doorbell_playing = true;
        doorbell_timer = 0;
        doorbell_song_index = 0;
        UG_WindowShow(&doorbell_window);
      }
    }
  }

  // Handle doorbell playback
  if (doorbell_playing)
  {
    // TODO: switch to MIDI file playback when implemented

    // Play the doorbell song
    if (doorbell_song_index < doorbell_song_length)
    {
      // We are in the song
      if (doorbell_timer >= doorbell_note_delay[doorbell_song_index])
      {
        doorbell_timer -= doorbell_note_delay[doorbell_song_index];
        OnNoteOn(our_channel, doorbell_song_notes[doorbell_song_index++], 127);
      }
    }
    else
    {
      // We completed the song
      doorbell_playing = false;
      doorbell_song_index = 0;
      // Return to main window
      UG_WindowHide(&doorbell_window);
      UG_WindowShow(&main_window);
      return;
    }
  }
}

bool get_slave_status(const uint8_t& address, slave_status& state)
{
  // Holds the response
  const size_t expected_status_size(6 + (4 * notes_per_slave));
  uint8_t buffer[expected_status_size];

  // Request the status
  const size_t status_size(Wire1.requestFrom(address, expected_status_size, I2C_STOP, 5000));
  // Returned data length is always <= requested length, so no worry of a buffer overflow
  for (size_t i(0); i < status_size; ++i)
  {
    buffer[i] = Wire1.read();
  }

  // Parse the status
  state.load_slave_status(buffer, status_size);
  return state.valid();
}

void validate_slave_config(const uint8_t& slave, const slave_status& status)
{
  // Verify the slave is using the number of PWM bits we expect
  const uint8_t slave_pwm_bits(status.pwm_bits());
  if (slave_pwm_bits != pwm_bits)
  {
    draw_BSOD(tft);
    tft.println("Slave configuration / firmware error!");
    tft.print("Slave ");
    tft.print(slave + 1);
    tft.print(" is configured to use ");
    tft.print(slave_pwm_bits);
    tft.println("-bit PWM, but the");
    tft.print("system is configured to use ");
    tft.print(pwm_bits);
    tft.println("-bit PWM!");
    tft.println();
    tft.println("Please verify the correct firmware is loaded on the");
    tft.println("slaves and the master boards.");
    halt_system();
  }

  // Verify the slave has the number of channels we expect
  const uint8_t slave_channels(status.num_channels());
  if (slave_channels != notes_per_slave)
  {
    draw_BSOD(tft);
    tft.println("Configuration / firmware error!");
    tft.print("Slave ");
    tft.print(slave + 1);
    tft.print(" has ");
    tft.print(slave_channels);
    tft.println("channel(s), but the system is setup");
    tft.print("for ");
    tft.print(notes_per_slave);
    tft.println(" channel slaves!");
    tft.println();
    tft.println("Please verify the correct slave(s) are attached,");
    tft.println("and the master has the correct firmware loaded.");
    halt_system();
  }

  // Verify the power supply is working
  const float ps_voltage(status.ps_voltage());
  if (ps_voltage < minimum_ps_voltage)
  {
    draw_BSOD(tft);
    tft.println("Power supply / slave error!");
    tft.println();
    tft.print("Slave: ");
    tft.println(slave + 1);
    tft.print("Measured Voltage: ");
    tft.print(ps_voltage);
    tft.println(" V");
    tft.print("Minimum Voltage: ");
    tft.print(minimum_ps_voltage);
    tft.println(" V");
    tft.println();
    tft.println("Please verify all cables are connected securely, &");
    tft.println("the slave has at least one chime connected.");
    tft.println("All slaves must have the ground wire connected.");
    halt_system();
  }
}

void validate_slave_status(const uint8_t& slave, const slave_status& status)
{
  // Check all mapped notes to ensure they are working
  for (uint8_t note(0); note < (sizeof(note_map) / sizeof(note_map[0])); ++note)
  {
    // Only check notes that are connected
    if (note_map[note].slave_address)
    {
      // Get slave & channel
      const uint8_t note_slave(note_map[note].slave_address - i2c_slave_base_address);
      const uint8_t channel(note_map[note].channel);

      // Only check notes on this slave
      if (slave != note_slave)
      {
        // Skip notes that are on other slaves
        continue;
      }

      // Get channel state
      const channel_state_t state(status.channel_state(channel));
      if (state != channel_working)
      {
        draw_BSOD(tft);
        tft.println("Channel malfunction found!");
        tft.print("Note:    ");
        tft.print(note);
        tft.print(" (");
        tft.print(config_file::lookup_note_name(note));
        tft.println(")");
        tft.print("Slave:   ");
        tft.println(note_slave + 1);
        tft.print("Channel: ");
        tft.println(channel + 1);
        tft.print("Status:  ");
        tft.print(static_cast<uint8_t>(state), DEC);
        tft.print(" (");
        slave_status::print_channel_state(tft, state);
        tft.println(")");
        tft.println();
        tft.println("Check the solenoid cable is attached correctly.");
        tft.println("Check the solenoid and wiring for damage.");
        tft.println("Check the INI file has the correct settings.");
        tft.println("Remove all power before retrying.");
        halt_system();
      }
    }
  }

  // Verify no channels are shorted (even if they aren't mapped)
  for (uint8_t channel(0); channel < notes_per_slave; ++channel)
  {
    const channel_state_t state(status.channel_state(channel));
    if (state == channel_failed_short)
    {
      draw_BSOD(tft);
      tft.println("Shorted channel found!");
      tft.print("Slave:   ");
      tft.println(slave + 1);
      tft.print("Channel: ");
      tft.println(channel + 1);
      tft.print("Status:  ");
      tft.print(static_cast<uint8_t>(state), DEC);
      tft.print(" (");
      slave_status::print_channel_state(tft, state);
      tft.println(")");
      tft.println();
      tft.println("Even if this channel isn't mapped, it must not be");
      tft.println("shorted, as that can damage the power supply,");
      tft.println("slave board, and/or solenoid.");
      tft.println();
      tft.println("Check the solenoid and wiring for damage.");
      tft.println("Remove all power before retrying.");
      halt_system();
    }
  }
}

void next_slave()
{
  current_slave++;
  if (current_slave >= slaves_found)
  {
    current_slave = 0;
  }
}

void ps_update()
{
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

  // About button
  UG_ButtonCreate(&main_window, &main_window_button_about, BTN_ID_3,
    width - padding - button_size,
    padding,
    width - padding,
    padding + button_size);
  UG_ButtonSetFont(&main_window, BTN_ID_3, &font_FontAwesome_mod_50X40);
  UG_ButtonSetText(&main_window, BTN_ID_3, fa_icon_info);

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
      UG_WindowShow(&about_window);
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
  UG_WindowResize(&settings_window, 19, 19, 319-20, 239-20);
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

void draw_about_window()
{
  // Window layout
  UG_WindowCreate(&about_window, obj_buff_about_window, sizeof(obj_buff_about_window) / sizeof(*obj_buff_about_window), about_window_callback);
  UG_WindowResize(&about_window, 24, 39, 319-25, 239-40);
  UG_WindowSetTitleTextAlignment(&about_window, ALIGN_CENTER);
  UG_WindowSetTitleText(&about_window, "About MIDI Chimes");
  // UG_WindowSetBackColor(&about_window, C_FOREST_GREEN);
  // UG_WindowSetForeColor(&about_window, C_WHITE);

  // UI layout variables
  const uint16_t width(UG_WindowGetInnerWidth(&about_window));
  const uint16_t height(UG_WindowGetInnerHeight(&about_window));
  const uint16_t padding(5);
  const uint16_t icon_size(50);

  // MIDI icon
  UG_TextboxCreate(&about_window, &about_icon_midi, TXB_ID_0,
    padding,                // top-left x
    padding,                // top-left y
    padding + icon_size,  // bottom-right x
    padding + icon_size); // bottom-right y
  UG_TextboxSetFont(&about_window, TXB_ID_0, &font_FontAwesome_mod_50X40);
  UG_TextboxSetAlignment(&about_window, TXB_ID_0, ALIGN_CENTER);
  UG_TextboxSetText(&about_window, TXB_ID_0, fa_icon_music);

  // USB icon
  UG_TextboxCreate(&about_window, &about_icon_usb, TXB_ID_1,
    width - padding - icon_size,
    padding,
    width - padding,
    padding + icon_size);
  UG_TextboxSetFont(&about_window, TXB_ID_1, &font_FontAwesome_mod_50X40);
  UG_TextboxSetAlignment(&about_window, TXB_ID_1, ALIGN_CENTER);
  UG_TextboxSetText(&about_window, TXB_ID_1, fa_icon_usb);

  // Title text
  UG_TextboxCreate(&about_window, &about_title, TXB_ID_2,
    padding + icon_size + padding,
    padding,
    width - padding - icon_size - padding,
    padding + icon_size);
  UG_TextboxSetFont(&about_window, TXB_ID_2, &FONT_12X20);
  UG_TextboxSetAlignment(&about_window, TXB_ID_2, ALIGN_CENTER);
  UG_TextboxSetText(&about_window, TXB_ID_2, "MIDI\nChimes");

  // About text
  UG_TextboxCreate(&about_window, &about_text, TXB_ID_3,
    padding,
    padding + icon_size + padding,
    width - padding,
    height - padding);
  // UG_TextboxSetFont(&about_window, TXB_ID_3, &FONT_12X20);
  UG_TextboxSetAlignment(&about_window, TXB_ID_3, ALIGN_TOP_LEFT);
  char* about_string = new char[512]();
  sprintf(about_string,
    "MIDI Chimes Master Board\n\nVersion %d.%d.%d\nCompiled %s %s\n\ngithub.com/DAVe3283/MIDI-Chimes",
    VERSION_MAJOR, VERSION_MINOR, VERSION_REVISION,
    __DATE__, __TIME__);
  UG_TextboxSetText(&about_window, TXB_ID_3, about_string);
}

void about_window_callback(UG_MESSAGE* msg)
{
  if (msg->event == OBJ_EVENT_CLICKED)
  {
    // Close the about window
    UG_WindowHide(&about_window);
  }
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

void draw_doorbell_window()
{
  // Window layout
  UG_WindowCreate(&doorbell_window, obj_buff_doorbell_window, sizeof(obj_buff_doorbell_window) / sizeof(*obj_buff_doorbell_window), doorbell_callback);
  UG_WindowResize(&doorbell_window, 49, 49, 319-50, 239-50);
  UG_WindowSetTitleTextAlignment(&doorbell_window, ALIGN_CENTER);
  UG_WindowSetTitleText(&doorbell_window, "Doorbell");
  // UG_WindowSetBackColor(&doorbell_window, C_FOREST_GREEN);
  // UG_WindowSetForeColor(&doorbell_window, C_WHITE);

  // UI layout variables
  const uint16_t width(UG_WindowGetInnerWidth(&doorbell_window));
  const uint16_t height(UG_WindowGetInnerHeight(&doorbell_window));

  // Doorbell icon
  UG_TextboxCreate(&doorbell_window, &doorbell_icon, TXB_ID_0, 0, 0, width, 50);
  UG_TextboxSetFont(&doorbell_window, TXB_ID_0, &font_FontAwesome_mod_50X40);
  UG_TextboxSetAlignment(&doorbell_window, TXB_ID_0, ALIGN_CENTER);
  UG_TextboxSetText(&doorbell_window, TXB_ID_0, fa_icon_bell);

  // Doorbell text
  UG_TextboxCreate(&doorbell_window, &doorbell_text, TXB_ID_1, 0, 50, width, height);
  UG_TextboxSetFont(&doorbell_window, TXB_ID_1, &FONT_12X20);
  UG_TextboxSetAlignment(&doorbell_window, TXB_ID_1, ALIGN_CENTER);
  UG_TextboxSetText(&doorbell_window, TXB_ID_1, "Doorbell Pressed!\nSomeone's at\nthe door!");
}

void doorbell_callback(UG_MESSAGE* msg)
{
  // TODO: tapping the doorbell window should cancel doorbell playback
}
