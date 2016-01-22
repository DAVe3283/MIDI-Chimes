// -----------------------------------------------------------------------------
// Master.ino
//
// The master Teensy that runs the MIDI chimes.
// This teensy is responsible for running the display, USB and physical MIDI,
// address auto-assignment, etc.
//
// TODO:
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
// * Needs to talk to the display
//   * Show errors
//   * Show input source (USB or physical MIDI)
//   * Master volume
//   * Options
//     * Override velocity to master volume vs. scale velocity 0-master
//       * Default should probably be scale
//     * Physical MIDI port is OUT or THRU
//     * Pass USB MIDI to physical MIDI OUT? (or should I always just do this?)
// * Read from SD card
//   * Basic file browser
// * Playback of MIDI files (from SD card)
// * Save options to EEPROM (or FLASH or whatever the Teensy has)
//   * Or save them to the SD card?
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include "i2c_t3.h"


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------



// I2C config
const uint8_t slave_addresses[] =
{
  32,32,32,
  // 'A', // Slave 0
  // 'B', // Slave 1
  // 'C', // Slave 2
};
const size_t num_slaves(sizeof(slave_addresses)/sizeof(*slave_addresses));

// Note mapping
const size_t notes_per_slave(10);
// WARNING: make sure this table is big enough!
const uint8_t note_map[num_slaves][notes_per_slave] =
{
  // Slave 0
  // G3  G3# A3  A3# B3  C4  C4# D4  D4# E4
  {  55, 56, 57, 58, 59, 60, 61, 62, 63, 64, },

  // Slave 1
  // F4  F4# G4  G4# A4  A4# B4  C5  C5# D5
  {  65, 66, 67, 68, 69, 70, 71, 72, 73, 74, },

  // Slave 2
  // D5# E5  F5  F5# G5  <--- Not Used --->
  {  75, 76, 77, 78, 79,  0,  0,  0,  0,  0, },
};
// TODO: convert this from a reverse-lookup table to a lookup table.
// It won't be as easy to read, but it will be faster. Or do we need the speed?

// Pins
const uint8_t pwm_pin(3);
const uint8_t led_pin(13);

// Timeouts
const uint16_t blink_time(20000); // microseconds


// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity);
void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnVelocityChange(uint8_t channel, uint8_t note, uint8_t velocity);
// void OnControlChange(uint8_t channel, uint8_t control, uint8_t value);
// void OnProgramChange(uint8_t channel, uint8_t program);
// void OnAfterTouch(uint8_t channel, uint8_t pressure);
// void OnPitchChange(uint8_t channel, int pitch);
// TODO: handle more MIDI stuff?

// Gets the slave address and channel on the slave for a given MIDI note
bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out);


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


// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  // Configure I2C
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Configure serial
  ser.begin(1500000);

  // Configure USB MIDI
  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleNoteOff(OnNoteOff);
}

// Main program loop
void loop()
{
  // Handle USB MIDI messages
  usbMIDI.read();
}

void OnNoteOn(uint8_t channel, uint8_t note, uint8_t velocity)
{
  digitalWrite(led_pin, HIGH); // temp diagnostics

  // Are we handling this channel?
  if ((our_channel == 0) || (our_channel == channel))
  {
    // Lookup note
    uint8_t slave_address;
    uint8_t slave_channel;
    if (get_slave_and_channel(note, slave_address, slave_channel))
    {
      // Send I2C message
      Wire.beginTransmission(slave_addresses[slave_address]);
      Wire.write(slave_address);
      Wire.write(slave_channel);
      Wire.write(velocity);
      Wire.endTransmission();
      // TODO: come up with a protocol
    }
  }

  digitalWrite(led_pin, LOW); // temp diagnostics
}

void OnNoteOff(uint8_t channel, uint8_t note, uint8_t velocity)
{
  digitalWrite(led_pin, LOW);
  // ser.print("Note Off! Channel ");
  // ser.print(channel);
  // ser.print(" Note: ");
  // ser.print(note);
  // ser.print(" Velocity: ");
  // ser.print(velocity);
  // ser.println();

  // Ask for I2C data
  Wire.requestFrom(slave_addresses[0], 2);
  uint8_t data;
  while(Wire.available()) {
    data = Wire.receive();
    ser.print("d=0x");
    ser.println(data, HEX);
  }
}

bool get_slave_and_channel(const uint8_t& midi_note, uint8_t& slave_address_out, uint8_t& slave_channel_out)
{
  for (uint8_t slave(0); slave < num_slaves; ++slave)
  {
    for (uint8_t channel(0); channel < notes_per_slave; ++channel)
    {
      if (note_map[slave][channel] == midi_note)
      {
        slave_address_out = slave;
        slave_channel_out = channel;
        return true;
      }
    }
  }
  return false;
}
