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
  'A', // Slave 0
  'B', // Slave 1
  'C', // Slave 2
};
const size_t num_slaves(sizeof(slave_addresses)/sizeof(*slave_addresses));
// TODO: auto-assignment of slave addresses

// Note mapping
const size_t notes_per_slave(9);
// WARNING: make sure this table is big enough!
const uint8_t note_map[num_slaves][notes_per_slave] =
{
  // Slave 0
  // G3  G3# A3  A3# B3  C4  C4# D4  D4#
  {  55, 56, 57, 58, 59, 60, 61, 62, 63 },

  // Slave 1
  // E4  F4  F4# G4  G4# A4  A4# B4  C5
  {  64, 65, 66, 67, 68, 69, 70, 71, 72 },

  // Slave 2
  // C5# D5  D5# E5  F5  F5# G5  < N/A >
  {  73, 74, 75, 76, 77, 78, 79,  0,  0 },
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
const uint8_t led_pin(13);

// Timeouts
const uint16_t blink_time(20000); // microseconds


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
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Configure serial
  ser.begin(1500000);

  // Configure USB MIDI
  usbMIDI.setHandleNoteOn(OnNoteOn);
  // usbMIDI.setHandleNoteOff(OnNoteOff); // Not needed
  // TODO: handle more MIDI stuff?
}

// Main program loop
void loop()
{
  // Handle USB MIDI messages
  usbMIDI.read();
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
