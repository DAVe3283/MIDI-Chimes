// -----------------------------------------------------------------------------
// Slave.ino
//
// Program for all the slave Teensy units that run the MIDI chimes coils.
// Each slave is responsible for running a number of chimes, and keeping them
// from overheating.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include "i2c_t3.h"


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I2C config
const uint8_t i2c_address('A');

// PWM pins used by the chime coils
const uint8_t chime_pins[] =
{
   3,
   4,
   5,
   6,
   9,
  10,
  20,
  21,
  22,
  23,
};

const uint8_t num_channels(sizeof(chime_pins) / sizeof(*chime_pins));
//const uint8_t pwm_pin(3); // DELETE ME

const uint8_t led_pin(13);

// Timeouts
uint16_t strike_time(90); // milliseconds
uint16_t settle_time(160); // milliseconds
const uint16_t blink_time(20000); // microseconds

// PWM Config
const uint8_t pwm_bits(12);
const uint32_t pwm_freq(8789);


// -----------------------------------------------------------------------------
// Thermal Management
// -----------------------------------------------------------------------------

// Maximum number of strikes (per chime) in a rolling window
const uint8_t max_strikes(60);

// Cooldown period (in ms).
// Every time this expires, an additional strike is allowed, up to max_strikes.
const uint16_t cooldown_period(1000);


// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
void i2c_receive(size_t numBytes);
void i2c_requested();

// Strike a chime (handles thermals, settle time, etc.)
void strike_chime(const uint8_t& channel, const uint16_t& duty_cycle);

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// Timers
elapsedMillis cooldown_timer;
elapsedMillis strike_timer[num_channels];
elapsedMicros message_blink_timer;

// Serial IO
usb_serial_class  usb = usb_serial_class();

// Are we striking currently?
bool striking[num_channels];

// Thermal load limiter
int8_t strikes_remaining[num_channels];

// Strike chime variables
bool strike(false);


// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  // Diagnostics
  usb.begin(9600);

  // Configure I2C
  Wire.begin(I2C_SLAVE, i2c_address, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_requested);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Configure PWM channels
  analogWriteResolution(pwm_bits);
  for (int channel(0); channel < num_channels; ++channel)
  {
    pinMode(chime_pins[channel], OUTPUT);
    analogWriteFrequency(chime_pins[channel], pwm_freq);
    analogWrite(chime_pins[channel], 0); // Off
    striking[channel] = false;
    strikes_remaining[channel] = max_strikes;
  }
}

// Main program loop
void loop()
{
  // Handle cooldown
  if (cooldown_timer >= cooldown_period)
  {
    cooldown_timer -= cooldown_period;
    for (int channel(0); channel < num_channels; ++channel)
    {
      // Allow an additional strike (up to max_strikes)
      if (strikes_remaining[channel] < max_strikes)
      {
        strikes_remaining[channel]++;
      }
    }
  }

  // Stop strike after timeout
  for (int channel(0); channel < num_channels; ++channel)
  {
    if (striking[channel] && (strike_timer[channel] >= strike_time))
    {
      striking[channel] = false; // We are no longer striking
      strike_timer[channel] = 0; // Start settle timer
      analogWrite(chime_pins[channel], 0); // Turn off PWM
    }
  }

  // Blink LED on I/O traffic
  if (message_blink_timer >= blink_time)
  {
    digitalWrite(led_pin, LOW);
  }
  else
  {
    digitalWrite(led_pin, HIGH);
  }

  // Handle USB I/O
  if (usb.available() > 0)
  {
    const char incomingByte(static_cast<char>(usb.read()));
    message_blink_timer = 0;

    switch (incomingByte)
    {
    default:
      // Echo unknown commands (sanity check the serial link)
      usb.print(incomingByte);
      break;
    }
  }
}

void i2c_receive(size_t numBytes)
{
  // We are expecting 3 bytes for a strike command
  if (numBytes == 3)
  {
    // Get channel
    uint8_t channel = Wire.read();

    // Get duty cycle value
    uint8_t buffer[2];
    buffer[0] = Wire.read();
    buffer[1] = Wire.read();
    uint16_t duty_cycle;
    memcpy(&duty_cycle, buffer, 2);

    // Strike the chime (or so attempt)
    strike_chime(channel, duty_cycle);
  }
  // TODO: handle errors?
  message_blink_timer = 0;
}

void i2c_requested()
{
  Wire.write(0x8);
  Wire.write(0xD);
  message_blink_timer = 0;
}

void strike_chime(const uint8_t& channel, const uint16_t& duty_cycle)
{
  // Diag: spit out the results
  float dc = static_cast<float>(duty_cycle) / static_cast<float>((1 << pwm_bits) - 1) * 100.0;
  usb.print("Got strike command for channel ");
  usb.print(channel, DEC);
  usb.print(", PWM Duty Cycle: ");
  usb.print(dc);
  usb.print("% (");
  usb.print(duty_cycle);
  usb.print("/");
  usb.print((1 << pwm_bits) - 1);
  usb.println(")");

  // Verify we are not already striking the chime, and the settle has completed
  if (!striking[channel] && (strike_timer[channel] >= settle_time))
  {
    // Check for overheat
    if (strikes_remaining[channel] > 0)
    {
      striking[channel] = true;
      strike_timer[channel] = 0;
      strikes_remaining[channel]--;
      analogWrite(chime_pins[channel], duty_cycle);
    }
    else
    {
      // Overheated
      // TODO: log this, report to master, etc.
      usb.print("Channel ");
      usb.print(channel, DEC);
      usb.println(" overheated! Skipped strike.");
    }
  }
  else
  {
    // Already striking or settling
    // TODO: Log this, report to master, etc.
    usb.print("Already striking/settling channel ");
    usb.print(channel, DEC);
    usb.println("! Skipped strike.");
  }
}
