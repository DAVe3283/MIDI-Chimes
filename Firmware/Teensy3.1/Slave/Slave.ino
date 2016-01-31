// -----------------------------------------------------------------------------
// Slave.ino
//
// Program for all the slave Teensy units that run the MIDI chimes coils.
// Each slave is responsible for running a number of chimes, and keeping them
// from overheating.
//
// TODO:
// * log errors
// * report status to master
// * measure voltages (on demand? between strikes?)
// * diagnose transistors
//   * verify voltage is high when PWM is off (@ bootup?)
//   * verify voltage is low halfway through strike time
//   * verify voltage is high halfway through settle time
//   * be able to handle the powersupply being turned off to save power
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
   3, // Channel 1
   4, // Channel 2
   5, // Channel 3
   6, // Channel 4
   9, // Channel 5
  10, // Channel 6
  20, // Channel 7
  21, // Channel 8
  22, // Channel 9
  23, // Channel 10
};
const uint8_t num_channels(sizeof(chime_pins) / sizeof(*chime_pins));

// Analog pins used for feedback
const uint8_t feedback_pins[num_channels] =
{
   A0, // Channel 1 (pin 14)
   A1, // Channel 2 (pin 15)
   A2, // Channel 3 (pin 16)
   A3, // Channel 4 (pin 17)
       // Pins 18-19 are I2C!
  A15, // Channel 5 (on back)
  A16, // Channel 6 (on back)
  A17, // Channel 7 (on back)
  A18, // Channel 8 (on back)
  A19, // Channel 9 (on back)
  A20, // Channel 10 (on back)
};

const uint8_t led_pin(13);

// Timeouts
uint16_t strike_time(90); // milliseconds
uint16_t settle_time(160); // milliseconds
const uint16_t blink_time(20000); // microseconds

// PWM Config
const uint8_t pwm_bits(12);
const uint32_t pwm_freq(8789); // Hz

// Voltage Divider
// I am using a divider of 160k and 10k, giving (160k+10k):10k, or 17:1.
const float divider_ratio(17.0);
// We are using the Teensy's 1.2V stable internal reference for better accuracy.
// That means we can read 0 - 20.4 volts.

// We can do a 13-bit read on the Teensy 3.1 & 3.2
const uint8_t analog_read_bits(13);


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

// Read the voltage on the given channel
float read_voltage(const uint8_t& channel);

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

// Measured coil voltages at startup (testing)
float startup_voltage[num_channels];


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

  // Configure analog inputs
  analogReadResolution(analog_read_bits);
  analogReference(INTERNAL); // Set reference voltage to 1.2V

  // Measure startup voltages (testing)
  for (int channel(0); channel < num_channels; ++channel)
  {
    startup_voltage[channel] = read_voltage(channel);
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
    case '\r':
      usb.println();
      // Testing
      for (int channel(0); channel < num_channels; ++channel)
      {
        usb.print("Channel ");
        usb.print(channel);
        usb.print(" measured ");
        usb.print(startup_voltage[channel]);
        usb.println("V at startup.");
      }
      break;

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

  // Verify channel is sane (prevent buffer overflow)
  if (channel >= num_channels)
  {
    usb.print("We only have ");
    usb.print(num_channels, DEC);
    usb.print(" channels (0 - ");
    usb.print(num_channels - 1, DEC);
    usb.println("), ignoring this request!");
    // TODO: log error?
    return;
  }

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

float read_voltage(const uint8_t& channel)
{
  const float analog_raw(analogRead(feedback_pins[channel]));
  const float analog_max((1 << analog_read_bits) - 1);
  float voltage
    = 1.2 // reference voltage
    * divider_ratio // times the divider
    * analog_raw / analog_max; // times the measured ratio
  return voltage;
}
