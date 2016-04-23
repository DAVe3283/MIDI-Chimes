// -----------------------------------------------------------------------------
// Slave.ino
//
// Program for all the slave Teensy units that run the MIDI chimes coils.
// Each slave is responsible for running a number of chimes, and keeping them
// from overheating.
//
// TODO:
// * Handle address auto-assignment on startup
// * It looks like interrupts can fire inside setup()...
//   * I should probably attach the I2C ISRs at the end then.
// * wait for power supply to get enabled at startup, then settle, then measure channels
//   * Should this be part of the startup sequence before allowing normal operation?
// * periodically check if the power supply is enabled
//   * if not, don't mark channels as shorted/open.
//   * Probably just don't run the strike routine at all, so the verification never happens
// * if a channel becomes shorted, turn off the master power supply
// * Should I implement some sort of cumulative current limit to protect the PCB?
//   If each coil is 2A, and all 10 are told to fire, that's 20A through the PCB traces...
//   Now imagine if they are 10A each O_o
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <i2c_t3.h>


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// General I/O pins
const uint8_t ps_enable_pin(0);     // Power supply enable pin
const uint8_t addr_latch_i_pin(1);  // Address latch input pin
const uint8_t addr_latch_o_pin(2);  // Address latch output pin
const uint8_t led_pin(13);          // LED pin

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
  A12, // Channel 1 (on back)
  A15, // Channel 2 (pin 26 on back)
  A16, // Channel 3 (pin 27 on back)
  A17, // Channel 4 (pin 28 on back)
  A14, // Channel 5 (bottom row)
   A0, // Channel 6 (pin 14)
   A1, // Channel 7 (pin 15)
   A2, // Channel 8 (pin 16)
   A3, // Channel 9 (pin 17)
  A19, // Channel 10 (pin 30 on back)
};

// Timeouts
const uint16_t strike_time( 90); // milliseconds
const uint16_t settle_time(160); // milliseconds
const uint16_t blink_time(  20); // milliseconds

// PWM Config
const uint8_t pwm_bits(12);
const uint32_t pwm_freq(8789); // Hz

// Voltage Divider
// I am using a divider of 160k and 10k, giving (160k+10k):10k, or 17:1.
const float divider_ratio(17.0f);
// We are using the Teensy's 1.2V stable internal reference for better accuracy.
// That means we can read 0 - 20.4 volts.

// We can do a 13-bit read on the Teensy 3.1 & 3.2
const uint8_t analog_read_bits(13);

// Transistor characteristics (for diagnostics)
// Voltages based on experimental & datasheet values for 2N6387 transistor & my coils
const float transistor_Vce_drop(  1.1f); // Transistor V_CE when on with my coils (measured value, datasheet spec is <= 2V)
const float connected_min_voltage(0.5f); // Voltages above this at startup are considered connected
const float shorted_voltage(      1.5f); // Voltages below this are considered shorted (regardless of ps_setpoint)
// Error limits (determined experimentally)
const float max_error_percent(          0.03f); // Max error (ideal - real feedback) as a % of ps_setpoint before evaluating shorts/opens
const float shorted_percent_ps_setpoint(0.95f); // Voltages lower than this % of ps_setpoint when commanded off are considered shorted
const float open_percent_ps_setpoint(   0.98f); // Voltages higher than this % of ps_setpoint when commanded on are considered open circuit

// States channels can be in
enum channel_state_t : uint8_t
{
  channel_working = 0,
  channel_disconnected = 1,
  channel_failed_short = 2,
  channel_failed_open = 3,
};


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
// ISRs
void i2c_startup(size_t numBytes);
void i2c_receive(size_t numBytes);
void i2c_startup_ack();
void i2c_status_requested();

// I2C address auto-assignment procedure
void i2c_addr_auto_assign();

// Startup diagnostics
void measure_ps_voltage();

// Strike a chime (handles thermals, settle time, etc.)
void strike_chime(const uint8_t& channel, const uint16_t& duty_cycle);

// Read the voltage on the given channel
float read_voltage(const uint8_t& channel);

// Check channel for shorts
bool is_shorted(const float& voltage);

// Handle a shorted channel (disable power supply)
void handle_shorted(const uint8_t& channel);

// Check channel for opens
bool is_open(const float& voltage);

// Verify transistors turn on/off
bool verify_on(const uint8_t& channel);
bool verify_off(const uint8_t& channel);


// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// I2C
volatile uint8_t i2c_address(0); // Assigned during startup
volatile bool i2c_ready(false); // Only goes true once we have completed address auto-assignment routine

// Timers
elapsedMillis cooldown_timer;
elapsedMillis strike_timer[num_channels];
elapsedMillis message_blink_timer;

// USB Debug
usb_serial_class usb = usb_serial_class();
bool debug(false);

// Are we striking currently?
volatile bool striking[num_channels];

// What Duty Cycle (%) are we commanding for the channel?
volatile float set_dc[num_channels];

// Have we verified the voltage is reasonable (coil & transistor are working)?
volatile bool verified[num_channels];

// Thermal load limiter
volatile int8_t strikes_remaining[num_channels];

// Power supply measurements
//bool ps_enabled(false); // Is the power supply currently enabled?
float ps_voltage[num_channels]; // Last measured voltage
float ps_setpoint(0); // Guess power supply setpoint (max measured voltage at startup)

// Channel diagnostics
channel_state_t channel_state[num_channels];
uint8_t num_connected_channels(0);
bool any_shorted(false); // Are any channels shorted?


// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  // Configure address latch pins
  pinMode(addr_latch_i_pin, INPUT);
  pinMode(addr_latch_o_pin, OUTPUT);
  digitalWrite(addr_latch_o_pin, LOW);

  // High-Z the PS_Enable pin
  pinMode(ps_enable_pin, INPUT);

  // Configure LED
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH); // On until we have an address and I2C is ready

  // Configure analog inputs
  analogReadResolution(analog_read_bits);
  analogReference(INTERNAL); // Set reference voltage to 1.2V

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

  // Diagnostics
  usb.begin(9600);

  // I2C address auto-config
  i2c_addr_auto_assign();
  // Master won't release us to continue until the power supply is stable

  // Measure power supply voltages at boot, look for connected channels
  measure_ps_voltage();

  // Calculate shorted voltage threshold at startup (possibly more aggressive than during operation)
  float shorted_level(shorted_voltage);
  if (shorted_level < (ps_setpoint * shorted_percent_ps_setpoint))
  {
    shorted_level = ps_setpoint * shorted_percent_ps_setpoint;
  }

  // Look for shorted channels at startup
  for (int channel(0); channel < num_channels; ++channel)
  {
    // Only worry about channels that are connected
    if ((channel_state[channel] != channel_disconnected) && (ps_voltage[channel] < shorted_level))
    {
      channel_state[channel] = channel_failed_short;
      any_shorted = true;
    }
  }

  // Startup complete
  digitalWrite(led_pin, LOW); // Indicate normal operation
}

// Main program loop
void loop()
{
  // Handle cooldown
  if (cooldown_timer >= cooldown_period)
  {
    cooldown_timer -= cooldown_period;
    noInterrupts(); // strikes_remaining can change inside the I2C ISR
    for (int channel(0); channel < num_channels; ++channel)
    {
      // Allow an additional strike (up to max_strikes)
      if (strikes_remaining[channel] < max_strikes)
      {
        strikes_remaining[channel]++;
      }
    }
    interrupts(); // re-enable ISR
  }

  // Handle strike and voltage measurement
  noInterrupts(); // striking, verified, & strike_timer can change inside the I2C ISR
  for (int channel(0); channel < num_channels; ++channel)
  {
    if (striking[channel])
    {
      // Measure feedback voltage halfway through strike time
      if (!verified[channel] && (strike_timer[channel] >= (strike_time / 2)))
      {
        verify_on(channel);
        // TODO: handle failure?
      }

      // Stop strike after timeout
      if (strike_timer[channel] >= strike_time)
      {
        striking[channel] = false; // We are no longer striking
        verified[channel] = false; // We need to verify it turns off
        strike_timer[channel] = 0; // Start settle timer
        analogWrite(chime_pins[channel], 0); // Turn off PWM
      }
    }
    else
    {
      // Measure power supply voltage at the end of settle time
      // Note: Theoretical risk of missing this measurement if a strike is
      //       commanded exactly as settle time runs out, and we missed it on
      //       the previous pass through. But we don't care
      if (!verified[channel] && (strike_timer[channel] >= settle_time))
      {
        verify_off(channel);
        // TODO: handle failure?
      }
    }
  }
  interrupts(); // re-enable ISR

  // Blink LED on I/O traffic
  if (message_blink_timer >= blink_time)
  {
    digitalWrite(led_pin, LOW);
  }
  else
  {
    digitalWrite(led_pin, HIGH);
  }

  // USB Debug
  if (usb.available() > 0)
  {
    const char incomingByte(static_cast<char>(usb.read()));
    message_blink_timer = 0;

    // Enable debug mode
    if (!debug)
    {
      debug = true;
      usb.println("Enabled USB debug output.");
    }

    switch (incomingByte)
    {
    case '\r':
      usb.println();
      // Testing
      for (int channel(0); channel < num_channels; ++channel)
      {
        usb.print("Channel ");
        usb.print(channel + 1);
        usb.print(" state is ");
        usb.print(channel_state[channel]);
        usb.print(", & last measured ");
        usb.print(ps_voltage[channel]);
        usb.println("V.");
      }
      usb.print("Power supply voltage setpoint estimated to be ");
      usb.print(ps_setpoint);
      usb.println("V.");
      break;

    case '0':
      strike_chime(4, (1 << pwm_bits) - 1);
      break;

    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      strike_chime(4, (1 << pwm_bits) * (incomingByte - '0') / 10);
      break;

    case 'r':
    case 'R':
      usb.print("Channel 5 currently measures ");
      usb.print(read_voltage(4));
      usb.println("V.");
      break;

    case 's':
    case 'S':
      i2c_status_requested();
      break;

    default:
      // Echo unknown commands (sanity check the serial link)
      usb.print(incomingByte);
      break;
    }
  }
}

void i2c_startup(size_t numBytes)
{
  usb.print("Got ");
  usb.print(numBytes);
  usb.println(" bytes of I2C data.");
  // We are expecting 2 bytes (command, value)
  if (numBytes == 2)
  {
    const uint8_t command(Wire.read());
    const uint8_t value(Wire.read());
    usb.print("Got command 0x");
    usb.print(command, HEX);
    usb.print(", value = 0x");
    usb.println(value, HEX);

    switch (command)
    {
      // Potential new address
      case 0x00:
        i2c_address = value;
        break;

      // Requesting a latch
      case 0x01:
        digitalWrite(addr_latch_o_pin, value);
        break;

      // Startup complete
      case 0x02:
        i2c_ready = true;
        break;

      // Unknown commands
      default:
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

void i2c_startup_ack()
{
  // Write ACK
  Wire.write(0x06);
}

void i2c_status_requested()
{
  // Create buffer to hold message
  uint8_t buffer[6 + (4 * num_channels)];

  // Header
  buffer[0] = 0; // Message version
  buffer[1] = pwm_bits;
  buffer[2] = num_channels;
  buffer[3] = num_connected_channels;

  // Power supply setpoint
  uint16_t ps_mV(static_cast<uint16_t>(ps_setpoint * 1000.0f));
  memcpy(buffer + 4, &ps_mV, sizeof(ps_mV));

  // Per-channel data
  for (int channel(0); channel < num_channels; ++channel)
  {
    // Channel state
    buffer[6 + channel] = static_cast<uint8_t>(channel_state[channel]);

    // Strikes remaining before overheat
    buffer[16 + channel] = strikes_remaining[channel];

    // Last measured voltage (mV)
    const uint16_t mV(static_cast<uint16_t>(ps_voltage[channel] * 1000.0f));
    memcpy(buffer + 26 + (sizeof(mV) * channel), &mV, sizeof(mV));
  }

  // Write status packet
  const size_t wrote(Wire.write(buffer, sizeof(buffer)));
  message_blink_timer = 0;

  if (debug && (wrote != sizeof(buffer)))
  {
    usb.print("I2C transmission failed! Tried to write ");
    usb.print(sizeof(buffer));
    usb.print(" bytes, but was only able to write ");
    usb.print(wrote);
    usb.println(" bytes.");
  }
}

void i2c_addr_auto_assign()
{
  // Watch for address broadcast
  noInterrupts();
  Wire.begin(I2C_SLAVE, i2c_address, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  Wire.onReceive(i2c_startup); // Handle startup messages
  interrupts();

  // Wait for ADDR_LATCH before committing the address
  while ((digitalRead(addr_latch_i_pin) == LOW) || (i2c_address == 0)) {};

  // Switch to the new address
  noInterrupts();
  Wire.begin(I2C_SLAVE, i2c_address, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  Wire.onRequest(i2c_startup_ack); // Ready to send ACK when requested
  interrupts();

  // Wait for the master to say we are done
  while (!i2c_ready) {};

  // I2C setup complete, switch to runtime ISRs
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_status_requested);
}

void measure_ps_voltage()
{
  // Measure power supply voltages
  for (int channel(0); channel < num_channels; ++channel)
  {
    // Get current voltage
    const float voltage(read_voltage(channel));
    ps_voltage[channel] = voltage;
    verified[channel] = true;

    // We assume the power supply setpoint is the max measured voltage
    if (voltage > ps_setpoint)
    {
      ps_setpoint = voltage;
    }

    // Determine if the channel is connected to anything
    if (voltage > connected_min_voltage)
    {
      channel_state[channel] = channel_working;
      num_connected_channels++;
    }
    else
    {
      channel_state[channel] = channel_disconnected;
    }
  }
}

void strike_chime(const uint8_t& channel, const uint16_t& duty_cycle)
{
  const float dc(static_cast<float>(duty_cycle) / static_cast<float>((1 << pwm_bits) - 1));
  if (debug)
  {
    usb.print("Got strike command for channel ");
    usb.print(channel + 1, DEC);
    usb.print(", PWM Duty Cycle: ");
    usb.print(dc * 100.0f);
    usb.print("% (");
    usb.print(duty_cycle);
    usb.print("/");
    usb.print((1 << pwm_bits) - 1);
    usb.println(")");
  }

  // Verify channel is sane (prevent buffer overflow)
  if (channel >= num_channels)
  {
    // TODO: log error?
    if (debug)
    {
      usb.print("We only have ");
      usb.print(num_channels, DEC);
      usb.print(" channels; ignoring this request!");
    }
    return;
  }

  // Verify the channel is working
  if (channel_state[channel] != channel_working)
  {
    if (debug)
    {
      usb.print("Channel is not working (channel_state = ");
      usb.print(channel_state[channel], DEC);
      usb.print("); ignoring this request!");
    }
    return;
  }

  // Verify we are not already striking the chime, and the settle has completed
  if (!striking[channel] && (strike_timer[channel] >= settle_time))
  {
    // Check for overheat
    if (strikes_remaining[channel] > 0)
    {
      // Strike the chime!
      striking[channel] = true; // We are now striking
      verified[channel] = false; // We need to verify it turns on
      strike_timer[channel] = 0; // Start strike timer
      strikes_remaining[channel]--; // We used an available strike
      analogWrite(chime_pins[channel], duty_cycle); // Turn on PWM
      set_dc[channel] = dc; // Store the PWM DC for channel validation
    }
    else
    {
      // Overheated
      // TODO: log this, report to master, etc.
      if (debug)
      {
        usb.print("Channel ");
        usb.print(channel + 1, DEC);
        usb.println(" overheated! Skipped strike.");
      }
    }
  }
  else
  {
    // Already striking or settling
    // TODO: Log this, report to master, etc.
    if (debug)
    {
      usb.print("Already striking/settling channel ");
      usb.print(channel + 1, DEC);
      usb.println("! Skipped strike.");
    }
  }
}

float read_voltage(const uint8_t& channel)
{
  const float analog_raw(analogRead(feedback_pins[channel]));
  const float analog_max((1 << analog_read_bits) - 1);
  float voltage
    = 1.2f // reference voltage
    * divider_ratio // times the divider
    * analog_raw / analog_max; // times the measured ratio
  return voltage;
}

bool is_shorted(const float& voltage)
{
  return voltage < (ps_setpoint * shorted_percent_ps_setpoint);
}

void handle_shorted(const uint8_t& channel)
{
  // Log shorted status
  channel_state[channel] = channel_failed_short;
  any_shorted = true;

  // Disable power supply
  pinMode(ps_enable_pin, OUTPUT);
  digitalWrite(ps_enable_pin, LOW);

  // TODO: remove debug
  usb.print("Channel ");
  usb.print(channel + 1);
  usb.println(" is shorted!");
}

bool is_open(const float& voltage)
{
  return voltage > (ps_setpoint * open_percent_ps_setpoint);
}

bool verify_on(const uint8_t& channel)
{
  // Measure current feedback voltage
  const float voltage(read_voltage(channel));

  // Ideal voltage is (ps_setpoint - transistor_Vce_drop) * (1 - set_dc[channel]) + transistor_Vce_drop
  // That can be simplified to save some instructions:
  const float ideal_voltage(ps_setpoint - ((ps_setpoint - transistor_Vce_drop) * set_dc[channel]));

  // Error between ideal and measured voltage
  const float error((ideal_voltage - voltage) / ps_setpoint);

  // Verify transistor is in expected range
  const bool in_range((error < max_error_percent) && (error > -max_error_percent));
  verified[channel] = true;

  // Handle failures
  if (!in_range)
  {
    // Check if channel failed short circuit
    if (is_shorted(voltage))
    {
      handle_shorted(channel);
    }

    // Check if channel failed open circuit
    else if (is_open(voltage))
    {
      channel_state[channel] = channel_failed_open;
      // TODO: remove debug
      usb.print("Channel ");
      usb.print(channel + 1);
      usb.println(" won't turn on!");
    }
  }

  if (debug)
  {
    usb.print("Verifying channel ");
    usb.print(channel + 1);
    usb.println(" during strike:");
    usb.print("    Ideal voltage: ");
    usb.print(ideal_voltage);
    usb.println("V");
    usb.print("    Measured voltage: ");
    usb.print(voltage);
    usb.println("V");
    usb.print("    Error: ");
    usb.print(error * 100.0f);
    usb.println("%");
    if (!in_range)
    {
      usb.println("    **** NOT IN EXPECTED RANGE! ****");
    }
  }

  return in_range;
}

bool verify_off(const uint8_t& channel)
{
  ps_voltage[channel] = read_voltage(channel);
  const float error((ps_setpoint - ps_voltage[channel]) / ps_setpoint);

  // Verify transistor is in expected range
  const bool in_range((error < max_error_percent) && (error > -max_error_percent));
  verified[channel] = true;

  // Handle failures
  if (!in_range)
  {
    // Check if channel failed short circuit
    if (is_shorted(ps_voltage[channel]))
    {
      handle_shorted(channel);
    }

    // Ignore "opens", because that is what we want. It is probably just flyback
    // voltage anyway.
  }

  if (debug)
  {
    usb.print("Verifying channel ");
    usb.print(channel + 1);
    usb.println(" after settle:");
    usb.print("    ps_setpoint: ");
    usb.print(ps_setpoint);
    usb.println("V");
    usb.print("    Measured voltage: ");
    usb.print(ps_voltage[channel]);
    usb.println("V");
    usb.print("    Error: ");
    usb.print(error * 100.0f);
    usb.println("%");
    if (!in_range)
    {
      usb.println("    **** NOT IN EXPECTED RANGE! ****");
    }
  }

  return in_range;
}
