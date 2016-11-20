// -----------------------------------------------------------------------------
// FakeSlave.ino
//
// Program to enable a single Teensy to pretend to be multiple slaves.
// This allows for easy debugging of the Master and I²C communication, and
// mainly if the appropriate notes are being struck based on what you expect.
//
// Usage: Compile as USB Serial, connect a terminal to it, go nuts!
//
// TODO:
//  Make it work.
//  * Track power supply enable status.
//  * Have a boot mode (or auto-detect booting?)
//  * Allow faking a channel verification to whatever state
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <i2c_t3.h>


// -----------------------------------------------------------------------------
// Enums
// -----------------------------------------------------------------------------
// States channels can be in
enum channel_state_t : uint8_t
{
  channel_working = 0,
  channel_disconnected = 1,
  channel_failed_short = 2,
  channel_failed_open = 3,
};


// -----------------------------------------------------------------------------
// Settings
// -----------------------------------------------------------------------------
const uint8_t num_slaves(3); // How many slaves we emulate
const uint8_t num_channels(10); // Channels per slave
const float virtual_ps_voltage(12.0f); // Voltage for the virtual power supply
channel_state_t channel_state[num_slaves][num_channels] =
{
  {
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
  },
  {
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
  },
  {
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
    channel_working,
  },
};

// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// General I/O pins
const uint8_t ps_enable_pin(0);     // Power supply enable pin
const uint8_t addr_latch_i_pin(1);  // Address latch input pin
const uint8_t addr_latch_o_pin(2);  // Address latch output pin
const uint8_t led_pin(13);          // LED pin

// Timeouts
const uint32_t strike_time( 90); // milliseconds
const uint32_t settle_time(155); // milliseconds
const uint32_t blink_time(  20); // milliseconds
// Power supply timeouts
const uint32_t ps_stable_time(2500); // milliseconds, datasheet spec for power supply
const uint32_t ps_toggle_time( 100); // milliseconds, datasheet spec for watchdog

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
const float connected_min_voltage(0.1f); // Voltages above this at startup are considered connected
const float shorted_voltage(      1.5f); // Voltages below this are considered shorted (regardless of ps_setpoint)
// Error limits (determined experimentally)
const float base_error(          0.02f); // Base allowed error during strike / settle
const float per_strike_error(    0.02f); // Extra error allowed during strike for each additional coil on simultaneously
const float shorted_percent_ps_setpoint(0.95f); // Voltages lower than this % of ps_setpoint when commanded off are considered shorted
const float open_percent_ps_setpoint(   0.98f); // Voltages higher than this % of ps_setpoint when commanded on are considered open circuit

// State power supply can be in
enum ps_state_t
{
  stable, // has been on for at least ps_stable_time
  stabilizing, // has just turned on
  disabled, // appears to be disabled
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
void check_all_ready();
void i2c_receive(size_t numBytes);
void i2c_startup_ack();
void i2c_status_requested();
void ps_en_isr();

// Update power supply state
void ps_state_update();

// I2C address auto-assignment procedure
void i2c_addr_auto_assign();

// Startup diagnostics
void measure_ps_voltage();

// Strike a chime (handles thermals, settle time, etc.)
void strike_chime(const uint8_t& slave_no, const uint8_t& channel, const uint16_t& duty_cycle);

// Check channel for shorts
bool is_shorted(const float& voltage);

// Handle a shorted channel (disable power supply)
void handle_shorted(const uint8_t& slave_no, const uint8_t& channel);

// Check channel for opens
bool is_open(const float& voltage);

// Verify transistors turn on/off
bool verify_on(const uint8_t& slave_no, const uint8_t& channel);
bool verify_off(const uint8_t& slave_no, const uint8_t& channel);

// Debug
void print_state(Print& out, const channel_state_t& state);
void print_state(Print& out, ps_state_t state);

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// I2C
volatile uint8_t i2c_address(0); // Assigned during startup
uint8_t i2c_address_base(0); // Lowest address we will respond to
volatile bool i2c_ready[num_slaves]; 
volatile bool i2c_all_ready(false); // Only goes true once we have completed address auto-assignment routine

// Timers
elapsedMillis cooldown_timer;
elapsedMillis strike_timer[num_slaves][num_channels];
elapsedMillis message_blink_timer;
elapsedMillis ps_stable_timer;

// USB Debug
usb_serial_class usb = usb_serial_class();
uint8_t debug_level(2);

// Are we striking currently?
volatile bool striking[num_slaves][num_channels];

// What Duty Cycle (%) are we commanding for the channel?
volatile float set_dc[num_slaves][num_channels];

// Have we verified the voltage is reasonable (coil & transistor are working)?
volatile bool verified[num_slaves][num_channels];

// Thermal load limiter
volatile int8_t strikes_remaining[num_slaves][num_channels];

// Power supply measurements
volatile ps_state_t ps_state(disabled); // Current power supply state
volatile uint32_t ps_stabilize_time(0); // How long the power supply has been stabilizing (microseconds)
float ps_voltage[num_slaves][num_channels]; // Last measured voltage
float ps_setpoint(0); // Guess power supply setpoint (max measured voltage at startup)

// Channel diagnostics
uint8_t num_connected_channels(0);
volatile bool any_shorted(false); // Are any channels shorted?


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

  // Configure LED
  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH); // On until we have an address and I2C is ready

  // Configure analog inputs
  analogReadResolution(analog_read_bits);
  analogReference(INTERNAL); // Set reference voltage to 1.2V

  // Configure PWM channels
  analogWriteResolution(pwm_bits);
  for (uint8_t slave_no(0); slave_no < num_slaves; ++slave_no)
  {
    for (uint8_t channel(0); channel < num_channels; ++channel)
    {
      striking[slave_no][channel] = false;
      strikes_remaining[slave_no][channel] = max_strikes;
    }
  }

  // Diagnostics
  usb.begin(9600);
  while (!usb)
  {
    delay(800);
    digitalWrite(led_pin, LOW);
    delay(200);
    digitalWrite(led_pin, HIGH);
  }
  usb.println("USB ready, now ready to begin startup sequence.");

  // Attach PS_EN interrupt
  pinMode(ps_enable_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(ps_enable_pin), ps_en_isr, RISING);

  // I2C address auto-config
  i2c_addr_auto_assign();

  // Wait for power supply to stabilize
  while (ps_state != stable)
  {
    ps_state_update();
  }

  // Measure power supply voltages at boot, look for connected channels
  measure_ps_voltage();

  // Calculate shorted voltage threshold at startup (possibly more aggressive than during operation)
  float shorted_level(shorted_voltage);
  if (shorted_level < (ps_setpoint * shorted_percent_ps_setpoint))
  {
    shorted_level = ps_setpoint * shorted_percent_ps_setpoint;
  }

  // Wait for the master to say we are done
  while (!i2c_all_ready) {};
  usb.println("All virtual slaves are ready! Startup complete.");

  // POST complete, switch to runtime ISRs
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_status_requested);

  // Startup complete
  digitalWrite(led_pin, LOW); // Indicate normal operation
}

// Main program loop
void loop()
{
  // Check power supply status
  ps_state_update();

  // Handle cooldown
  if (cooldown_timer >= cooldown_period)
  {
    cooldown_timer -= cooldown_period;
    noInterrupts(); // strikes_remaining can change inside the I2C ISR
    for (uint8_t slave_no(0); slave_no < num_slaves; ++slave_no)
    {
      for (int channel(0); channel < num_channels; ++channel)
      {
        // Allow an additional strike (up to max_strikes)
        if (strikes_remaining[slave_no][channel] < max_strikes)
        {
          strikes_remaining[slave_no][channel]++;
        }
      }
    }
    interrupts(); // re-enable ISR
  }

  // Handle strike and voltage measurement
  noInterrupts(); // striking, verified, & strike_timer can change inside the I2C ISR
  for (uint8_t slave_no(0); slave_no < num_slaves; ++slave_no)
  {
    for (uint8_t channel(0); channel < num_channels; ++channel)
    {
      if (striking[slave_no][channel])
      {
        // Measure feedback voltage halfway through strike time
        if (!verified[slave_no][channel] && (strike_timer[slave_no][channel] >= (strike_time / 2)))
        {
          verify_on(slave_no, channel);
        }

        // Stop strike after timeout
        if (strike_timer[slave_no][channel] >= strike_time)
        {
          striking[slave_no][channel] = false; // We are no longer striking
          verified[slave_no][channel] = false; // We need to verify it turns off
          strike_timer[slave_no][channel] = 0; // Start settle timer
          // analogWrite(chime_pins[channel], 0); // Turn off PWM
        }
      }
      else
      {
        // Measure power supply voltage at the end of settle time
        if (!verified[slave_no][channel] && (strike_timer[slave_no][channel] >= settle_time))
        {
          verify_off(slave_no, channel);
        }
      }
    }
  }
  interrupts(); // re-enable ISR

  // Blink LED on I/O traffic
  if (!any_shorted && (message_blink_timer >= blink_time))
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

    switch (incomingByte)
    {
    // Help
    case '?':
    case '/':
    case '\r':
      usb.println("MIDI Chimes Slave Board");
      usb.print("Compiled: ");
      usb.print(__DATE__);
      usb.print(" ");
      usb.println(__TIME__);
      usb.println();
      usb.printf(  "I²C addresses = 0x00,%02X-%02X", i2c_address_base, i2c_address_base + num_slaves - 1);
      usb.println();
      usb.println("Commands:");
      usb.println("    R   = Reset, wait for address auto-assignment");
      usb.println("    S   = Display status");
      usb.println("    1-9 = Set debug output to the specified level");
      usb.println("    0   = Disable debug output");
      usb.println("    J   = Force channel 1 to channel_disconnected");
      usb.println("    K   = Force channel 1 to channel_failed_short");
      usb.println("    L   = Force channel 1 to channel_failed_open");
      usb.println();
      break;

    // Reset
    case 'r':
    case 'R':
      usb.println("Reset! Please power on the master now.");
      i2c_addr_auto_assign();
      while (!i2c_all_ready) {};
      Wire.onReceive(i2c_receive);
      Wire.onRequest(i2c_status_requested);
      usb.println("All virtual slaves are ready! Reset complete.");
      break;

    // Display current status
    case 's':
    case 'S':
      usb.print("Power supply state: ");
      print_state(usb, ps_state);
      usb.println();
      if (any_shorted)
      {
        usb.println("Shorted channel(s) found; power supply forcibly disabled by this slave!");
      }
      usb.print("Power supply voltage setpoint (estimated at boot): ");
      usb.print(ps_setpoint);
      usb.println(" V");
      usb.println();
      // for (int channel(0); channel < num_channels; ++channel)
      // {
      //   usb.print("Channel ");
      //   if (channel < 9)
      //   {
      //     usb.print(" ");
      //   }
      //   usb.print(channel + 1);
      //   usb.print(" last measured ");
      //   if (ps_voltage[channel] < 10.0f)
      //   {
      //     usb.print(" ");
      //   }
      //   usb.print(ps_voltage[channel]);
      //   usb.print(" V; state = ");
      //   usb.print(channel_state[channel]);
      //   usb.print(" (");
      //   print_state(usb, channel_state[channel]);
      //   usb.println(")");
      // }
      usb.println();
      break;

    // Select USB debug output level
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
      debug_level = incomingByte - '0';
      usb.print("USB debug ");
      if (debug_level > 0)
      {
        usb.print("ENABLED (Level ");
        usb.print(debug_level);
        usb.println(").");
      }
      else
      {
        usb.println("DISABLED.");
      }
      usb.println();
      break;

    // Force channel 1 state
    case 'j':
    case 'J':
      usb.println("Forced channel 1 state to channel_disconnected.");
      channel_state[0][0] = channel_disconnected;
      break;
    case 'k':
    case 'K':
      usb.println("Forced channel 1 state to channel_failed_short.");
      handle_shorted(0, 0);
      break;
    case 'l':
    case 'L':
      usb.println("Forced channel 1 state to channel_failed_open.");
      channel_state[0][0] = channel_failed_open;
      break;

    // Unknown commands
    default:
      usb.println("Press '?' for help.");
      usb.println();
      break;
    }
  }
}

void i2c_startup(size_t numBytes)
{
  noInterrupts();
  const uint8_t target(Wire.getRxAddr()); // What slave address was used
  usb.printf("Target 0x%02X got %d bytes: ", target, numBytes);

  // We are expecting 2 bytes (command, value)
  if (numBytes == 2)
  {
    const uint8_t command(Wire.read());
    const uint8_t value(Wire.read());
    if (target == 0x00)
    {
       switch (command)
      {
        // Potential new address
        case 0x00:
          i2c_address = value;
          usb.printf("Potential new I²C address: 0x%02X", i2c_address);
          break;

        // Unknown commands
        default:
          usb.print("Unknown command.");
          break;
      }
    }
    else
    {
      const uint8_t slave_no(target - i2c_address_base);
      usb.printf("(virtual slave %d) ", slave_no);
      if (slave_no < num_slaves)
      {
        switch (command)
        {
          // Requesting a latch
          case 0x01:
            digitalWrite(addr_latch_o_pin, value);
            usb.print("Latch ");
            if (value)
            {
              usb.print("ON.");
            }
            else
            {
              usb.print("OFF.");
            }
            break;

          // Startup complete
          case 0x02:
            i2c_ready[slave_no] = true;
            usb.print("This virtual slave is ready.");
            check_all_ready();
            break;

          // Unknown commands
          default:
            usb.print("Unknown command.");
            break;
        }
      }
      else
      {
        usb.print("Invalid virtual slave, ignoring.");
      }
    }
  }
  usb.println();
  interrupts();
}

void check_all_ready()
{
  bool all_ready(true);
  for (int i(0); i < num_slaves; ++i)
  {
    all_ready &= i2c_ready[i];
  }
  i2c_all_ready = all_ready;
}

void i2c_receive(size_t numBytes)
{
  const uint8_t target(Wire.getRxAddr()); // Slave address that was used
  const uint8_t slave_no(target - i2c_address_base); // Slave in use

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
    strike_chime(slave_no, channel, duty_cycle);
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
  const uint8_t target(Wire.getRxAddr()); // Slave address that was used
  const uint8_t slave_no(target - i2c_address_base); // Slave in use
  if (debug_level >= 3)
  {
    usb.printf("Status was requested from 0x%02X (slave %d).\r\n", target, slave_no);
  }

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
    buffer[6 + channel] = static_cast<uint8_t>(channel_state[slave_no][channel]);

    // Strikes remaining before overheat
    buffer[16 + channel] = strikes_remaining[slave_no][channel];

    // Last measured voltage (mV)
    const uint16_t mV(static_cast<uint16_t>(ps_voltage[slave_no][channel] * 1000.0f));
    memcpy(buffer + 26 + (sizeof(mV) * channel), &mV, sizeof(mV));
  }

  // Write status packet
  const size_t wrote(Wire.write(buffer, sizeof(buffer)));
  message_blink_timer = 0;

  if ((debug_level >= 1) && (wrote != sizeof(buffer)))
  {
    usb.print("I2C transmission failed! Tried to write ");
    usb.print(sizeof(buffer));
    usb.print(" bytes, but was only able to write ");
    usb.print(wrote);
    usb.println(" bytes.");
  }
}

void ps_en_isr()
{
  // We only get here when PS_EN had a rising edge

  // Power supply was disabled
  if (ps_state == disabled)
  {
    // We are starting to stabilize the power supply!
    ps_state = stabilizing;
    ps_stabilize_time = 0;
    if (debug_level >= 2)
    {
      usb.println("Power supply stabilizing...");
    }
  }

  // Power supply was stabilizing
  else if (ps_state == stabilizing)
  {
    // Did we get the edge in time?
    if (ps_stable_timer <= ps_toggle_time)
    {
      ps_stabilize_time += ps_stable_timer;
    }
    else
    {
      // Didn't get the edge in time, restart stabilizing process
      ps_stabilize_time = 0;
    }

    // Are we stable?
    if (ps_stabilize_time >= ps_stable_time)
    {
      // We are stable!
      ps_state = stable;
      if (debug_level >= 2)
      {
        usb.println("Power supply now stable!");
      }
    }
  }

  // Power supply was already stable
  else if (ps_state == stable)
  {
    // We stay stable unless we missed an edge
    if (ps_stable_timer > ps_toggle_time)
    {
      // Back to stabilizing
      ps_state = stabilizing;
      ps_stabilize_time = 0;
      if (debug_level >= 2)
      {
        usb.print("Power supply re-stabilizing (");
        usb.print(ps_stable_timer);
        usb.println(" us)...");
      }
    }
  }

  // Unknown state!
  else
  {
    // Should never get here!
    // Put ps_state back to a known state (start stabilizing)
    ps_state = stabilizing;
    ps_stabilize_time = 0;
  }

  // We handled an edge, so the timer resets no matter what
  ps_stable_timer = 0;
}

void ps_state_update()
{
  // Disable interrupts while handing power supply state (race conditions!)
  noInterrupts();

  // Have we missed any edges?
  if (ps_stable_timer > ps_toggle_time)
  {
    // Mark power supply as disabled, we missed an edge
    if (ps_state != disabled)
    {
      ps_state = disabled;
      if (debug_level >= 2)
      {
        usb.print("Watchdog timeout exceeded (");
        usb.print(ps_stable_timer);
        usb.println(" ms).");
        usb.println("Assuming power supply is now disabled.");
      }
    }
    ps_stable_timer = 0;
  }

  // We are done handling power supply state, re-enable interrupts
  interrupts();
}

void i2c_addr_auto_assign()
{
  // Indicate we aren't ready
  i2c_all_ready = false;
  for (uint8_t i(0); i < num_slaves; ++i)
  {
    i2c_ready[i] = false;
  }

  // Watch for address broadcast
  noInterrupts();
  i2c_address = 0x00;
  Wire.begin(I2C_SLAVE, i2c_address, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  Wire.onReceive(i2c_startup); // Handle startup messages
  Wire.onRequest(nullptr);
  interrupts();

  // Wait for ADDR_LATCH before committing the address
  while ((digitalRead(addr_latch_i_pin) == LOW) || (i2c_address == 0x00)) {};
  noInterrupts();
  i2c_address_base = i2c_address;
  usb.printf("Got address latch, acting as slaves 0x00,%02X-%02X.\r\n", i2c_address_base, i2c_address_base + num_slaves - 1);
  interrupts();

  // Switch to the new address
  noInterrupts();
  Wire.begin(I2C_SLAVE, 0, i2c_address_base + num_slaves - 1, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_1000);
  Wire.onRequest(i2c_startup_ack); // Ready to send ACK when requested
  usb.println("Ready to send ACK.");
  interrupts();
}

void measure_ps_voltage()
{
  for (uint8_t slave_no = 0; slave_no < num_slaves; ++slave_no)
  {
    // Measure power supply voltages
    for (uint8_t channel(0); channel < num_channels; ++channel)
    {
      // Get current voltage
      const float voltage(virtual_ps_voltage);
      ps_voltage[slave_no][channel] = voltage;
      verified[slave_no][channel] = true;

      // We assume the power supply setpoint is the max measured voltage
      if (voltage > ps_setpoint)
      {
        ps_setpoint = voltage;
      }
    }
  }
}

void strike_chime(const uint8_t& slave_no, const uint8_t& channel, const uint16_t& duty_cycle)
{
  const float dc(static_cast<float>(duty_cycle) / static_cast<float>((1 << pwm_bits) - 1));
  if (debug_level >= 1)
  {
    usb.printf("Strike commanded. Slave %d, channel %2d, PWM DC: %6.2f%% (%4d/%4d)\r\n",
      slave_no, channel + 1, dc * 100.0f, duty_cycle, (1 << pwm_bits) - 1);
  }

  // Verify channel is sane (prevent buffer overflow)
  if (channel >= num_channels)
  {
    if (debug_level >= 1)
    {
      usb.print("--> We only have ");
      usb.print(num_channels, DEC);
      usb.println(" channels; ignoring this request!");
    }
    return;
  }

  // Verify the channel is working
  if (channel_state[slave_no][channel] != channel_working)
  {
    if (debug_level >= 1)
    {
      usb.println("--> Channel is not working; ignoring this request!");
    }
    return;
  }

  // Verify we are not already striking the chime, and the settle has completed
  if (!striking[slave_no][channel] && verified[slave_no][channel])
  {
    // Check for overheat
    if (strikes_remaining[slave_no][channel] > 0)
    {
      // Strike the chime!
      striking[slave_no][channel] = true; // We are now striking
      verified[slave_no][channel] = false; // We need to verify it turns on
      strike_timer[slave_no][channel] = 0; // Start strike timer
      strikes_remaining[slave_no][channel]--; // We used an available strike
      // analogWrite(chime_pins[channel], duty_cycle); // Turn on PWM
      set_dc[slave_no][channel] = dc; // Store the PWM DC for channel validation
    }
    else
    {
      // Overheated
      // TODO: log this, report to master, etc.
      if (debug_level >= 1)
      {
        usb.println("--> Channel overheated! Skipped strike.");
      }
    }
  }
  else
  {
    // Already striking or settling
    // TODO: Log this, report to master, etc.
    if (debug_level >= 1)
    {
      usb.println("--> Already striking/settling channel! Skipped strike.");
    }
  }
}

bool is_shorted(const float& voltage)
{
  return voltage < (ps_setpoint * shorted_percent_ps_setpoint);
}

void handle_shorted(const uint8_t& slave_no, const uint8_t& channel)
{
  // Log shorted status
  channel_state[slave_no][channel] = channel_failed_short;
  any_shorted = true;

  // Disable power supply
  detachInterrupt(digitalPinToInterrupt(ps_enable_pin));
  pinMode(ps_enable_pin, OUTPUT);
  digitalWrite(ps_enable_pin, LOW);
  ps_state = disabled;

  // Diagnostics
  if (debug_level >= 1)
  {
    usb.print("Channel ");
    usb.print(channel + 1);
    usb.println(" is shorted!");
    usb.println("Power supply has been disabled for safety.");
  }
}

bool is_open(const float& voltage)
{
  return voltage > (ps_setpoint * open_percent_ps_setpoint);
}

bool verify_on(const uint8_t& slave_no, const uint8_t& channel)
{
  verified[slave_no][channel] = true;
  return true;
}

bool verify_off(const uint8_t& slave_no, const uint8_t& channel)
{
  verified[slave_no][channel] = true;
  return true;
}

void print_state(Print& out, const channel_state_t& state)
{
  switch (state)
  {
  case channel_working:
    out.print("Working Normally");
    break;

  case channel_disconnected:
    out.print("Disconnected / Solenoid Not Found");
    break;

  case channel_failed_short:
    out.print("Short-Circuited");
    break;

  case channel_failed_open:
    out.print("Failed Open-Circuit");
    break;

  default:
    out.print("Unknown / Invalid Channel Status");
    break;
  }
}

void print_state(Print& out, ps_state_t state)
{
  switch (state)
  {
    case stable:
      out.print("Stable");
      break;

    case stabilizing:
      out.print("Stabilizing");
      break;

    case disabled:
      out.print("Disabled");
      break;

    default:
      out.print("Unknown / Invalid");
      break;
  }
}
