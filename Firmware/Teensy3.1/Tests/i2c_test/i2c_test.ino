// -----------------------------------------------------------------------------
// i2c_test.ino
//
// A program to test various I2C bus things.
// Currently, a dumb slave that always returns 0xD.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
#include <Wire.h>


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I2C config
const uint8_t i2c_address(32);

// Pins
const uint8_t pwm_pin(3);
const uint8_t led_pin(13);

// Timeouts
uint16_t strike_time(90); // milliseconds
uint16_t settle_time(160); // milliseconds
const uint16_t blink_time(20000); // microseconds

// PWM Config
const uint8_t pwm_bits(10);
const uint32_t pwm_freq(7813);


// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
void i2c_receive(int numBytes);
void i2c_requested();
void set_pwm_percent(const uint32_t& percent);
void print_pwm_setup();

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// Timers
elapsedMillis strike_timer;
elapsedMicros message_blink_timer;

// Serial IO
usb_serial_class  usb = usb_serial_class();
// HardwareSerial    gsx = HardwareSerial();
// HardwareSerial2   obd = HardwareSerial2();
// HardwareSerial3   gps = HardwareSerial3();

// PWM
int duty_cycle(1 << (pwm_bits - 1)); // Default to 50%
// bool pwm_sweep(false);
// const unsigned long adjustInterval(1); // Interval (ms)
// elapsedMillis sweepTimer;

// Strike chime variables
bool strike(false);


// -----------------------------------------------------------------------------
// Function Definitions
// -----------------------------------------------------------------------------

// Initial setup routine
void setup()
{
  usb.begin(9600);
  // gsx.begin(9600);
  // obd.begin(9600);
  // gps.begin(4800);

  // Configure I2C
  Wire.begin(i2c_address);
  Wire.onReceive(i2c_receive);
  Wire.onRequest(i2c_requested);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Configure PWM
  pinMode(pwm_pin, OUTPUT);
  analogWriteResolution(pwm_bits);
  analogWriteFrequency(pwm_pin, pwm_freq);
  analogWrite(pwm_pin, 0); // Off
}

// Main program loop
void loop()
{
  // Stop strike after timeout
  if (strike && (strike_timer >= strike_time))
  {
    strike = false;
    strike_timer = 0; // Begin settle timer
    analogWrite(pwm_pin, 0);
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
    case('0'):
    case('1'):
    case('2'):
    case('3'):
    case('4'):
    case('5'):
    case('6'):
    case('7'):
    case('8'):
    case('9'):
      // pwm_sweep = false;
      set_pwm_percent(((incomingByte - '0') * 5) + 50); // Adjust 50% - 100% in 5% steps
      break;

    case('/'):
      // pwm_sweep = false;
      set_pwm_percent(100);
      break;

    // case('*'):
    //   usb.println(incomingByte);
    //   pwm_sweep = true;
    //   usb.println("Sweeping PWM through range!");
    //   break;

    case('\r'):
      if (!strike && (strike_timer >= settle_time))
      {
        usb.println("Striking chime!");
        strike = true;
        strike_timer = 0;
        analogWrite(pwm_pin, duty_cycle);
      }
      else
      {
        usb.println("Already striking chime! Skipped.");
      }
      break;

    case('.'):
    case('?'):
      print_pwm_setup();
      break;

    case('+'):
      // strike_time += 5; // Increase by 5ms
      settle_time += 5; // Increase by 5ms
      print_pwm_setup();
      break;

    case('-'):
      // strike_time -= 5; // Decrease by 5ms
      settle_time -= 5; // Decrease by 5ms
      print_pwm_setup();
      break;

    default:
      // Echo unknown commands (sanity check the serial link)
      usb.print(incomingByte);
      break;
    }
  }
}

void i2c_receive(int numBytes)
{
  usb.print("Got ");
  usb.print(numBytes);
  usb.print(" byte(s):");
  for (int i(0); i < numBytes; ++i)
  {
    uint8_t data = Wire.read();
    usb.print(" 0x");
    usb.print(data, HEX);
  }
  usb.println();
  message_blink_timer = 0;
}

void i2c_requested()
{
  Wire.write(0x8);
  Wire.write(0xD);
  message_blink_timer = 0;
}

void set_pwm_percent(const uint32_t& percent)
{
  duty_cycle = static_cast<int>(percent * (1 << pwm_bits) / 100);
  usb.print("Set PWM Duty Cycle to ");
  usb.print(percent);
  usb.print("% (");
  usb.print(duty_cycle);
  usb.print("/");
  usb.print(1 << pwm_bits);
  usb.println(")");
}

void print_pwm_setup()
{
  float dc = static_cast<float>(duty_cycle) / static_cast<float>(1 << pwm_bits) * 100.0;
  usb.println("Current PWM Settings:");
  usb.print("  Frequency: ");
  usb.print(pwm_freq);
  usb.println(" Hz");
  usb.print("  Resolution: ");
  usb.print(pwm_bits);
  usb.println(" bits");
  usb.print("  Duty Cycle: ");
  usb.print(dc);
  usb.print("% (");
  usb.print(duty_cycle);
  usb.print("/");
  usb.print(1 << pwm_bits);
  usb.println(")");
  usb.print("  Strike Time: ");
  usb.print(strike_time);
  usb.println(" ms");
  usb.print("  Settle Time: ");
  usb.print(settle_time);
  usb.println(" ms");
}
