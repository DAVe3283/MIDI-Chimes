// -----------------------------------------------------------------------------
// midi_test.ino
//
// A program to test USB MIDI functionality of the Teensy.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------
//#include <Wire.h>
#include "i2c_t3.h"


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// I2C config
const uint8_t i2c_slave_address(32);

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
void OnNoteOn(byte channel, byte note, byte velocity);
void OnNoteOff(byte channel, byte note, byte velocity);
void OnVelocityChange(byte channel, byte note, byte velocity);
void OnControlChange(byte channel, byte control, byte value);
void OnProgramChange(byte channel, byte program);
void OnAfterTouch(byte channel, byte pressure);
void OnPitchChange(byte channel, int pitch);

// void set_pwm_percent(const uint32_t& percent);
// void print_pwm_setup();

// -----------------------------------------------------------------------------
// Globals
// -----------------------------------------------------------------------------

// Timers
elapsedMillis strike_timer;
elapsedMicros message_blink_timer;

// Serial IO
// usb_serial_class  usb = usb_serial_class();
HardwareSerial    ser = HardwareSerial();

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
  // Configure I2C
  // Wire.begin();
  Wire.begin(I2C_MASTER, 0, I2C_PINS_18_19, I2C_PULLUP_EXT, I2C_RATE_100);

  // Configure LED
  pinMode(led_pin, OUTPUT);

  // Test
  digitalWrite(led_pin, HIGH);
  Wire.beginTransmission(i2c_slave_address);
  Wire.write(0x00);
  Wire.write(0x10);
  Wire.write(0xAA);
  Wire.endTransmission();
  digitalWrite(led_pin, LOW);

  // Configure serial
  ser.begin(1500000);

  // Configure USB MIDI
  usbMIDI.setHandleNoteOn(OnNoteOn);
  usbMIDI.setHandleNoteOff(OnNoteOff);

  // Configure PWM
  pinMode(pwm_pin, OUTPUT);
  analogWriteResolution(pwm_bits);
  analogWriteFrequency(pwm_pin, pwm_freq);
  analogWrite(pwm_pin, 0); // Off
}

// Main program loop
void loop()
{
  usbMIDI.read();

  // if (usb.available() > 0)
  // {
  //   const char incomingByte(static_cast<char>(usb.read()));
  //   message_blink_timer = 0;

  //   switch (incomingByte)
  //   {
  //   case('0'):
  //   case('1'):
  //   case('2'):
  //   case('3'):
  //   case('4'):
  //   case('5'):
  //   case('6'):
  //   case('7'):
  //   case('8'):
  //   case('9'):
  //     // pwm_sweep = false;
  //     set_pwm_percent(((incomingByte - '0') * 5) + 50); // Adjust 50% - 100% in 5% steps
  //     break;

  //   case('/'):
  //     // pwm_sweep = false;
  //     set_pwm_percent(100);
  //     break;

  //   // case('*'):
  //   //   usb.println(incomingByte);
  //   //   pwm_sweep = true;
  //   //   usb.println("Sweeping PWM through range!");
  //   //   break;

  //   case('\r'):
  //     if (!strike && (strike_timer >= settle_time))
  //     {
  //       usb.println("Striking chime!");
  //       strike = true;
  //       strike_timer = 0;
  //       analogWrite(pwm_pin, duty_cycle);
  //     }
  //     else
  //     {
  //       usb.println("Already striking chime! Skipped.");
  //     }
  //     break;

  //   case('.'):
  //   case('?'):
  //     print_pwm_setup();
  //     break;

  //   case('+'):
  //     // strike_time += 5; // Increase by 5ms
  //     settle_time += 5; // Increase by 5ms
  //     print_pwm_setup();
  //     break;

  //   case('-'):
  //     // strike_time -= 5; // Decrease by 5ms
  //     settle_time -= 5; // Decrease by 5ms
  //     print_pwm_setup();
  //     break;

  //   default:
  //     // Echo unknown commands (sanity check the serial link)
  //     usb.print(incomingByte);
  //     break;
  //   }
  // }
}

void OnNoteOn(byte channel, byte note, byte velocity)
{
  digitalWrite(led_pin, HIGH);
  // ser.print("Note On! Channel ");
  // ser.print(channel);
  // ser.print(" Note: ");
  // ser.print(note);
  // ser.print(" Velocity: ");
  // ser.print(velocity);
  // ser.println();

  // Send I2C message
  Wire.beginTransmission(i2c_slave_address);
  Wire.write(note);
  Wire.endTransmission();
}

void OnNoteOff(byte channel, byte note, byte velocity)
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
  Wire.requestFrom(i2c_slave_address, 2);
  uint8_t data;
  while(Wire.available()) {
    data = Wire.receive();
    ser.print("d=0x");
    ser.println(data, HEX);
  }
}

// void set_pwm_percent(const uint32_t& percent)
// {
//   duty_cycle = static_cast<int>(percent * (1 << pwm_bits) / 100);
//   usb.print("Set PWM Duty Cycle to ");
//   usb.print(percent);
//   usb.print("% (");
//   usb.print(duty_cycle);
//   usb.print("/");
//   usb.print(1 << pwm_bits);
//   usb.println(")");
// }

// void print_pwm_setup()
// {
//   float dc = static_cast<float>(duty_cycle) / static_cast<float>(1 << pwm_bits) * 100.0;
//   usb.println("Current PWM Settings:");
//   usb.print("  Frequency: ");
//   usb.print(pwm_freq);
//   usb.println(" Hz");
//   usb.print("  Resolution: ");
//   usb.print(pwm_bits);
//   usb.println(" bits");
//   usb.print("  Duty Cycle: ");
//   usb.print(dc);
//   usb.print("% (");
//   usb.print(duty_cycle);
//   usb.print("/");
//   usb.print(1 << pwm_bits);
//   usb.println(")");
//   usb.print("  Strike Time: ");
//   usb.print(strike_time);
//   usb.println(" ms");
//   usb.print("  Settle Time: ");
//   usb.print(settle_time);
//   usb.println(" ms");
// }
