// -----------------------------------------------------------------------------
// pwm_test.ino
//
// A program to test various PWM functions to see how the chimes respond.
// -----------------------------------------------------------------------------

// -----------------------------------------------------------------------------
// Includes
// -----------------------------------------------------------------------------


// -----------------------------------------------------------------------------
// Constants
// -----------------------------------------------------------------------------

// Pins
const uint8_t pwm_pin(3);
const uint8_t led_pin(13);

// Timeouts
const uint16_t strike_time(1000); // milliseconds
const uint16_t blink_time(20000); // microseconds

// PWM config
const uint8_t pwm_bits(12);
const uint32_t pwm_freq(11718); // 11718 Hz is "ideal" for 12 bits on Teensy

// -----------------------------------------------------------------------------
// Declarations
// -----------------------------------------------------------------------------
void set_pwm_percent(const uint32_t& percent);


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
      usb.println(incomingByte);
      // pwm_sweep = false;
      set_pwm_percent((incomingByte - '0') * 10);
      break;

    case('/'):
      usb.println(incomingByte);
      // pwm_sweep = false;
      set_pwm_percent(100);
      break;

    // case('*'):
    //   usb.println(incomingByte);
    //   pwm_sweep = true;
    //   usb.println("Sweeping PWM through range!");
    //   break;

    case('+'):
      usb.println(incomingByte);
      usb.println("Striking chime!");
      strike = true;
      strike_timer = 0;
      analogWrite(pwm_pin, duty_cycle);
      break;

    default:
      // Echo!
      usb.print(incomingByte);
      if (incomingByte == '\r')
      {
        usb.print('\n');
      }
      break;
    }
  }
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