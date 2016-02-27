# MIDI-Chimes
Software &amp; notes from retrofitting some old chimes to MIDI control

Eventually, this will contain schematics, PCB layouts, data sheets, and source
code to make everything work. I will add things as I get to them.

## Project Details

TODO: writeup project details

## Firmware

The project uses several Teensy 3.2 microcontrollers. There is a master, which
handles MIDI and the user interface. The rest are slaves, which run 10 chimes
each.

The master and slaves communicate over I2C. The master talks to a touch screen
LCD over SPI.

### Libraries

I am using the following libraries:

* [i2c_t3][] - A superior I2C library that runs quite fast on the Teensy3.
* [ILI9341_t3][] - An optimized ILI9343 SPI LCD driver library for the Teensy3.
* [ILI9341_fonts][] - Additional fonts for ILI9341_t3.
* [Adafruit_STMPE610][] - STMPE610 SPI touch screen controller library.
* [Adafruit_FT6206][] - FT6206 I2C touch screen controller library.
* [SdFat][] - SD Card + FAT file system library.

The Adafruit_STMPE610 library needs `#include <Wire.h>` changed to
`#include <i2c_t3.h>` or it conflicts with the i2c_t3 library.

#### SdFat

To work right, the SdFat library needs the following settings in SdFatConfig.h:

`#define SD_SPI_CONFIGURATION 1`

This uses the Arduino SPI.h library, which is really an optimized one for the
Teensy 3.1.

`#define ENABLE_SPI_TRANSACTION 1`

This enables SPI transactions, which makes using multiple devices on the SPI bus
much nicer.

I had some issues with SdFat-beta, but I might go back and make that work,
because it does some cool stuff.

## Schematics

See the "PCB & Schematic" directory.

### Data Connector Pinout

| Pin |  Description  |
| --- | ------------- |
|   1 | Ground        |
|   2 | +5V_SB        |
|   3 | I2C SCL       |
|   4 | I2C SDA       |
|   5 | Address Latch |
|   6 | PS_EN         |

+5V_SB is 5V standby power from the PSU.
PS_EN is the power supply enable line.
Address latch is used to auto-assign I2C addresses on startup (future use).

If I did this right, no magic smoke should escape if (when) someone plugs this
connector in backwards. Off-by-one... not so much.

### Transistor Feedback Voltage

I am using a 160k / 10k resistor divider, with a 1 uF capacitor in parallel with
the 10k resistor. Using the Teensy3's internal 1.2V precision reference, I can
read a transistor voltage of 0 to 20.4 volts. The capacitor filters out the PWM
frequency, letting me be extremely lazy with my firmware when checking how
things are working.

### Master

I need external pullup resistors on the I2C bus, as the internal Teensy ones
don't work all that well. For high speed, I probably want 1.5k or so.

MIDI uses an opto-isolator. I should pick up some 6N138 opto-isolators.

http://www.mouser.com/ProductDetail/Molex/15-91-2140/?qs=sGAEpiMZZMttdt6ek4ZYDGcRuHgvKenX
    This is for the pads on the back of the Teensy.

I think I can use the MAB5SH part in Eagle as the footprint for the MIDI conenctor.
    Yeah, I think we're good. But just in case: http://www.newark.com/hirschmann/mab-5-sh/din-audio-video-connector-socket/dp/61T4646

## License

Content in each folder may be licensed differently. Overall, I strive to use the
Unlicense, but some portions may use external libraries preventing that. Those
portions will be licensed as necessary. Data sheets and the like are provided
for reference only, and are subject to all licenses/copyright specified within.

[Adafruit_FT6206]: https://github.com/adafruit/Adafruit_FT6206_Library
[Adafruit_STMPE610]: https://github.com/adafruit/Adafruit_STMPE610
[i2c_t3]: https://github.com/nox771/i2c_t3
[ILI9341_fonts]: https://github.com/PaulStoffregen/ILI9341_fonts
[ILI9341_t3]: https://github.com/PaulStoffregen/ILI9341_t3
[SdFat]: https://github.com/greiman/SdFat
