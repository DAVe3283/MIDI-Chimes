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

The master and slaves communicate over I2C. I plan to just hardcode their
addresses, because I am in a time crunch. Later, I will do auto-assignment of
address on bootup.

Currently, the midi_test and i2c_test sketches are doing about what I expect.

I need to come up with a message set to tell the slave devices when to strike
the chimes, and how hard.

The aftermarket I2C library is pretty slick. I need to use that. Plus, I can set
the bus speed to something obnoxious.

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

Notes:

I am planning on using a voltage divider and capacitor to filter the feedback
voltage from the transistor high-side to verify things are working.

The plan is currently a 160k / 10k divider, with a 1uF capacitor in parallel
with the 10k resistor. This will let me read 0 - 20.4 volts, and should filter
the PWM out so I can be lazy with my firmware.

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
