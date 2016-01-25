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

Notes:

I am planning on using a voltage divider and capacitor to filter the feedback
voltage from the transistor high-side to verify things are working.

The plan is currently a 160k / 10k divider, with a 1uF capacitor in parallel
with the 10k resistor. This will let me read 0 - 20.4 volts, and should filter
the PWM out so I can be lazy with my firmware.

I need external pullup resistors on the I2C bus, as the internal Teensy ones
don't work all that well. For high speed, I probably want 1.5k or so.

MIDI uses an opto-isolator. I should pick up some 6N138 opto-isolators.

TODO

## License

Content in each folder may be licensed differently. Overall, I strive to use the
Unlicense, but some portions may use external libraries preventing that. Those
portions will be licensed as necessary. Data sheets and the like are provided
for reference only, and are subject to all licenses/copyright specified within.
