# Data Connector Pinout

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

# Transistor Feedback Voltage

I am using a 160k / 10k resistor divider, with a 1 uF capacitor in parallel with
the 10k resistor. Using the Teensy3's internal 1.2V precision reference, I can
read a transistor voltage of 0 to 20.4 volts. The capacitor filters out the PWM
frequency, letting me be extremely lazy with my firmware when checking how
things are working.

# Master

## I2C Bus

I need external pullup resistors on the I2C bus, as the internal Teensy ones
don't work all that well. For high speed, I probably want 1.5k or so.

## Legacy MIDI Ports

I am going to support the legacy DIN MIDI jacks. MIDI uses a current loop and
opto-isolator (optocoupler) to avoid ground loops. I will be using the 6N138.

I think I can use the MAB5SH part in Eagle as the footprint for the MIDI conenctor.
    Yeah, I think we're good. But just in case: http://www.newark.com/hirschmann/mab-5-sh/din-audio-video-connector-socket/dp/61T4646

I need to use a protection diode on the MIDI In port to protect the optocoupler
from exceeding its breakdown voltage (5V max). I could use the diode I use to
absorb flyback from the coils. Massive overkill, but simplifies the BoM. A small
surface mount part might be easier to lay out, though.

### Simple Option

I need some sort of output transistor for the MIDI out/through circuit.
Worst case, a shorted cable could cause a load of up to 23 mA (5 V / 220 ohms)
when pulling MIDI_OUT to ground, but the Teensy can only handle 9 mA per pin.

Plus, a faulty/malicious device could actually put more than 5V on Pin 5. A
transistor with sufficient ratings solves all this.

#### Overkill option

Because this is a chance for me to have too much fun with design and learn some
things, I could isolate the MIDI out as well. I would need an opto-isolator, and
an isolated power supply to provide the 5V for the current loop and to run the
optocoupler.

I will order a REE-0505S from Mouser and see if it works well enough. If not, I
will try a MEV1S0505SC, but this would need a footprint designed for Eagle.
(Actually, I need to verify the generic DC-DC footprint works on the REE...)

If that all works, I could route out an isolation gap in the PCB, and do all the
high-voltage design. Of which I currently know nearly nothing. But once I learn,
I would have a MIDI input/output stage with > 1000V isolation! heh

## Misc

http://www.mouser.com/ProductDetail/Molex/15-91-2140/?qs=sGAEpiMZZMttdt6ek4ZYDGcRuHgvKenX
    This is for the pads on the back of the Teensy.
