I want the chimes to work as a doorbell as well as MIDI chimes. Why? Why not?

To make this happen, I bought a doorbell (GE 19247) for $14 or so.

# GE 19247

This sucker is surprisingly well designed. The PCB appears to be one layer
(though it has screen-printing on both sides), with the CPU inserted vertically
on a card. Impressively well laid out, IMO.

Furthermore, it appears the speaker output is isolated from ground. This is the
correct way to do things (I am impressed, especially at this price point), but
sadly, that means simply snooping a speaker pin won't work.

## EEPROM

There is a FM24C02C 256x8 bit EEPROM attached to the CPU. When the doorbell
button is pressed, there is some I/O traffic to the EEPROM. Here is one exchange
I captured.

| Address |  R/W  | Value |
|---------|-------|-------|
| 0x13    | Read  | 0x0A  |
| 0x01    | Read  | 0x26  |
| 0x02    | Read  | 0x87  |
| 0x03    | Read  | 0x90  |
| 0x14    | Read  | 0x0A  |
| 0x04    | Read  | 0x79  |
| 0x05    | Read  | 0xA5  |
| 0x06    | Read  | 0xD0  |
| 0x0A    | Read  | 0x01  |
| 0x0A    | Write | 0x01  |

I have no idea what it is doing with that data (or why it re-writes 0x0A from 1
to 1), but it doesn't matter for what we are doing.

## A trick!

Since we can't read the speaker output (floating ground), we need _some_ way to
tell the doorbell button was pressed. Well, we can watch SCL (or SDA) for a
high-to-low transition. That indicates the start of some I2C traffic, and
therefore, a button press!

## Measurements

The SCL and SDA lines appear to be pulled up to 2.7V, and the CPU itself runs
on 2.9V. 2.7V should be sufficiently high to count as logic high on the Teensy,
which runs at 3.3V.

The I2C bus is clocked at approximately 5.6 kHz, with occasional long pauses
mid-transaction. Given how slow it is, I doubt the wire running to the Teensy
will cause any bus corruption.

# TODO

I need to solder a wire to the SCL line, and hook it to the Teensy to verify I
can reliably detect button presses. I should also leave it running for a long
time period to count bus events, and make sure there is no random idle bus
traffic that would cause the doorbell to go off randomly.
