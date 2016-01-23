# I2C Protocol

This documents the protocol I am using for the I2C bus.

# Write (Master to Slave)

While I could get clever and allow multiple channels to be turned on and off per
message, there is no need for this. The I2C bus is substantially faster than the
incoming MIDI messages (per the spec, at least), so I can be lazy and just
handle one channel at a time.

Even more lazy, since the strike time for the chime is predetermined, and there
are no dampers on this setup, I don't even have to support a "note off" message
at all! So I don't.

## Note On Message

| Offset | Size |    Meaning     |
| ------ | ---- | -------------- |
|      0 |    1 | Channel        |
|      1 |    2 | PWM Duty Cycle |

### Channel

The channel (local to the current slave) is which activated/struck.

### PWM Duty Cycle

This is a 12-bit value (sent as 16-bits). Any value greater than 12 bits is
treated as 100% by the Teensy libraries.

I believe this is sent as little-endian.

# Read (Master request from Slave)

This will contain status for all the channels on the slave, including overheat
and transistor failure.

TODO: this
