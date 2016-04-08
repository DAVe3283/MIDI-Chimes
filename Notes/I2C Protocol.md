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

| Offset | Size |   Type   |    Meaning     |
|--------|------|----------|----------------|
|      0 |    1 | uint8_t  | Channel        |
|      1 |    2 | uint16_t | PWM Duty Cycle |

### Channel

The channel (local to the current slave) is which activated/struck.

### PWM Duty Cycle

This is a 12-bit value (sent as 16-bits). Any value greater than 12 bits is
treated as 100% by the Teensy libraries.

I believe this is sent as little-endian, but it doesn't matter since both master
and slave use the same processor.

# Read (Master request from Slave)

When the master requests a read from the slave, the slave writes the following
packet back to the master.

## Status Packet

| Offset | Size |   Type   |        Meaning        |
| -----: | ---: | -------- | --------------------- |
|      0 |    1 | uint8_t  | Message Version       |
|      1 |    1 | uint8_t  | PWM Bits              |
|      2 |    1 | uint8_t  | Physical Channels (n) |
|      3 |    1 | uint8_t  | Connected Channels    |
|      4 |    2 | uint16_t | Power Supply Voltage  |
|      6 |  n*1 | uint8_t  | Channel State         |
|    6+n |  n*1 | uint8_t  | Strikes Remaining     |
|   6+2n |  n*2 | uint16_t | Channel Voltage       |

### Message Version

Currently set to 0, but this allows me to change things in the future in a
backwards-compatible way.

### PWM Bits

How many bits resolution the PWM is using on this slave. Currently 12, but could
change. The master should ensure all PWM duty cycle values are in this range.

### Physical Channels

How many physical channels this board has. Currently 10.

The message is scaled to the number of channels used.

### Connected Channels

How many of the physical channels appear to be connected to chime coils.

This is there because I have 25 chimes, so the last slave board will only have
5 connected channels.

### Power Supply Voltage

The slave board's best guess at power supply voltage setpoint, measured during
startup.

### Channel State

This is an enum value that indicates the current state of each channel.
Currently:

| Value |              Meaning               |
| ----- | ---------------------------------- |
|     0 | Working normally                   |
|     1 | Disconnected (no coil detected)    |
|     2 | Transistor failed to short circuit |
|     3 | Transistor failed to open circuit  |

The channel disconnected status is so the master can verify the expected number
(and location) of coils are connected. If something isn't right, it should
display a message to the user, and ask them to verify the connections.

If a channel is failed to short circuit, the main power supply should be turned
off to prevent damage to the coil attached to that channel. The slave will
attempt to do this on its own, but the master should turn off the main supply
itself, just to be safe. The master should display a message to the user, so
they know what happened.

If a channel is failed to open circuit, it is safe to keep using the channel and
to leave the master supply on, but that channel may not work. The master should
display a message to the user so they know what happened.

### Strikes Remaining

A counter of how many more times we can strike this chime before the coil is
considered overheated. If this value is 0, the channel is overheated and will
ignore strike commands to protect the coil.

As time passes (and the coil cools), this value will increase, allowing the
channel to be used again.

This is purely informational, as the slave manages the thermals. However, it
might be fun to have a live display of the strikes remaining values.

### Channel Voltage

This is the last measured voltage of the given channel with the transistor off
(not striking the chime). It is measured once at startup, and again every time
a chime is finished striking.

This is purely informational, but might be fun to have a live display on the
master of all the channel voltages.

# Startup Process

I need to come up with a startup process to auto-assign addresses to the slave
boards, and verify everything is working (let the slaves verify PS voltage,
etc.).

## Fun Extras

It would be interesting to test and see if the R_BE (Base-Emitter resistance)
across the transistors is fairly consistent. From the data sheet, it looks to be
0.45-0.56 ohms depending on current flow. My testing shows it vary from 120 ohms
at 5mA then dropping quickly under 1 ohm by 1 A, and down to ~0.45 ohms at ~2.5
amps.

Perhaps instead, I could connect some various resistive (or inductive) loads and
just make a calibration curve of feedback voltage (at 100% PWM) vs. current, if
the transistors are consistent enough. Might be fun to have each coil's current
consumption (and resistance, since we have the power supply voltage) show up in
a diagnostics screen on the master.
