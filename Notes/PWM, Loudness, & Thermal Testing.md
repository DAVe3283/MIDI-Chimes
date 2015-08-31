# PWM Setting Tests

The PIC16F1503 has limited PWM frequencies when targeting 9-10 bit resolution.

| Frequency (Hz) | Resolution | CPU Clock | Transistor Temp |    Coil Whine?     |
| -------------: | ---------: | --------: | :-------------- | :----------------- |
|          1,953 |     10-bit |     8 MHz | <Untested>      | <Untested>         |
|          4,883 |     10-bit |    20 MHz | Excellent       | Yes, within ~1'    |
|          7,813 |     10-bit |     8 MHz | Excellent       | Barely, within ~6" |
|          9,766 |      9-bit |    20 MHz | Good            | Barely, within ~4" |
|         15,625 |      9-bit |     8 MHz | <Untested>      | <Untested>         |
|         19,531 |     10-bit |    20 MHz | Needs heatsink  | None               |

All _tested_ settings functioned just fine.

## Summary

It looks like 7,813 Hz is about ideal. Transistor runs really cold (probably
doesn't need a heatsink at all!); coil whine should be unnoticeable once covers
are in place, even standing close.  If coil whine is an issue, moving to
9,766 Hz would work, but heatsinks might be a good idea at that frequency.

A strike time of 85 to 100 ms is ideal for sound. The longer the strike, the
harder it is to keep things cool. 90 is a good compromise.

A settling time of 150 ms seems to sufficient, though going 160 ms with a 90 ms
strike time leads to a nice clean 4 strikes per second. However, this rate
_WILL_ require overall thermal management over a rolling 60-second (or so)
window to protect the coil.


# Thermals

During testing, it quickly became apparent that the part most likely to have
thermal issues is the coil itself. Repeated striking of the chime caused the
coils to quickly heat up.

To manage this, I attempted to optimize strike time. Details are in the sections
below.

Also of concern is settling time (how long the striker takes to return to its
rest position and stop bouncing).

Testing indicates that 350 ms is the upper limit of a reasonable value, and 150
ms is the lower limit. Below 150 ms, the striker doesn't fully settle against
the stop, and begins to affect the strike volume and timing.

## Overall Duty Cycle

The coils do take some time to actually heat up (> 60 seconds). Overall, I
should enfoce a limit on the total on-time (or even number of strikes) in a
rolling 60-second window.

## Thermal Throttling

To avoid interfering with sections of music that rapidly strike the same note,
a rolling 60-second window with a 60 strike limit seems prudent. Every second,
decrement the strike counter, and increment it with every strike. If it ever
reaches 60 (or whatever limit is reasonable), deny the strike, and flag a
thermal overload for the next I2C report. This will allow the Teensy to display
a thermal warning, and which chime is overheated. Perhaps maintain the flag
until the counter reaches all the way to 0? Or until cleared by the Teensy?


# Strike Time Tests

| Strike Time (ms) | Minimum PWM DC | Actual On Time (ms) |
| ---------------: | -------------: | ------------------: |
|               45 |            70% |                31.5 |
|               55 |            65% |                35.8 |
|               65 |            60% |                39.0 |
|               85 |            55% |                46.8 |
|              120 |            50% |                60.0 |

The faster the strike time, the harder it is to control volume. See the volume
tests below.

## Duty Cycle

Given a conservative 350 ms settling time, maximum duty cycle (assuming
back-to-back striking of the chime) can be calculated.

| Strike Time (ms) | Max Duty Cycle (%) |
| ---------------: | -----------------: |
|               45 |               11.4 |
|               50 |               12.5 |
|               55 |               13.6 |
|               60 |               14.6 |
|               65 |               15.7 |
|               70 |               16.7 |
|               75 |               17.6 |
|               80 |               18.6 |
|               85 |               19.5 |
|               90 |               20.5 |
|              100 |               22.2 |
|              125 |               26.3 |


# Chime Volume

I decided to measure chime volume for a given strike time and PWM DC %.
These are taken at about 1 meter with my Nexus 5. Or, in other words, are only
slightly better than made up numbers.

It turns out that above ~70 dB, the Nexus highly compresses the value. Most of
the measurements indicate that 90% and 100% PWM DC are the almost the same
volume, but that is not the case by far. So take these tables with a grain of
salt.

If a PWM value isn't listed, that is because the striker doesn't make contact
with the chime for a strike time that small combined with a PWM DC that small.

## 65 ms Strike Time

| PWM DC |  SPL  |
| -----: | ----: |
|    60% | 57 dB |
|    65% | 62 dB |
|    70% | 66 dB |
|    75% | 68 dB |
|    80% | 71 dB |
|    85% | 74 dB |
|    90% | 75 dB |
|    95% | 76 dB |
|   100% | 78 dB |

## 85 ms Strike Time

| PWM DC |  SPL  |
| -----: | ----: |
|    55% | 55 dB |
|    60% | 58 dB |
|    65% | 62 dB |
|    70% | 66 dB |
|    75% | 67 dB |
|    80% | 70 dB |
|    85% | 73 dB |
|    90% | 74 dB |
|    95% | 76 dB |
|   100% | 76 dB |

## 100 ms Strike Time

| PWM DC |  SPL  |
| -----: | ----: |
|    55% | 52 dB |
|    60% | 60 dB |
|    65% | 63 dB |
|    70% | 67 dB |
|    75% | 70 dB |
|    80% | 72 dB |
|    85% | 73 dB |
|    90% | 76 dB |
|    95% | 77 dB |
|   100% | 78 dB |

## Result Summary

The data is not as conclusive as I would like. I need a real SPL meter, with
peak hold.

Going by ear, it seems that 85-100 ms strike times yield identical outputs, so
that is a good range to stick in. Hence, why I picked a 90 ms strike time above.
