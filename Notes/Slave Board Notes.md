Notes about the slave board.

Voltage measured with transistor on 100%:

| Current (mA) | Voltage |  R_BE |
| -----------: | ------- | ----: |
|            5 |     0.6 | 120.0 |
|         1000 |     0.8 |   0.8 |
|         2000 |     0.9 |  0.45 |
| Actual Coils |     1.1 | ~0.44 |

So any voltage over 0.5V at boot means something is connected.
I should probably consider anything under 1.5V (and < 50% ps_setpoint) shorted at startup.

Or, since the chimes will draw ~2A, simply subtract 0.9V from the measured
voltage, and use that for the percentage calculations to verify operation?

Perhaps some testing with the real chimes is in order...

Measured values from my chimes.
Ideal values calced with
    ideal_voltage = ps_setpoint - ((ps_setpoint - transistor_Vce_drop) * set_dc[channel])
    ps_setpoint = 19.13
    transistor_Vce_drop = 1.1

| PWM DC | Measured | Ideal | Error |
| -----: | -------- | ----- | ----: |
|     0% |    19.13 | 19.13 |  0.0% |
|    10% |    17.76 | 17.33 | -2.3% |
|    20% |          | 15.52 |       |
|    30% |    14.07 | 13.72 | -1.8% |
|    40% |    12.18 | 11.92 | -1.4% |
|    50% |    10.31 | 10.12 | -1.0% |
|    60% |     8.38 |  8.31 | -0.4% |
|    70% |     6.51 |  6.51 |  0.0% |
|    80% |     4.61 |  4.71 |  0.5% |
|    90% |     2.73 |  2.90 |  0.9% |
|   100% |     1.10 |   1.1 |  0.0% |
