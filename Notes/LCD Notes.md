Notes about the LCD screen.

# Connection / Pins

The display pins are numbered like an Arduino UNO.

| Circuit       | Teensy | Arduino |
| ------------  | -----: | ------: |
| LCD Reset     | 7      | RESET   |
| SPI Touch CS  | 8      | 8       |
| I2C Touch SCL | 19     | A5/SCL  |
| I2C Touch SDA | 18     | A4/SDA  |
| LCD DC        | 9      | 9       |
| LCD CS        | 10     | 10      |
| SPI MOSI      | 11     | 11      |
| SPI MISO      | 12     | 12      |
| SPI SCK       | 14     | 13      |
| MicorSD CS    | 15     | 4       |
| LED Backlight | 3      | 3       |

CS = chip select
DC = data/command select, appears non standard, used for display
I2C: Use the pins on the same side as the SPI bus of the display board. Arduino exposes these pins in two places, and only one pair works.

# Backlight

The backlight intensity can be controlled by PWM-ing the backlight pin to switch
the LEDs on and off.

I tried a 16-bit PWM @ 549 Hz and could see no noticable flicker. I think that
16 bits is serious overkill. Even 8 bits might be overkill.

| PWM % DC |     Percieved Brightness    |
|----------|-----------------------------|
| 0% (Off) | Can't see anything          |
| 1/65535  | Can't see anything          |
| 1%       | Very dim, almost unusable   |
| 5%       | Usable in soft indoor light |
| 10%      | About 25% full brightness   |
| 30%      | About 50%                   |
| 50%      | About 75%                   |
| 90%      | About 98%                   |

If the backlight is turned completely off, you can't see _anything_, even by
shining a flashlight on the screen. For all intents and purposes, the screen is
off. We probably want to avoid this.

For a standby/eco mode (power supply off), something in the 1% - 5% range is
probably good.

For an "unattended" mode (chimes active but no display interaction for some
time), something in the 30-50% rand is probably good.

It would be cool to have it smoothly ramp the brightness up and down as the
chimes transition between various power saving modes. Make the transition quick
enough to not get in the way of user operation, but slow enough to look good.
That probably means in the 100 - 500 ms range to get brighter on interaction.
We could fade out much more slowly (1-5 seconds?) though.
