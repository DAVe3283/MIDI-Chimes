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

CS = chip select
DC = data/command select, appears non standard, used for display
I2C: Use the pins on the same side as the SPI bus of the display board. Arduino exposes these pins in two places, and only one pair works.