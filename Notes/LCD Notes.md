Notes about the LCD screen.

# Connection / Pins

The display pins are numbered like an Arduino UNO.

|   Circuit    | Teensy | Display |
| ------------ | -----: | ------: |
| LCD Reset    |      7 |   RESET |
| Touch CS     |      8 |       8 |
| LCD DC       |      9 |       9 |
| LCD CS       |     10 |      10 |
| SPI MOSI     |     11 |      11 |
| SPI MISO     |     12 |      12 |
| SPI SCK      |     14 |      13 |
| MicorSD CS   |     15 |       4 |

CS = chip select
DC = data/command select, appears non standard, used for display
