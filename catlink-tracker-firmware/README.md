# CatLink Tracker
This is a device that transmits GNSS position data to a receiver using an SX1268 LoRA module. It is optimized for low-power consumption.

## Pins
| IO Pin | MCU Function      | Module Function                |
|--------|-------------------|--------------------------------|
| A0     | ADC1 C5           | Battery VIN                    |
| A2     | Input EXTI2       | DIO1                           |
| A5     | SPI1 SCK          | LoRA SCK                       |
| A6     | SPI1 MISO         | LoRA MISO                      |
| A7     | SPI1 MOSI         | LoRA MOSI                      |
| A9     | Input EXTI9_5     | DIO3                           |
| A11    | Output            | GNSS ON/OFF                    |
| A12    | Input             | GNSS 1PPS                      |
| B0     | SPI1 NSS          | LoRA NSS                       |
| B4     | Output open-drain | Battery voltage divider switch |
| B6     | USART1 TX         | GNSS RX                        |
| B7     | USART1 RX         | GNSS TX                        |
