# CatLink Tracker
This is a device that transmits GNSS position data to a receiver using an SX1268 LoRA module. It is optimized for low-power consumption.

## Pins
| IO Pin | MCU Function      | Module Function                |
|--------|-------------------|--------------------------------|
| A0     | ADC1 C5           | Battery VIN                    |
| A2     | Output open-drain | Battery voltage divider switch |
| A4     | SPI1 NSS          | LoRA NSS                       |
| A5     | SPI1 SCK          | LoRA SCK                       |
| A6     | SPI1 MISO         | LoRA MISO                      |
| A7     | SPI1 MOSI         | LoRA MOSI                      |
| A15    | Input EXTI15_10   | DIO3                           |
| B3     | Output            | GNSS ON/OFF                    |
| B4     | Input EXTI4       | DIO1                           |
| B5     | Input             | GNSS 1PPS                      |
| B6     | USART1 TX         | GNSS RX                        |
| B7     | USART1 RX         | GNSS TX                        |
