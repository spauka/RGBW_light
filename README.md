# STM32 WS2812 compatible RGBW Controller

This repo contains code for controlling WS2812 compatible RGBW leds using an STM32 microcontroller.

Note: Requires [libopencm3](https://github.com/libopencm3/libopencm3) to work.

![image](https://user-images.githubusercontent.com/635580/94989172-1fc31f00-05b6-11eb-9acc-d5bd990b383b.png)

SPI is used to allow the hardware to manage timing for us - although we do have to craft bytes to conform to the wierd format used by these LEDs.

USB serial is used to print status.

## TODO
 - [ ] Implement configuration via USB serial
 - [ ] Implement hardware DMA so that we don't have to stay in the SPI loop while updating the LEDs
