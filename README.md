# Firefly simulator 3000
IOT version.

Communicates through a nRF24 module with my raspberry pi server.
The server is running the [nRF24_MQTT bridge](https://github.com/yetifrisstlama/nRF24_MQTT).

## Building
On Ubuntu 18
```bash
$ sudo apt-get install gcc-avr avr-libc avrdude
$ make
$ make flash
```
You can turn on UART prints in the Makefile to see what's going on.
```Makefile
CFLAGS     += -DDEBUG_LEVEL=2
```
