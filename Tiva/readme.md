TI Tiva launchpad client to test nRF24L01 communication.

This is my own port of the [Arduino rf24 library](https://github.com/nRF24/RF24) for Tiva launchpad,
i.e. for TM4C123GH6PMI processor.

This client listens for transmissions and sends replies with ACK payload. 
The payload is formed by replacing 2nd character with ```O```, and is 
intended to convert ```PINGx``` to ```PONGx``` message, where ```x``` is 
the message number. This shows how to use the auto-ack messages of the radio 
module to have very simple method of two-way communication.

The main code also contains code to send the received messages to UART (UART0 which in the launchpad 
is connected to the debug/programming USB port), and also read lines from the same UART 
and send them as the auto-ack replies.

### Connections
The pinout used is according to following list.
```
PA2	  = NRF24L01 SPI CLK
PA3	  = NRF24L01 SPI FSS
PA4	  = NRF24L01 SPI RX
PA5	  = NRF24L01 SPI TX
PB3	  = NRF24L01 CE
PB4	  = NRF24L01 CSN
```
The library uses the hardware SPI (SSI) so the possible pins are limited.


### Compiling
The program is built using the [Energia IDE](https://energia.nu/), but since I dislike the built-in 
libraries and initializations and whatnot, I override the ```main``` and ```init``` functions and 
as a result small changes are needed in the Energia installation.

In the default main C/Cpp file, ```init``` and ```main``` functions need to be weakened by adding
```__attribute__((weak))```
before the function names, i.e. in 
```..\Energia15\packages\energia\hardware\tivac\1.0.3\cores\tivac\main.cpp```
change the ```init``` and ```main``` to following function definitions:
```
void __attribute__((weak)) _init(void)
int __attribute__((weak)) main(void)
```

This allows overriding the functions in own code and preventing all default "Energia" initializations from happening. This way the state of the processor is in known state (according to the datasheet) when our own main function starts.
