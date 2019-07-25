Raspberry PI zero W software to test nRF24L01 communication.

Uses the [pynrf24 library](https://github.com/jpbarraca/pynrf24).

This program sends a ```PINGx``` message, where ```x``` is the message number, 
periodically over the air, prints the transmission status (OK/failed) and also 
prints any auto-ack reply messages if they are received.

Example of output when used together with the TI Tiva program is below.
```
$ python transmit.py
STATUS          = 0x0e RX_DR=0 TX_DS=0 MAX_RT=0 RX_P_NO=7 TX_FULL=0
RX_ADDR_P0-1    = 0xd2f0f0f0f0 0xe1f0f0f0f0
RX_ADDR_P2-5    = 0xc3 0xc4 0xc5 0xc6
TX_ADDR         = 0xd2f0f0f0f0
RX_PW_P0-6      = 0x00 0x00 0x00 0x00 0x00 0x00
EN_AA           = 0x3f
EN_RXADDR       = 0x03
RF_CH           = 0x4c
RF_SETUP        = 0x06
SETUP_AW        = 0x03
OBSERVE_TX      = 0x00
CONFIG          = 0x0e
FIFO_STATUS     = 0x11
DYNPD           = 0x3f
FEATURE         = 0x06
Data Rate       = 1MBPS
Model           = nRF24l01+
CRC Length      = 16 bits
PA Power        = PA_MAX
< PING00 OK
< PING01 OK
> PONG00
< PING02 OK
> PONG01
< PING03 OK
> PONG02
< PING04 OK
> PONG03
< PING05 OK
> PONG04
< PING06 OK
> PONG05
< PING07 OK
> PONG06
< PING08 OK
> PONG07
```

As can be seen, there is no PONG reply for the first packet (PING00),
since when that is received, the PONG is written to the buffer.
When next packet arrives (PING01) the PONG00 message in the transmit
buffer is automatically sent back.


### Connections
Connections between the Raspberry PI and the nrf24L01 module are according to following list (I used Raspberry PI Zero W)
```
nrf24L01   RPI
=====================================
3V3        pin 1
GND        pin 6
MOSI       BCM 10, pin 19, SPI0_MOSI
MISO       BCM 9, pin 21, SPI0_MISO
CLK        BCM 11, pin 23, SPI0_SCLK
CSN        BCM 8, pin 24, SPI0_CE0
CE         BCM 25, pin 22
IRQ        BCM 16, not in use
```
