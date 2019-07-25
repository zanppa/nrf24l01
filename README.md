# nrf24l01
Raspberry PI to [TI Tiva](http://www.ti.com/tool/EK-TM4C123GXL) communication with (nRF24L01)[http://www.nordicsemi.com/Products/Low-power-short-range-wireless/nRF24-series] radio modules.

Uses dynamic payload and auto ACK payloads for two directional communication,
while one side (Raspberry PI in this case) is always in transmit mode and the 
other (Tiva) is in receive mode. This makes communication very simple.
