#!/usr/bin/python
# -*- coding: utf-8 -*-
#
# Example program to communicate with TI Tiva Launchpad
# with Nordic nRF24L01 radio module, using ACK payloads.
#
# This is the transmitter running on Raspberry Pi Zero W.
# This program sends periodically a PINGxx message, where
# xx is a hex going from 0 to FF.
#
# Tiva replies with PONGyy, where yy is the previous received
# number. Both sent and received messages are printed on the
# screen as well as possible transmit errors.
#
# 2018, Lauri Peltonen <first and last with dot inbetween@gmail.com>
#
# Original code from pynrf24 library
# https://github.com/jpbarraca/pynrf24
# João Paulo Barraca <jpbarraca@gmail.com>
#

# Raspberry pi zero W pin configuration
# 3V3: pin 1
# GND: pin 6
# MOSI: BCM 10, pin 19, SPI0_MOSI
# MISO: BCM 9, pin 21, SPI0_MISO
# CLK: BCM 11, pin 23, SPI0_SCLK
# CSN: BCM 8, pin 24, SPI0_CE0
# CE: BCM 25, pin 22
# IRQ: BCM 16, not in use

from __future__ import print_function

from nrf24 import NRF24
import time


# Transmit pipes, LSB byte first!
# Tiva uses following ones, they are copied here
# const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };
pipes = [[0xE1, 0xf0, 0xf0, 0xf0, 0xf0], [0xD2, 0xf0, 0xf0, 0xf0, 0xF0]]

radio = NRF24()
radio.begin(0, 0, 25, 16) # use SPI0 and CE0, set CE and IRQ pins
radio.setRetries(15,15)

# Dynamic payload must be enabled for ACK payload
radio.enableDynamicPayloads()

# ACK payload must be enabled on transmitter too to receive them
radio.enableAckPayload()

radio.setPayloadSize(8)
radio.setChannel(76)

# Use default 1 MBPS rate instead of slower one
#radio.setDataRate(NRF24.BR_250KBPS)

# Set maximum transmit power
radio.setPALevel(NRF24.PA_MAX)

radio.openWritingPipe(pipes[1])
radio.openReadingPipe(1, pipes[0])

radio.startListening()
radio.stopListening()

radio.printDetails()

number = 0

while True:
    # Format message and send it over air
    sendStr = "PING" + ("%0.2X" % number)
    if radio.write(sendStr):
        print("< " + sendStr + " OK")  # Transmit was ok
    else:
        print("TX timeout")	# Transmit failed, no ACK received


    # Check if data was received (ACK payload)
    while radio.available() or radio.isAckPayloadAvailable(): # no parameters = all pipes, no irq
        recv_buffer = []
        radio.read(recv_buffer)

        # Print received data as ascii
        print("> ", end="")
        for value in recv_buffer:
            print(chr(value), end="")
        print()

    # Wait a bit before next message
    time.sleep(2)

    number += 1
    if number > 255:
        number = 0
