/**
 * Simple NRF24L01 to UART bridge driver
 *
 * For TI Tiva or Stellaris launchpads, TM4C123 or LM4F120 microcontrollers.
 *
 * Behaves as a receiver, receiving data from another NRF24L01 module
 * Sends all data received to UART, 115200 baud.
 * Sends ascii as is, with newline after each received packet
 *
 * Can also transmit data back in auto-ack packets, sends data received
 * on UART after newline \r or \n is received
 *
 * Green LED blinks whenever data is received through UART
 * Red LED blinks whenever data is received over air
 *
 * Tested to work with Energia IDE, after some modifications to
 * the default main C/Cpp file, init and main functions need to be weakened, add:
 * __attribute__((weak))
 * before the function, i.e. in 
 * ...\Energia15\packages\energia\hardware\tivac\1.0.3\cores\tivac\main.cpp
 * change init and main to following function definitions:
 * void __attribute__((weak)) _init(void)
 * int __attribute__((weak)) main(void)
 *
 * Copyright (C) 2016-2018 Lauri Peltonen <first and last with dot inbetween@gmail.com>
 */

#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_memmap.h"

#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/systick.h"

#include "driverlib/uart.h"

#include "common.h"
#include "nrf24l01.h"


// TI TIVA launchpad built-in RGB led pins
#define LED_RED				GPIO_PIN_1
#define LED_GREEN			GPIO_PIN_3
#define LED_BLUE			GPIO_PIN_2

#define MAX_MESSAGE			32		// Maximum message length to send or receive

// Radio pipe addresses, must match sender and receiver
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

// LED color variable
volatile uint8_t ledStatus = 0;

// Message to be sent over radio in an ACK packet
volatile uint8_t sendMessage[MAX_MESSAGE] = {0};
volatile uint8_t sendMessageLength = 0;
volatile uint8_t sendMessageReady = 0;

// Message received from radio
uint8_t receiveMessage[MAX_MESSAGE] = {0};
uint8_t receiveMessageLength = 0;

/**
 * Override for Energia IDEs default init function
 * that must be weakened so that his overrides it.
 */
void _init(void) {;}


/**
 * Handler for UART receive interrupt.
 *
 * Adds characters from UART to transmit buffer and
 * upon receiving \r or \n marks the buffer as ready.
 * Only fills the buffer if the ready flag is not set.
 */ 
void __attribute__ ((interrupt)) UARTIntHandler(void)
{
	unsigned long ulInts;
	long lChar;
	uint8_t ucChar;

	// Get and clear the current interrupt source(s)
	ulInts = UARTIntStatus(UART0_BASE, ~0);
	UARTIntClear(UART0_BASE, ulInts);

	// TX FIFO has space available
	if(ulInts & UART_INT_TX) { }

	// Receive interrupts
	if(ulInts & (UART_INT_RX | UART_INT_RT))
	{
		// Read the UART's characters into the buffer.
		while(UARTCharsAvail(UART0_BASE))
		{
			lChar = UARTCharGetNonBlocking(UART0_BASE);

			// If the character did not contain any error notifications, copy it to the output buffer.
			if(!(lChar & ~0xFF)) {
				if(sendMessageLength < MAX_MESSAGE) {
					ucChar = lChar & 0xFF;
					if(ucChar == '\r' || ucChar == '\n') {
						sendMessageReady = 1;
					} else if(sendMessageReady == 0) {				// Only accept if message was already sent
						sendMessage[sendMessageLength++] = ucChar;  // Add
            //UARTCharPutNonBlocking(UART0_BASE, ucChar);
            //ledStatus ^= LED_BLUE;
					} else {
						UARTCharPutNonBlocking(UART0_BASE, '#');	// # marks that message queue is full
					}
				}
				ledStatus ^= LED_GREEN;

			} else {
				// TODO: Handle uart errors here
			}
		}
	}
}


/**
 * Send buffer over UART0
 *
 * Returns 0 if uart is busy (FIFO not empty), otherwise 1
 * Note: Buffer may need to be < 16 chars (FIFO size), otherwise some characters may get lost...
 */
uint8_t UARTSend(const uint8_t *pui8Buffer, uint32_t ui32Count)
{
	if(UARTBusy(UART0_BASE)) return 0;
	while(ui32Count--) {
		UARTCharPut(UART0_BASE, *pui8Buffer++);
		//UARTCharPutNonBlocking(UART0_BASE, *pui8Buffer++);
	}
	return 1;
}

unsigned char hexmap[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    'A', 'B', 'C', 'D', 'E', 'F' };
//unsigned char space = ' ';
//unsigned char newline[] = { '\r', '\n' };

void UARTSendHex(const uint8_t val)
{
  unsigned char str[2] = {0};
  str[0] = hexmap[(val>>4) & 0xF];
  str[1] = hexmap[val & 0xF];
  UARTSend(str, 2);
}

/**
 * Main function
 *
 */
int main(void)
{
	int i;
	uint8_t more, len;

	// Set clock speed to 80 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL| SYSCTL_OSC_INT);

	// Led peripheral and pins
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);

	// TODO: Update serial code
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
							(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
							 UART_CONFIG_PAR_NONE));
	UARTIntRegister(UART0_BASE, UARTIntHandler);
	UARTIntEnable(UART0_BASE, UART_INT_RX | UART_INT_RT);	// Interrupt on receive full and receive byte

	// Enable radio
	rf24Setup();

	IntMasterEnable();

	for(i=0;i<0x18;i++) {
		UARTSendHex(rf24ReadRegister(i));
    UARTSend("\r\n",2);
	}

	// Initialize radio comm
	while(!UARTSend("RX\r\n", 4));

	rf24Init();

  // Set transmit power to full
  rf24SetPALevel(3);

	// Enable dynamic payloads and auto-ack
	rf24SetDynamicPayload(1, 0x3F);			// All pipes
	rf24UseAckPayload(1, 0x3F);				// All pipes send auto-ack

	rf24OpenReadingPipe(1, pipes[1]);
	rf24StartListening();

	rf24PowerUp();	// Keep the radio on standby

	// Main loop
	while(1)
	{
		// Blink LEDs if they are set
		GPIOPinWrite(GPIO_PORTF_BASE, LED_RED | LED_GREEN | LED_BLUE, ledStatus);

		// Message read from UART is pending, add it to auto-ack FIFO
		// TODO: Verify that maximum of 3 messages are pending at one time!
    // Commented out since we use the automatic PONG with raspberry pi test
    // Typically writing something here would fill up the auto-ack buffer
    // if the automatic pong is enabled at the same time
#if 0
		if(sendMessageReady != 0)
		{
			rf24WriteAckPayload(1, sendMessage, sendMessageLength);

			// Clear the flags to be ready for next message
			sendMessageLength = 0;
			sendMessageReady = 0;
		}
#endif

		// If bytes have been received, send them through UART
		if(rf24Received(0)) {						// 0 = all pipes

      // RED led changes state when messages are received
			ledStatus ^= LED_RED;

			do {
				receiveMessageLength = rf24GetPayloadSize();
				more = rf24Read(&receiveMessage[0], receiveMessageLength);

				// Print received message to uart
				while(!UARTSend(&receiveMessage[0], receiveMessageLength));
				while(!UARTSend("\r\n", 2));

        // Thsi was added to auto-reply to messages from raspberry pi
        // Convert PING to PONG and add as ack payload
        if(receiveMessageLength >= 1)
            receiveMessage[1] = 'O';
            
        rf24WriteAckPayload(1, receiveMessage, receiveMessageLength);
			} while(more);							// Read until RX_EMPTY
		}
	}
	return 0;
}
