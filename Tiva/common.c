#include <stdint.h>
typedef uint8_t bool;

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/rom_map.h"
#include "driverlib/rom.h"
#include "driverlib/timer.h"

#include "common.h"
#include "inc/hw_gpio.h"
#include "driverlib/gpio.h"


// Global variables
uint32_t commonTimer[TIMERS] = {0};
volatile timerCallback timerCb[TIMER_CALLBACKS] = {0};

volatile uint8_t waitMutex;		// Lock for the exact wait timer
volatile timerCallback waitCb;	// Callback for exact waitCb

void delayMicrosec(uint32_t time)
{
	// ROM_SysCtlDelay(ROM_SysCtlClockGet()/3000000 * time);
	SysCtlDelay(SysCtlClockGet()/3000000 * time);
}

void delayMillisec(uint32_t time)
{
	// ROM_SysCtlDelay(ROM_SysCtlClockGet()/3000 * time);
	SysCtlDelay(SysCtlClockGet()/3000 * time);
}
