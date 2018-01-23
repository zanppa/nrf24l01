#ifndef __COMMON_H__
#define __COMMON_H__

// Common #defines that should be available everywhere
#define CLOCKS_IN_US	  80	  // 80 MHz -> 80 clocks in one microsecond


#define TIMERS	  6
#define TIMER_CALLBACKS	 5

#define timerStep		 1000  // Timer running period [us] (system timer, i.e. thread timer)
#define commonTimerStep	 1000  // Common callback timer running period [us]

//#define TO_FAST_TIMER_VALUE(x) (x / fastTimerStep)
//#define TO_COMMON_TIMER_VALUE(x) (x / timerStep)

// Timer callback function prototype
typedef void (*timerCallbackFunction)(void *pdata, char caller);

// Fast timer (10 us timestep) callback function list
typedef struct _timerCallback {
  uint32_t timer;	 // Timer value
  timerCallbackFunction callback;  // Callback function
  void *data;	 // Data pointer to pass to callback, e.g. state
} timerCallback;


// Common (extern) variables
extern uint32_t commonTimer[TIMERS];
extern volatile timerCallback timerCb[TIMER_CALLBACKS];

extern volatile uint8_t waitMutex;		// Lock for the exact wait timer
extern volatile timerCallback waitCb;	// Callback for exact waitCb

#define TIMER_FREE	  0
#define TIMER_LOCK	  1
#define TIMER_RUN	  2

// Common helper functions

#define CALLER_TIMER	1
#define CALLER_THREAD	0

#define RETURN_WAIT	   0
#define RETURN_DONE	   1

#define TIMED_FUNCTION(name_args) char name_args(void *pdata, char caller)

#define TIMED_BEGIN()\
	static unsigned short _timed_pt = 0;\
	static char _timed_mutex = 0;\
	if(_timed_mutex && caller == CALLER_THREAD) return RETURN_WAIT;\
	switch(_timed_pt) {\
	case 0:

#define TIMED_END()\
	}\
	_timed_pt = 0;\
	_timed_mutex = 0;\
	return RETURN_DONE;

// Wait (and block) until exact timer is available
#define LOCK_TIMER()\
	do {\
				_timed_pt = __LINE__; case __LINE__:\
		if(waitMutex != TIMER_FREE) return RETURN_WAIT;\
		waitMutex = TIMER_LOCK;\
	} while(0)

// Schedule continuation with exact timer
// Note that this is called from interrupt, so if any other thread uses
// same resources as this while this timer is running, they need to be
// volatile or otherwise thread safe!
// Time is given in us
#define TIMER_WAIT(func, time)\
	do {\
		waitCb.callback = &func;\
				waitMutex = TIMER_RUN;\
				_timed_mutex = 1;\
		TimerLoadSet64(TIMER0_BASE, time * CLOCKS_IN_US);\
		TimerEnable(TIMER0_BASE, TIMER_A);\
				_timed_pt = __LINE__; case __LINE__:\
		if(waitMutex == TIMER_RUN) return RETURN_WAIT;\
				_timed_mutex = 0;\
	} while(0)

// Yield from exact timer (interrupt callback) and continue next time the thread is run
// (without releasing the timer mutex)
#define TIMER_YIELD()\
		do {\
			_timed_pt = __LINE__; case __LINE__:\
			if(caller == CALLER_TIMER) return RETURN_WAIT;\
		} while(0)

// Release the exact timer
#define RELEASE_TIMER()\
		do {\
		  waitCb.callback = 0;\
		   waitMutex = TIMER_FREE;\
		   _timed_pt = __LINE__; case __LINE__:\
		  if(caller == CALLER_TIMER) return RETURN_WAIT;\
		} while(0)


// Microsecond delay
void delayMicrosec(uint32_t time);

// Millisecond delay
void delayMillisec(uint32_t time);

// Initialize the timer used for exact wait macros
void InitTimedFunctions(void);

#endif
