//*****************************************************************************
//
// ignite.c - Illuminating main program
//
//*****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

void GPIOMotionIsr(void);
void PeriodicTimerIsr(void);
void IdleTimerIsr(void);

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>Hello World (hello)</h1>
//!
//! A very simple ``hello world'' example.  It simply displays ``Hello World!''
//! on the UART and is a starting point for more complicated applications.
//!
//! UART0, connected to the Virtual Serial Port and running at
//! 115,200, 8-N-1, is used to display messages from this application.
//
//*****************************************************************************

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);
}

void InitClocksGPIOAndTimer()
{
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                       SYSCTL_OSC_MAIN);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_2);

	//
	// Enable peripheral and register interrupt handler
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOIntRegister(GPIO_PORTA_BASE, GPIOMotionIsr);

	//
	// Make pin 7 rising edge triggered interrupt.
	//
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_RISING_EDGE);

	//
	// Enable the pin interrupts.
	//
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_5);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_PERIODIC);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_A_ONE_SHOT);
	TimerIntRegister(TIMER0_BASE, TIMER_A, PeriodicTimerIsr);
	TimerIntRegister(TIMER1_BASE, TIMER_A, IdleTimerIsr);
	int ulPeriod = (SysCtlClockGet() ) ; // once per second

	TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod * 15);

	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	TimerEnable(TIMER0_BASE, TIMER_A);
	TimerEnable(TIMER1_BASE, TIMER_A);

	//
	// Enable the GPIO port that is used for the on-board LED.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	//
	// Enable the GPIO pins for the LED (PF2 & PF3).
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

}

int timerTrigged = 0, idleTimerTrigged = 0, interrupted = 0, ledOn = 0;
int MAIN_DELAY;
//*****************************************************************************
//
// Print "Hello World!" to the UART on the evaluation board.
//
//*****************************************************************************
int
main(void)
{
    // Initialize relevant GPIO pins and periodic timer
    InitClocksGPIOAndTimer();

    // Initialize the UART.
    ConfigureUART();

    MAIN_DELAY = SysCtlClockGet() / 200;

    UARTprintf("Started!\n");

    int ulPeriod = (SysCtlClockGet() * 15) ;
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
	GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);
    while(1)
    {
    	if (idleTimerTrigged)
    	{
    		idleTimerTrigged = 0;
    		UARTprintf("Idle timer trigged, turning leds off\n");
    		// Turn the leds off
    		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, 0);
    	}


    	// Do nothing with this now
    	if (interrupted)
    	{
        	interrupted = 0;
    		UARTprintf("Interrupted\n");

    		if (TimerValueGet(TIMER1_BASE, TIMER_A) == TimerLoadGet(TIMER1_BASE, TIMER_A))
    		{
       			// Turn the led on
        		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
        		GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);

    			UARTprintf("Starting timer\n");
    			TimerEnable(TIMER1_BASE, TIMER_A);
    		}
    		else
    		{
    			// Turn the led on
    			GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    			GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_2, GPIO_PIN_2);

        		UARTprintf("RE-Starting timer\n");
        		TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod);
    			TimerEnable(TIMER1_BASE, TIMER_A);
    		}
    	}
    	if (timerTrigged)
    	{
    		if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))
    		{
    			UARTprintf("Input active (motion detected)\n");
    		}
    		if (TimerValueGet(TIMER1_BASE, TIMER_A) == TimerLoadGet(TIMER1_BASE, TIMER_A))
    		{
    			UARTprintf("Timer not running\n");
    		}
    		else
    		{
    			UARTprintf("Timer running for %d\n", (TimerValueGet(TIMER1_BASE, TIMER_A) + SysCtlClockGet()/2)  / SysCtlClockGet());
    		}
    		timerTrigged = 0;
    	}
        //
        // Delay for a bit.
        //
        SysCtlDelay(MAIN_DELAY);
    }
}

// Interrupt handler for the GPIO motion signal
void GPIOMotionIsr(void)
{
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_5);
	interrupted = 1;
}

// Interrupt handler for the periodic status interrupt
void PeriodicTimerIsr(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	timerTrigged = 1;
}

// Interrupt handler for the idle movement timer
void IdleTimerIsr(void)
{
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	idleTimerTrigged = 1;
}
