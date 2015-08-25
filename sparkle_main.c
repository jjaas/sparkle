//*****************************************************************************
//
// sparkle_main.c - Sparkling main program
//
// Sets on PWM signal for LED control when interrupt signal is detected
// from PIR movement detector. After a period of no movement, turns the
// signal off.
//
// PIR signal interrupt is trigged on both signal edges.
// Interrupt turns on the PWM generator signal, ramping up the PWM pulse width.
// PWM pulse is used to drive the MOSFET, and thus smoothly lighting up the LED strip.
//
// Each interrupt resets the idle timer (TIMER1).

// When the no movement is detected for defined period (60sec), timer runs out
// causing another interrupt. This interrupt starts a PWM pulse width ramp-down,
// dimming the leds and finally turns off the PWM generator output.
//
// Pins used on EK-TM4C123GLX board
//
//   PA5  -  PIR sensor input
//   PC5  -  PWM generator output
//   PF2  -  On-board led, blue component
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
#include "driverlib/pwm.h"
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
#define LIGHTS_ON_PERIOD_SEC 60
#define PWM_FREQUENCY 1500
volatile uint32_t ui32Load;
volatile uint32_t ui32PWMClock;
volatile uint8_t ui8Adjust;
int ulPeriod;
void InitClocksGPIOAndTimer()
{
    ui8Adjust = 83;
    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    ROM_FPULazyStackingEnable();

    //
    // Set the clocking to run directly from the crystal.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_8);

    // PWM Setup
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

    ui32PWMClock = SysCtlClockGet() / 8;
    ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, ui32Load );

    // Started as not active
    PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);

    //
    // Enable peripheral and register interrupt handler
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOIntRegister(GPIO_PORTA_BASE, GPIOMotionIsr);

    //
    // Make pin 7 rising edge triggered interrupt.
    //
    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);

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
    ulPeriod = (SysCtlClockGet() ) ; // once per second

    TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod);
    TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod * LIGHTS_ON_PERIOD_SEC);

    IntEnable(INT_TIMER0A);
    IntEnable(INT_TIMER1A);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    TimerEnable(TIMER0_BASE, TIMER_A);

    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Enable the GPIO pin for blue LED component (PF2).
    //
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

}

int timerTrigged = 0, idleTimerTrigged = 0, interrupted = 0, ledOn = 0;
//*****************************************************************************
//
// Main loop
//
//*****************************************************************************
int
main(void)
{
	int main_delay;

	// Initialize relevant GPIO pins and periodic timer
    InitClocksGPIOAndTimer();

    // Initialize the UART.
    ConfigureUART();

    main_delay = SysCtlClockGet() / 200;

    UARTprintf("Started!\n");

    ulPeriod = (SysCtlClockGet() * LIGHTS_ON_PERIOD_SEC) ;

    // Set the blue led on at startup. Will shut down in first periodic timer trig.
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    while(1)
    {
        if (idleTimerTrigged)
        {
            // idle (no movement detected) timer trigged

            idleTimerTrigged = 0;
            UARTprintf("Idle timer trigged, turning leds off\n");

            if (GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5))
            {
                // level is still high - let's restart the timer
                UARTprintf("Level still high, restarting timer\n");
                TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod);
                TimerEnable(TIMER1_BASE, TIMER_A);
            }
            else
            {
                int i;
                // Turn the leds off
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
                for (i = ui32Load; i > 10; i-=10)
                {
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, i);
                    SysCtlDelay(main_delay/10);
                }

                PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);

            }
        }

        if (interrupted)
        {
            // Movement detected

            interrupted = 0;
            UARTprintf("Interrupted\n");

            if (TimerValueGet(TIMER1_BASE, TIMER_A) == TimerLoadGet(TIMER1_BASE, TIMER_A))
            {

                // Turn the led on
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);

                // Turn the PMW output on
                PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, true);
                PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 10);
                int i;
                for (i = 10; i < ui32Load; i+=10)
                {
                    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, i);
                    SysCtlDelay(main_delay/10);
                }

                UARTprintf("Starting timer\n");
                TimerEnable(TIMER1_BASE, TIMER_A);
            }
            else
            {
                // Leds are on at this point - no need to set those on again

                UARTprintf("RE-Starting timer\n");
                TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod);
                TimerEnable(TIMER1_BASE, TIMER_A);
            }
        }
        if (timerTrigged)
        {
            // periodic status led driving timer trigged

            int gpioActive = GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5);
            if (gpioActive)
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
                if (gpioActive)
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
                }
                else
                {
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0);
                }
            }
            timerTrigged = 0;
        }
        //
        // Delay for a bit.
        //
        SysCtlDelay(main_delay);
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
