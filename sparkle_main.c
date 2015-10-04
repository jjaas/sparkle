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
//   PE4  -  HW switch input
//   PA2  -  IR detector input
//   PB6  -  PWM generator output for IR LED
//   PC5  -  PWM generator output for LED strip 1 (motion trigged)
//   PC4  -  PWM generator output for LED strip 2 (switch trigged)
//   PF2  -  On-board led, blue component
//
//*****************************************************************************
// Todo: make this runtime selectable, by the user buttons, etc.
//#define RUN_AS_MASTER
#define RUN_AS_SLAVE

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

void GPIOLightSwitchIsr(void);
void GPIOMotionDetectorIsr(void);
void SendIRCode(uint32_t code);

void blink_n(uint32_t n);
void IRIntHandler(void);
void IRTimerIsr(void);
void IdleTimerIsr(void);
void rampGenericPWM(bool rampDirection, uint32_t pwmBase, uint32_t pwm,
		uint32_t pwmBit, uint32_t delay);
int decodePulseBuffer(uint32_t *pulse_buf);

void delay_ms(uint32_t ui32Us);
void delay_us(uint32_t ui32Us);

#define LIGHTS_ON_PERIOD_SEC 60
#define PWM_FREQUENCY 500
#define PWM_LOW 10
#define PWM_STEP 10

#define PWM_RAMP_UP   true
#define PWM_RAMP_DOWN false

#define ON 1
#define OFF 0

// define PORT and PIN that the detector is connected to
#define IR_PORT          GPIO_PORTA_BASE
#define IR_PIN           GPIO_PIN_2

#define IR_TIMER_BASE    TIMER0_BASE
#define IR_TIMER         TIMER_A
#define IR_TIMER_PERIPH  SYSCTL_PERIPH_TIMER0
#define IR_TIMER_CFG     TIMER_CFG_A_ONE_SHOT
#define IR_TIMER_INT1    INT_TIMER1A
#define IR_TIMER_INT2    TIMER_TIMA_TIMEOUT

#define IR_TIMEOUT_VAL   20000 // 20 ms (20000 us) for now
#define MAX_PULSE_COUNT  100
#define IR_MAX_BITS_VAL  6 // couple of extra bits

// Todo: set better binary values
#define AMBIENCE_ON 0x1
#define AMBIENCE_OFF 0x2
#define SWITCH_ON 0x5
#define SWITCH_OFF 0x6

bool idleTimerTrigged = 0, motionDetectorTrigged = 0, lightSwitchTrigged = 0;
int lightSwitchState = 0;
int receivedIRCode = 0;
uint32_t ulPeriod;
uint32_t ui32Load;

volatile uint32_t ir_pulse_count = 0, ir_timeout_flag, ir_ppct;
volatile uint32_t g_ulIRPeriod, g_ulCountsPerMicrosecond;
uint32_t pulse_buf[MAX_PULSE_COUNT + 1];  // pulse width count buffer

#ifndef RUN_AS_MASTER
#ifndef RUN_AS_SLAVE
#error Define either role
#endif
#endif
#ifdef RUN_AS_MASTER
#ifdef RUN_AS_SLAVE
#error Define either role
#endif
#endif

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
void ConfigureUART(void) {
	//
	// Enable the GPIO Peripheral used by the UART.
	//
//	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

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

void InitClocksGPIOAndTimer() {
	uint32_t ui32PWMClock;

	//
	// Enable lazy stacking for interrupt handlers.  This allows floating-point
	// instructions to be used within interrupt handlers, but at the expense of
	// extra stack usage.
	//
	ROM_FPULazyStackingEnable();

	//
	// Set the clocking to run directly from the crystal.
	//
	ROM_SysCtlClockSet(
	SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);
	ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_1);

	// PWM Setup
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	GPIOPinConfigure(GPIO_PC4_M0PWM6);
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4);
	GPIOPinConfigure(GPIO_PC5_M0PWM7);
	GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);

	ui32PWMClock = SysCtlClockGet() / 8;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;

	PWMGenConfigure(PWM0_BASE, PWM_GEN_3,
	PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, ui32Load);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, PWM_LOW);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, PWM_LOW);

	// Started as not active
	PWMOutputState(PWM0_BASE, PWM_OUT_6_BIT, false);
	PWMOutputState(PWM0_BASE, PWM_OUT_7_BIT, false);
	PWMGenEnable(PWM0_BASE, PWM_GEN_3);

	// PWM Setup for IR LED
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	GPIOPinConfigure(GPIO_PB6_M0PWM0);
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
	PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 1050);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 525);

	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);

	//
	// Enable peripheral and register interrupt handler
	//
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOIntRegister(GPIO_PORTA_BASE, GPIOMotionDetectorIsr);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	GPIOIntRegister(GPIO_PORTE_BASE, GPIOLightSwitchIsr);

#ifdef RUN_AS_MASTER
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_5);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_BOTH_EDGES);
#else
	GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_2);
	GPIOIntTypeSet(GPIO_PORTA_BASE, GPIO_PIN_2, GPIO_FALLING_EDGE);
#endif

	GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_4);
	GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_RISING_EDGE);

	//
	// Enable the pin interrupts.
	//
	GPIOIntEnable(GPIO_PORTA_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_5);
	GPIOIntEnable(GPIO_PORTE_BASE, GPIO_INT_PIN_4);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_A_ONE_SHOT);
	TimerConfigure(TIMER1_BASE, TIMER_CFG_A_ONE_SHOT);
	TimerIntRegister(TIMER0_BASE, TIMER_A, IRTimerIsr);
	TimerIntRegister(TIMER1_BASE, TIMER_A, IdleTimerIsr);
	ulPeriod = (SysCtlClockGet()); // once per second

	TimerLoadSet(TIMER0_BASE, TIMER_A, ulPeriod);
	TimerLoadSet(TIMER1_BASE, TIMER_A, ulPeriod * LIGHTS_ON_PERIOD_SEC);
	TimerLoadSet(TIMER1_BASE, TIMER_B, ulPeriod * 3);

	IntEnable(INT_TIMER0A);
	IntEnable(INT_TIMER1A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

	//
	// Enable the GPIO port that is used for the on-board LED.
	//
	ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

	//
	// Enable the GPIO pin for blue LED component (PF2).
	//
	ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);

	g_ulCountsPerMicrosecond = ROM_SysCtlClockGet() / 1000000;
	// 10ms = timeout delay
	g_ulIRPeriod = g_ulCountsPerMicrosecond * IR_TIMEOUT_VAL;
}

//
// ramp the PWM pulse widht up or down
//
void rampTopPWM(bool rampDirection) {
	rampGenericPWM(rampDirection, PWM0_BASE, PWM_OUT_7, PWM_OUT_7_BIT,
			SysCtlClockGet() / 2048);
}
void rampBottomPWM(bool rampDirection) {
	rampGenericPWM(rampDirection, PWM0_BASE, PWM_OUT_6, PWM_OUT_6_BIT,
			SysCtlClockGet() / 4096);
}

void rampGenericPWM(bool rampDirection, uint32_t pwmBase, uint32_t pwm,
		uint32_t pwmBit, uint32_t delay) {

	// Start value is the current PWM pulse width regardless the ramp direction
	uint32_t i = PWMPulseWidthGet(pwmBase, pwm);

	if (rampDirection == PWM_RAMP_UP) {
		uint32_t targetPwmLoad = PWMGenPeriodGet(pwmBase, PWM_GEN_3);
		PWMOutputState(pwmBase, pwmBit, true);

		for (; i < targetPwmLoad; i += PWM_STEP)
		{
			PWMPulseWidthSet(pwmBase, pwm, i);
			SysCtlDelay(delay);
		}
	} else // rampDirection == PWM_RAMP_DOWN
	{
		for (; i > PWM_LOW; i -= PWM_STEP)
		{
			PWMPulseWidthSet(pwmBase, pwm, i);
			SysCtlDelay(delay);
		}

		PWMOutputState(pwmBase, pwmBit, false);
	}
}

//
// (re)start timer for idle period counting (no movement -> lights out)
//
void startIdleDetectionTimer(uint32_t period) {
	TimerLoadSet(TIMER1_BASE, TIMER_A, period);
	TimerEnable(TIMER1_BASE, TIMER_A);
}

//
// Sets the defined status led on or off
//
void setStatusLedState(bool statusOn) {
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, statusOn ? GPIO_PIN_2 : 0);
}

//
// returns motion detector GPIO pin status
//
bool motionDetectorGPIOActive() {
	return GPIOPinRead(GPIO_PORTA_BASE, GPIO_PIN_5) != 0;
}

//
// returns light switch GPIO pin status
//
int lightSwitchGPIOActive() {
	return GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4);
}

//
// returns idle timer running status, indicates whether lights are still on or not
//
bool idleTimerRunning() {
	return TimerValueGet(TIMER1_BASE, TIMER_A)
			!= TimerLoadGet(TIMER1_BASE, TIMER_A);
}

//
// returns idle timer remaining value in wall clock time
//
uint32_t idleTimerRemainingSecondsGet() {
	return (TimerValueGet(TIMER1_BASE, TIMER_A) + SysCtlClockGet() / 2)
			/ SysCtlClockGet();
}

//*****************************************************************************
//
// Main loop
//
//*****************************************************************************
int main(void) {

	// Initialize relevant GPIO pins and periodic timer
	InitClocksGPIOAndTimer();

	// Initialize the UART.
	ConfigureUART();

	UARTprintf("Started!\n");

	ulPeriod = (SysCtlClockGet() * LIGHTS_ON_PERIOD_SEC);

	// Set the blue led on at startup. Will shut down in first periodic timer trig.
	setStatusLedState(false);

	while (1) {
#ifdef RUN_AS_MASTER
		if (lightSwitchTrigged) {
			int i, realHit = 1;

			for (i = 0; i < 50; i++) {

				SysCtlDelay(SysCtlClockGet() / 1000);

				if (lightSwitchGPIOActive() == 0) {
					realHit = 0;
					break;
				}
			}
			if (realHit) {
				UARTprintf("Light switch hit\n");
				if (lightSwitchState == OFF) {
					lightSwitchState = ON;
					UARTprintf("Switching ON\n");
					SendIRCode(SWITCH_ON);
					rampBottomPWM(PWM_RAMP_UP);
				} else {
					lightSwitchState = OFF;
					UARTprintf("Switching OFF\n");
					SendIRCode(SWITCH_OFF);
					rampBottomPWM(PWM_RAMP_DOWN);
				}
			} else {
				UARTprintf("False hit\n");
			}
			lightSwitchTrigged = 0;

		}

		if (idleTimerTrigged) {
			// idle (no movement detected) timer trigged
			idleTimerTrigged = 0;

			UARTprintf("Idle timer trigged, turning leds off\n");

			if (motionDetectorGPIOActive()) {
				// Level is still high - let's restart the timer.

				UARTprintf("Level still high, restarting timer\n");
				startIdleDetectionTimer(ulPeriod);
			} else {
				setStatusLedState(false);

				// Turn the PMW output off
				SendIRCode(AMBIENCE_OFF);
				rampTopPWM(PWM_RAMP_DOWN);
				if (lightSwitchState == 1) {
					SendIRCode(SWITCH_OFF);
					rampBottomPWM(PWM_RAMP_DOWN);
					lightSwitchState = 0;
				}
			}
		}

		if (motionDetectorTrigged) {
			// Movement detected

			motionDetectorTrigged = 0;
			UARTprintf("motionDetectorTrigged\n");

			if (!idleTimerRunning()) {
				setStatusLedState(true);

				// Turn the PMW output on
				SendIRCode(AMBIENCE_ON);
				rampTopPWM(PWM_RAMP_UP);

				UARTprintf("Starting timer\n");
			} else {
				// Leds are on at this point - no need to set those on again
				UARTprintf("RE-Starting timer\n");
			}

			startIdleDetectionTimer(ulPeriod);
		}
		//if (timerTrigged)

		{
			// signal the state using on-board led and UART printouts

			bool gpioActive = motionDetectorGPIOActive();
			if (gpioActive) {
				UARTprintf("Input active (motion detected)\n");
			}
			if (!idleTimerRunning()) {
				UARTprintf("Timer not running\n");
			} else {
				UARTprintf("Timer running for %d\n",
						idleTimerRemainingSecondsGet());
				if (gpioActive) {
					setStatusLedState(true);
				} else {
					setStatusLedState(false);
				}
			}
		}
		//
		// Sleep until next interrupt
		//
#else
		UARTprintf("SLAVE LOOP: %d\n", ir_pulse_count);
		if (receivedIRCode != 0) {
			uint32_t _code = receivedIRCode;
			receivedIRCode = 0;
			switch (_code) {
			case AMBIENCE_ON:
				rampTopPWM(PWM_RAMP_UP);
				break;

			case AMBIENCE_OFF:
				rampTopPWM(PWM_RAMP_DOWN);
				break;

			case SWITCH_ON:
				rampBottomPWM(PWM_RAMP_UP);
				break;

			case SWITCH_OFF:
				rampBottomPWM(PWM_RAMP_DOWN);
				break;

			default:
				break;
			}

			// Blink out the code
			if (_code < 10) {
				blink_n(_code);
			}
		}
#endif
		SysCtlSleep();
	}
}
void blink_n(uint32_t n) {
	setStatusLedState(false);
	delay_ms(1000);
	int i;
	for (i = 0; i < n; i++) {
		setStatusLedState(true);
		delay_ms(200);
		setStatusLedState(false);
		delay_ms(200);
	}
}

// 3ms => bit0=6ms, bit1=9ms
#define T1 3
#define T2 (2*T1)
#define T4 (4*T1)

// Send bit pattern
// Start pulse first, then the bits, lsb first
void SendIRCode(uint32_t code) {

	blink_n(code);
	// 1. send start pattern
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	delay_ms(T4);

	// 2. send code bit-by-bit
	int i;
	// UARTprintf("TX: sending %x\n", code);
	for (i = 0; i < IR_MAX_BITS_VAL; i++) {

		// transmit start of the bit (PWM off)
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
		delay_ms(T1);

		// transmit end of the bit (PWM on)
		PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
		// int bit = (code >> i) & 0x1; // LSB first
		int bit = (code >> ((IR_MAX_BITS_VAL - 1) - i)) & 0x1; // MSB first
		// UARTprintf("tx: bit %d - %d\n", i, bit);
		if (bit) {
			delay_ms(T2);
		} else {
			delay_ms(T1);
		}
	}

	// 3. Set the PWM off
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, false);
}

// Interrupt handler for the GPIO motion detector signal
void GPIOMotionDetectorIsr(void) {
	GPIOIntClear(GPIO_PORTA_BASE, GPIO_INT_PIN_2 | GPIO_INT_PIN_5);
#ifdef RUN_AS_MASTER
	motionDetectorTrigged = 1;
#else
	IRIntHandler();
#endif

}

// Interrupt handler for the GPIO light switch signal
void GPIOLightSwitchIsr(void) {
	GPIOIntClear(GPIO_PORTE_BASE, GPIO_INT_PIN_4);
	lightSwitchTrigged = 1;
}

// Interrupt handler for the idle movement timer
void IdleTimerIsr(void) {
	TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
	idleTimerTrigged = 1;
}

// Interrupt handler IR detection timeout
void IRTimerIsr(void) {
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	GPIOIntTypeSet(IR_PORT, IR_PIN, GPIO_FALLING_EDGE);
	receivedIRCode = decodePulseBuffer(pulse_buf);
	ir_pulse_count = 0;

//timerTrigged = 1;
}

int decodePulseBuffer(unsigned int *pulse_buf) {
	unsigned int i, bitThreshold;
	unsigned int code;

// First pulse is a start bit representing 4 time periods(approx. 2.4ms??)
// calculate 2.5 time periods as the threshold between 2t and 3t
// any pulse width greater that 2.5t will be considered a logical '1'
	bitThreshold = (pulse_buf[0] * 5) / 8;
	if (bitThreshold < T1 * 1000) {
		// some disturbance
		return 0;
	}
	UARTprintf("bit threshold value: %d", bitThreshold);
	code = 0;
	i = 0;
	for (i = 0; i < IR_MAX_BITS_VAL; i++) { //Start with pulse/bit 12 (MSB)
		code <<= 1;
		if (pulse_buf[i + 1] > bitThreshold) { // If a '1' is detected..
			code |= 1;
		}
		UARTprintf("[%d] - Len: %d - Code: %d\n", i, pulse_buf[i], code);
	}
	UARTprintf("Received code: %d\n", code);

	return (int) code;
}

void IRIntHandler(void) {
	uint32_t ulTimerVal;

	ulTimerVal = TimerValueGet(IR_TIMER_BASE, IR_TIMER);  //Read timer value

// Reset the timer
	TimerLoadSet(IR_TIMER_BASE, IR_TIMER, g_ulIRPeriod);
	ir_timeout_flag = 0;

	if (ir_pulse_count == 0) {
// Change the IO pin to trig on RISING, because after this we are
// counting IR detector pulses, starting with RISING edge
		GPIOIntTypeSet(IR_PORT, IR_PIN, GPIO_RISING_EDGE);

// Start the timer
		TimerEnable(IR_TIMER_BASE, IR_TIMER);
	} else {
		TimerEnable(IR_TIMER_BASE, IR_TIMER);
		if (ir_pulse_count < MAX_PULSE_COUNT)
			pulse_buf[ir_pulse_count - 1] = (int) (g_ulIRPeriod - ulTimerVal)
					/ g_ulCountsPerMicrosecond;
	}
	ir_pulse_count++;

}
void delay_ms(uint32_t ui32Ms) {

// 1 clock cycle = 1 / SysCtlClockGet() second
// 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
// 1 second = SysCtlClockGet() / 3
// 0.001 second = 1 ms = SysCtlClockGet() / 3 / 1000

	ROM_SysCtlDelay(ui32Ms * (ROM_SysCtlClockGet() / 3 / 1000));
}

void delay_us(uint32_t ui32Us) {
	ROM_SysCtlDelay(ui32Us * (ROM_SysCtlClockGet() / 3 / 1000000));
}

