///////////////////////////////////////////////////////////////////////
// tinyWanderer.c
// written by dale wheat - 10 march 2011
///////////////////////////////////////////////////////////////////////

// notes:

// device = ATtiny13A
// clock = 9.6 MHz internal RC oscillator

#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

///////////////////////////////////////////////////////////////////////////////
// macros
///////////////////////////////////////////////////////////////////////////////

#define nop() asm("nop")

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))

///////////////////////////////////////////////////////////////////////
// global variable (gasp!) declarations
///////////////////////////////////////////////////////////////////////

volatile unsigned char left_chasm_detect = 0;
volatile unsigned char right_chasm_detect = 0;

volatile unsigned int left_sensor_led_off;
volatile unsigned int left_sensor_led_on;
volatile unsigned int right_sensor_led_off;
volatile unsigned int right_sensor_led_on;

///////////////////////////////////////////////////////////////////////
// delay() - a short delay
///////////////////////////////////////////////////////////////////////

void delay(void) {

	volatile unsigned int i;

	for(i = 0; i < 250; i++); // kill some time
}

///////////////////////////////////////////////////////////////////////
// nap() - go to sleep until n timer/counter interrupts occur
//    1 = ~13 mS
//   72 = ~1 second
///////////////////////////////////////////////////////////////////////

void nap(unsigned char n) {

	while(n) {

		set_sleep_mode(SLEEP_MODE_IDLE);
		sleep_mode();

		n--;
	}
}

///////////////////////////////////////////////////////////////////////
// LED control functions
///////////////////////////////////////////////////////////////////////

#define led_on() sbi(PORTB, PB4)
#define led_off() cbi(PORTB, PB4)

///////////////////////////////////////////////////////////////////////
// servo() - servo control function
///////////////////////////////////////////////////////////////////////

void servo(signed char left, signed char right) {

	// input values are relative speed functions for each servo
	// range is -10 to +10
	// special case 0 turns off servo pulses, stopping the motor

	if(left) {
		OCR0A = 27 + left;
		sbi(TCCR0A, COM0A1); // enable left servo output
	} else {
		cbi(TCCR0A, COM0A1); // disable left servo output
	}

	if(right) {
		OCR0B = 28 - right;
		sbi(TCCR0A, COM0B1); // enable right servo output
	} else {
		cbi(TCCR0A, COM0B1); // disable right servo output
	}
}

///////////////////////////////////////////////////////////////////////
// drive() - linear/angular velocity abstraction function
///////////////////////////////////////////////////////////////////////

void drive(signed char linear_velocity, signed char angular_velocity) {

	signed char left_servo_velocity;
	signed char right_servo_velocity;

	left_servo_velocity = linear_velocity - angular_velocity;
	right_servo_velocity = linear_velocity + angular_velocity;

	servo(left_servo_velocity, right_servo_velocity); // set individual wheel velocities
}

///////////////////////////////////////////////////////////////////////
// sensor functions
///////////////////////////////////////////////////////////////////////

#define LEFT_SENSOR 1
#define RIGHT_SENSOR 2

unsigned int sensor(unsigned char channel) {

	switch(channel) {

		case LEFT_SENSOR: // left sensor ADC1
			ADMUX = 0<<REFS0 | 0<<ADLAR | 0<<MUX1 | 1<<MUX0;
			break;

		case RIGHT_SENSOR: // right sensor ADC3
			ADMUX = 0<<REFS0 | 0<<ADLAR | 1<<MUX1 | 1<<MUX0;
			break;
	}

	ADCSRA = 1<<ADEN | 1<<ADSC | 0<<ADATE | 1<<ADIF | 0<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0; // start conversion
	while(bit_is_clear(ADCSRA, ADIF)); // wait for conversion to complete, and for data to be written to registers

	return ADC;
}

///////////////////////////////////////////////////////////////////////
// main() - main program function
///////////////////////////////////////////////////////////////////////

void main(void) {

	// local variable declarations

	signed char left_velocity __attribute__ ((unused)) = 0; // left wheel velocity
	signed char right_velocity __attribute__ ((unused)) = 0; // right wheel velocity

	signed char linear_velocity __attribute__ ((unused)) = 0; // linear velocity
	signed char angular_velocity __attribute__ ((unused)) = 0; // angular velocity

	// initialize everything first

	// initialize PORTB
	//
	//	name	pin		usage
	//	----	---		-----
	//	PB0		5		left servo output OC0A
	//	PB1		6		right servo output OC0B
	//	PB2		7		left sensor input ADC1
	//	PB3		2		right sensor input ADC3
	//	PB4		3		IR LED output
	//	PB5		1		not used

	PORTB = 0<<PORTB5 | 0<<PORTB4 | 0<<PORTB3 | 0<<PORTB2 | 1<<PORTB1 | 1<<PORTB0;
	DDRB = 0<<DDB5 | 1<<DDB4 | 0<<DDB3 | 0<<DDB2 | 1<<DDB1 | 1<<DDB0;

	// initialize ATtiny13 timer/counter

	TCCR0A = 0<<COM0A1 | 0<<COM0A0 | 0<<COM0B1 | 0<<COM0B0 | 0<<WGM01 | 1<<WGM00;
	TCCR0B = 0<<FOC0A | 0<<FOC0B | 0<<WGM02 | 1<<CS02 | 0<<CS01 | 0<<CS00;

	TCNT0 = 0;
	//OCR0A = 28; // 19 = reverse, 28 = stop, 38 = forward
	//OCR0B = 28; // 19 = forward, 28 = stop, 19 = reverse

	TIMSK0 = 0<<OCIE0B | 0<<OCIE0A | 1<<TOIE0; // interrupts

	// initialize ATtiny13 on-chip analog-to-digital converter

	ADMUX = 0<<REFS0 | 0<<ADLAR | 0<<MUX1 | 0<<MUX0;
	ADCSRA = 1<<ADEN | 1<<ADSC | 0<<ADATE | 1<<ADIF | 0<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 0<<ADPS0;
	DIDR0 = 0<<ADC0D | 0<<ADC2D | 1<<ADC3D | 0<<ADC1D | 1<<AIN1D | 0<<AIN0D;

	sei(); // enable global interrupts

	drive(0, 0); // stop robot

	// the main foreground program loop

	while(1) {

		if((!left_chasm_detect) && (!right_chasm_detect)) {

			drive(1, 0); // continue ahead slow

		} else if((left_chasm_detect) && (!right_chasm_detect)) {

			// back up and turn slightly to the right

			drive(0, 0); // stop
			nap(36); // for half a second
			drive(-1, -1); // spin back right
			nap(36); // for half a second

		} else if((!left_chasm_detect) && (right_chasm_detect)) {

			// back up and turn slightly to the left

			drive(0, 0); // stop
			nap(36); // for half a second
			drive(-1, 1); // spin back left
			nap(36); // for half a second

		} else {

			// back up and then spin in place

			drive(0, 0); // stop
			nap(36); // for half a second
			drive(-1, 0); // straight back
			nap(36); // for half a second
			drive(-1, -1); // spin back right
			nap(36); // for half a second
		}

		nap(1); // now go to sleep and wait for a timer interrupt (~73 Hz)
	}
}

///////////////////////////////////////////////////////////////////////
// timer/counter0 overflow interrupt handler
///////////////////////////////////////////////////////////////////////

ISR(TIM0_OVF_vect) {

	left_sensor_led_off = sensor(LEFT_SENSOR); // left sensor reading with LED off
	right_sensor_led_off = sensor(RIGHT_SENSOR); // right sensor ready with LED off

	led_on(); // turn on IR LED
	delay(); // a short delay
	left_sensor_led_on = sensor(LEFT_SENSOR); // left sensor reading with LED on
	right_sensor_led_on = sensor(RIGHT_SENSOR); // right sensor reading with LED on

	led_off(); // now turn off IR LED

	// OK, now interpret the sensor readings

	if((left_sensor_led_off - left_sensor_led_on) > 50) {
		left_chasm_detect = 0; // left-side flooring detected
	} else {
		left_chasm_detect = 1; // left-side chasm detected
	}

	if((right_sensor_led_off - right_sensor_led_on) > 50) {
		right_chasm_detect = 0; // right-side flooring detected
	} else {
		right_chasm_detect = 1; // right-side chasm detected
	}
}

///////////////////////////////////////////////////////////////////////

// [end-of-file]
