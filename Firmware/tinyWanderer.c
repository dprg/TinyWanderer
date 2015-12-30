/****************************************************************************
*     Copyright (C) 2011 Paul Bouchier                                      *
*                                                                           *
*     This program is free software: you can redistribute it and/or modify  *
*     it under the terms of the GNU General Public License as published by  *
*     the Free Software Foundation, either version 3 of the License, or     *
*     (at your option) any later version.                                   *
*                                                                           *
*     This program is distributed in the hope that it will be useful,       *
*     but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*     GNU General Public License for more details.                          *
*                                                                           *
*     You should have received a copy of the GNU General Public License     *
*     along with this program.  If not, see <http://www.gnu.org/licenses/>, *
*     or the LICENSE-gpl-3 file in the root directory of this repository.   *
****************************************************************************/

#include <stdlib.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#include "tinyWanderer.h"


volatile unsigned int Tick;   // 100KHz pulse
volatile unsigned int rSpeed;   // right servo pulse width
volatile unsigned int lSpeed;		// left servo pulse width
volatile unsigned int Tick20ms;   // Servo frame variable
volatile unsigned char junk;
volatile unsigned int lastTick20ms;			// used to tell when Tick_20ms has changed
volatile unsigned char moveServoSmState, n_moveServoSmState;
volatile unsigned char seekTo;
volatile unsigned int nextMoveAt;
volatile unsigned char adcSmState, n_adcSmState;
volatile unsigned int lProximityDiff, rProximityDiff;	// difference between LED on & off from proximity phototransistors
volatile unsigned char analyzeLightSmState, n_analyzeLightSmState;
volatile unsigned char moveDelta;
volatile unsigned char driveDirection;
volatile unsigned int endBackupTime;

// Look for light changes & drive servo if seen
volatile unsigned char ledOffValue;	// phototransistor reading with IR illuminator off

/* Port use:
PB0: left wheel
PB1: right wheel servo
PB2/ADC1: left light sensor
PB3/ADC3: right light sensor
PB4: IR LED
*/

///////////////////////////////////////////////////////////////////////////////
// init() - initialize everything
// note:  this "function" is in section .init3 so it is executed before main()
///////////////////////////////////////////////////////////////////////////////
 
void init(void) __attribute__ ((naked, section(".init3")));
void init(void) {
   // initialize HW
   sei(); //  Enable global interrupts
   DDRB = (1<<PB0) | (1<<PB1) | (1<<PB4); // PB0-1,4 as outputs
   PORTB = 0;
   TCCR0A |= (1<<WGM01); // Configure timer 1 for CTC mode
   TIMSK |= (1<<OCIE0A); // Enable CTC interrupt
   OCR0A = 60; // Set CTC compare value
   //OCR0A = 150;
   TCCR0B |= (1<<CS00); // No prescaler

// Analog Comparator initialization
// Analog Comparator: Off
   ACSR=0x80;

// ADC initialization
// ADC Clock frequency: 150 kHz
// ADC Voltage Reference: AREF pin, 0 - VCC
// Only the 8 most significant bits of the AD conversion result are used, result left adjusted
   ADCSRA=0x86;

   Tick = 0;
   lSpeed = LSERVO_STOP;
   rSpeed = RSERVO_STOP;
   Tick20ms = 0;
   moveServoSmState = 0;
   nextMoveAt = Tick20ms + 1;
   moveDelta = 4;
   driveDirection = FWD_STRAIGHT;
   lastTick20ms = Tick20ms;
   adcSmState = 0;
   lProximityDiff = rProximityDiff = 0;
   analyzeLightSmState = 0;
}

void main(void) __attribute__ ((naked,noreturn));
void main (void)
{
   lSpeed = LSERVO_STOP; // FIXME: why does this get cleared?
   rSpeed = RSERVO_STOP;
   nextMoveAt = Tick20ms + 1;	
   nextMoveAt = normalize20ms(nextMoveAt);
   while(1)
   {
   //_delay_loop_2(1000);	// do nothing in main loop except wait to get interrupted
      //junk++;
   }
}

static void moveServoSm ()
{
      lastTick20ms = Tick20ms;
      switch (moveServoSmState) {
        case 0:
		 if (Tick20ms == nextMoveAt) {	// If done with delay at stopped
			lSpeed += moveDelta;		// ramp up to next speed
			rSpeed = lSpeed;
		    nextMoveAt = Tick20ms + 50;	// set pause to 50 * 20ms = 1s before change speed again
		    nextMoveAt = normalize20ms(nextMoveAt);
			n_moveServoSmState = 1;
		 }
		 break;
		case 1:			// moving up in period
		 if (Tick20ms == nextMoveAt) {	// if done with delay at this speed
		    nextMoveAt = Tick20ms + 50;	// set pause to 50 * 20ms = 1s before change speed again
		    nextMoveAt = normalize20ms(nextMoveAt);
			lSpeed += moveDelta;	// ramp up to next speed
			rSpeed = lSpeed;
		    if (lSpeed > LSERVO_FWD) {
			   lSpeed -= moveDelta;	// ramp down to next speed;
			   rSpeed = lSpeed;
			   n_moveServoSmState = 2;
            } 
	     }
		 break;
        case 2:		// moving down in period
		 if (Tick20ms == nextMoveAt) {	// if done with delay at this speed
		    nextMoveAt = Tick20ms + 50;	// set pause to 50 * 20ms = 1s before change speed again
		    nextMoveAt = normalize20ms(nextMoveAt);
			lSpeed -= moveDelta;	// ramp down to next speed
			rSpeed = lSpeed;
		    if (lSpeed < LSERVO_BACK) {
			   lSpeed = LSERVO_STOP;	// stop servo & wait 5 sec
			   rSpeed = RSERVO_STOP;
		       nextMoveAt = Tick20ms + 250;	// set pause to 250 * 20ms = 5s before change speed again
		       nextMoveAt = normalize20ms(nextMoveAt);
			   n_moveServoSmState = 0;
            } 
	     }		 
		 break;
        default:
		 n_moveServoSmState = 0;
      }
}


// ADC state machine measures reflected IR light from L & R sensors
void adcSm()
{
   unsigned int adcValue;

   switch (adcSmState) {

     case 0:						// Initialization state, only here at start
      lProximityDiff = rProximityDiff = 0; // init to no edge seen
      ADCSRA|=0x10;	// clear the adc interrupt flag
      //sPulseL = 50 + (adcValue/2);	// temp move servo based on input

	  PORTB &= ~(1<<PB4);			// turn IR LED off
   	  ADMUX = ADC_VREF_TYPE | 1;	// left justify result, select left IR phototransistor
      ADCSRA |= 0x40;				// start the next conversion
	  n_adcSmState = 1;
	  break;

     case 1:		// read left proximity photosensor value with IR LED off
	  ledOffValue = ADCH;
      ADCSRA|=0x10;	// clear the adc interrupt flag

	  PORTB |= (1<<PB4);			// turn IR LED on

   	  ADMUX = ADC_VREF_TYPE | 1;	// left justify result, select left IR phototransistor
      ADCSRA |= 0x40;				// start the next conversion
	  n_adcSmState = 2;
      break;

	 case 2:	// read left proximity photosensor with IR LED on
      adcValue = ADCH;
      ADCSRA|=0x10;	// clear the adc interrupt flag

	  lProximityDiff = abs(ledOffValue - adcValue);

	  PORTB &= ~(1<<PB4);			// turn IR LED off
   	  ADMUX = ADC_VREF_TYPE | 3;	// left justify result, select right IR photoxistor
      ADCSRA |= 0x40;				// start the next conversion
	  n_adcSmState = 3;
      break;

     case 3:		// read right proximity photosensor value with IR LED off
	  ledOffValue = ADCH;
      ADCSRA|=0x10;	// clear the adc interrupt flag

	  PORTB |= (1<<PB4);			// turn IR LED on

   	  ADMUX = ADC_VREF_TYPE | 3;	// left justify result, select right IR phototransistor
      ADCSRA |= 0x40;				// start the next conversion
	  n_adcSmState = 4;
      break;

	 case 4:	// read right proximity photosensor with IR LED on
      adcValue = ADCH;
      ADCSRA|=0x10;	// clear the adc interrupt flag

	  rProximityDiff = abs(ledOffValue - adcValue);

	  PORTB &= ~(1<<PB4);			// turn IR LED off
   	  ADMUX = ADC_VREF_TYPE | 1;	// left justify result, select left IR photoxistor
      ADCSRA |= 0x40;				// start the next conversion
	  n_adcSmState = 1;
      break;

	 default:
	  n_adcSmState = 0;
   }
}

// set motor speed depending on requested direction
void setSpeed()
{
   switch (driveDirection) {
     case FWD_STRAIGHT: lSpeed = LSERVO_FWD; rSpeed = RSERVO_FWD; break;
     case FWD_LEFT: lSpeed = LSERVO_FWD_SLO; rSpeed = RSERVO_FWD; break;
     case FWD_RIGHT: lSpeed = LSERVO_FWD; rSpeed = RSERVO_FWD_SLO; break;
     case BACK_STRAIGHT: lSpeed = LSERVO_BACK; rSpeed = RSERVO_BACK; break;
     case BACK_LEFT: lSpeed = LSERVO_BACK_SLO; rSpeed = RSERVO_BACK; break;
     case BACK_RIGHT: lSpeed = LSERVO_BACK; rSpeed = RSERVO_BACK_SLO; break;
     default: lSpeed = LSERVO_STOP; rSpeed = RSERVO_STOP;
   }
}

// kick off motion & detect something, then wait for it all to end
void analyzeLightSm()
{
   switch (analyzeLightSmState) {

     case 0:				// go forward watching for a cliff
      if (lProximityDiff < LIGHT_DIFF_TRIGGER) {
	     lSpeed = LSERVO_BACK_SLO;
		 rSpeed = RSERVO_BACK;
		 endBackupTime = Tick20ms + 40;	// set pause to 2s before fwd again
		 endBackupTime = normalize20ms(endBackupTime);
	     n_analyzeLightSmState = 1;
      } else if (rProximityDiff< LIGHT_DIFF_TRIGGER) {
	     lSpeed = LSERVO_BACK;
		 rSpeed = RSERVO_BACK_SLO;
		 endBackupTime = Tick20ms + 40;	// set pause to 2s before fwd again
		 endBackupTime = normalize20ms(endBackupTime);
		 n_analyzeLightSmState = 1;
      } else {
	     lSpeed = LSERVO_FWD;
		 rSpeed = RSERVO_FWD;
	  }
	  break;

     case 1:				// back away from cliff in designated direction for designated time
	  if (Tick20ms == endBackupTime) {
	     n_analyzeLightSmState = 0;
	  }
	  break;

	 default:
	  n_analyzeLightSmState = 0;
   }
}

void lightToSpeed()
{
   if (lProximityDiff > LIGHT_DIFF_TRIGGER) {
      //driveDirection = BACK_RIGHT;
	  lSpeed = LSERVO_FWD;
   } else {
      lSpeed = LSERVO_BACK;
   } 
   if (rProximityDiff > LIGHT_DIFF_TRIGGER) {
      rSpeed = RSERVO_FWD;
   } else {
      rSpeed = RSERVO_BACK;
   }
   //setSpeed();
}

// generate clock for use by state machines, and run 50Hz machines
void Timer ()
{
   if(Tick >= 2000)   // One servo frame (20ms) completed
      {
        Tick = 0;
        Tick20ms++;
		Tick20ms = normalize20ms(Tick20ms);	// if exceeds max, subtract max

		/*
		 * Run the servo frame-rate state machines
		 */
        adcSm();			// run the adc converter state machine
		analyzeLightSm();		// analyze the light readings
        //moveServoSm();		// run the servo movement state machine every servo frame
	 	//lightToSpeed();
		// assign next state
		adcSmState = n_adcSmState;
		analyzeLightSmState = n_analyzeLightSmState;
		moveServoSmState = n_moveServoSmState;
      }

   Tick = Tick + 1;
}

void servoPulseSm ()
{
//lSpeed = LSERVO_CENTER;
   if(Tick <= rSpeed) {  // Generate servo pulse
      PORTB |= (1<<PB1);   // Servo pulse high
   } else {
      PORTB &= ~(1<<PB1);   // Servo pulse low
   }


   if(Tick <= lSpeed) {   // Generate servo pulse
      PORTB |= (1<<PB0);   // Servo pulse high
   } else {
      PORTB &= ~(1<<PB0);   // Servo pulse low
   }
}

/*
This interrupt gets called every 10us when the timer expires. It runs all state machines
which do whatever they do, then exits.
*/

ISR(TIM0_COMPA_vect) __attribute__ ((naked));   // 100 KHz interrupt frequency
ISR(TIM0_COMPA_vect)
{
   Timer();	// update the 10us & 20ms Tick count & run servo frame-rate machines
   servoPulseSm();		// run the servo pulse state machine

   asm("reti");			// return from interrupt
} 

