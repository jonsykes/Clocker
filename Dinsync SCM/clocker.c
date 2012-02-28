/*
DINSync clock module: clock in->x24 out
*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdarg.h>
#include <ctype.h>


/** SETTINGS **/
//MIN_PW 10 is about 1.28ms
#define MIN_PW 10
#define MIN_ADC_DRIFT 1
#define USER_INPUT_POLL_TIME 100


/**MNEMONICS**/
#define CLKIN 0
//1 is x1
//2 is x24
#define INTERNAL 3

/** PINS **/

#define CLOCK_IN_pin PD2
#define CLOCK_IN_init DDRD &= ~(1<<CLOCK_IN_pin)
#define CLOCK_IN (PIND & (1<<CLOCK_IN_pin))

#define CLOCK_LED_pin PD3
#define CLOCK_LED_init DDRD |=(1<<CLOCK_LED_pin)
#define CLOCK_LED_PORT PORTD

#define DEBUG_pin PD1
#define DEBUG_init DDRD|=(1<<DEBUG_pin)
#define DEBUGHIGH PORTD |= (1<<DEBUG_pin)
#define DEBUGLOW PORTD &= ~(1<<DEBUG_pin)
#define DEBUGFLIP PORTD ^= (1<<DEBUG_pin)

#define OUT_PORT1 PORTB
#define OUT_DDR1 DDRB
#define OUT_MASK1 0b00111111
#define OUT_init1 OUT_DDR1 |= OUT_MASK1

#define OUT_PORT2 PORTD
#define OUT_DDR2 DDRD
#define OUT_MASK2 0b11000000
#define OUT_init2 OUT_DDR2 |= OUT_MASK2

#define BREAKOUT_MASK 0b111100
#define BREAKOUT_PULLUP PORTC
#define BREAKOUT_PIN PINC
#define BREAKOUT_DDR DDRC
#define BREAKOUT_init BREAKOUT_DDR &= ~(BREAKOUT_MASK)

#define SWITCHES_PULLUP PORTD |= ((1<<PD4) | (1<<PD5))

#define RESYNC (PINC & (1<<PC4))

/** MACROS **/

#define ALLON(p,x) p &= ~(x)
#define ALLOFF(p,x) p |= (x)
#define ON_NOMUTE(p,x) p &= ~(1<<(x))
#define ON(p,x)  p &= ~(1<<(x))
#define OFF(p,x) p |= (1<<(x))


/** TIMER **/
#define TMROFFSET 0

volatile uint32_t tmr[4];
volatile uint32_t clockin_irq_timestamp=0;



SIGNAL (SIG_OVERFLOW0){
	tmr[CLKIN]++;
	tmr[1]++;
	tmr[2]++;
	tmr[INTERNAL]++;
	TCNT0=0; // Re-init timer
}

uint32_t gettmr(uint8_t i){
	uint32_t result;
	cli();
	result = tmr[i];
	sei();
	return result;
}

void reset_tmr(uint8_t i){
	cli();
	tmr[i]=0;
	sei();
}

void inittimer(void){
	//Fast PWM , TOP at 0xFF, OC0 disconnected, Prescale @ FCK/8
	//@ 16MHz, timer increments at 2MHz, overflows at 128us
	//
	TCCR0A=(1<<WGM01) | (1<<WGM00) | (0<<COM0A0) | (0<<COM0A1);
	TCCR0B= (0<<WGM02) | (0<<CS00) | (1<<CS01) | (0<<CS02);
	TCNT0=TMROFFSET;
	TIMSK0|=(1<<TOIE0); 					// Enable timer overflow interrupt

	tmr[CLKIN]=0;
	tmr[1]=0;
	tmr[2]=0;
	tmr[INTERNAL]=0;
	sei();
}

ISR (INT0_vect){
	if (CLOCK_IN){
		clockin_irq_timestamp=tmr[CLKIN];
		tmr[CLKIN]=0;
		tmr[INTERNAL]=0;
	}
}

void pcint_init(void){
	//interrupt on any change on INT0 (PD2/clockin)
	EICRA = (1<<ISC00) | (0<<ISC01);
	EIMSK = (1<<INT0);
}


uint8_t diff(uint8_t a, uint8_t b);
inline uint8_t diff(uint8_t a, uint8_t b){
	if (a>b) return (a-b);
	else return (b-a);
}

uint32_t div32x8(uint32_t a, uint8_t b); 

uint32_t div32_8(uint32_t a, uint8_t b);
inline uint32_t div32_8(uint32_t a, uint8_t b){
//takes 27-42us if a is 3,5,6,7
//takes <5us if a is 1,2,4,8

	if (b==1) return (a);
	else if (b==2) return (a>>1);
	else if (b==4) return (a>>2);
	else if (b==8) return (a>>3);
	else if (b==16) return (a>>4);
	else return div32x8(a,b);
}



/** MAIN **/

int main(void){
        
	uint32_t per_x1=0, per_x24=0;

	uint32_t pw_x1=0, pw_x24=0; //pulse width amount for each jack

	uint8_t p1=0,p24=0; //current pulse # in the sequence for each jack

	char got_internal_clock=0;

	uint32_t now=0;

	char resync_up=0;

	uint8_t min_pw=MIN_PW;
	

	inittimer();

	CLOCK_IN_init; 
	CLOCK_LED_init;
	DEBUG_init;
	OUT_init1;
	OUT_init2;
	BREAKOUT_init; 

	pcint_init();

	while(1){

		if (RESYNC){
			if (resync_up==0){
				resync_up=1;
				
				p1=0;p24=0;
				ON(OUT_PORT1,0);
				ON(OUT_PORT1,5);

			}
		} else resync_up=0;


		if (CLOCK_IN)
			ON_NOMUTE(CLOCK_LED_PORT,CLOCK_LED_pin);
		else
			OFF(CLOCK_LED_PORT,CLOCK_LED_pin);



		/* 
			Check to see if the clock input timer tmr[INTERNAL] has surpassed the period
			If so, 
		*/
		now=gettmr(INTERNAL);
		if (now>=per_x1){
			got_internal_clock=1;
		}
	
		if (got_internal_clock || (clockin_irq_timestamp && CLOCK_IN)){

			/*	Clock IN received:
				Turn all outputs on
				Reset their timers
				Reset their pN counters
			*/
			cli();

				if (!clockin_irq_timestamp){
					if (tmr[INTERNAL]>per_x1)
						tmr[INTERNAL]=tmr[INTERNAL]-per_x1;
				} else {
					per_x1=clockin_irq_timestamp;
					clockin_irq_timestamp=0;
				}

				tmr[1]=0;
				tmr[2]=0;
			sei();

			ON(OUT_PORT1,0);
			ON(OUT_PORT1,3);

			got_internal_clock=0;
			p1=0;p24=0;

			pw_x1=per_x1>>1;
			pw_x24=div32_8(pw_x1,24);

			min_pw=MIN_PW+1;

			if ((per_x1>min_pw) && (pw_x1<MIN_PW)) pw_x1=MIN_PW;
			if ((per_x24>min_pw) && (pw_x24<MIN_PW)) pw_x24=MIN_PW;

			per_x24=div32_8(per_x1,24);
			
		}



		//Jack 1: x1, no skip, no shuffle
		//use d0,per0 which defaults to x1
		now=gettmr(1);
		if (now>=per_x1) {
			reset_tmr(1);
			ON(OUT_PORT1,0);
		}

		//Jack 2: x2, no skip, no shuffle
		//use d1/per1, it defaults to x2
		now=gettmr(2);
		if (now>=per_x24) {
			reset_tmr(2);
			ON(OUT_PORT1,5);
		}

/* Turn jacks off whenever the pulsewidth has expired*/

		if (gettmr(1)>=pw_x1) OFF(OUT_PORT1,0);
		if (gettmr(2)>=pw_x24) OFF(OUT_PORT1,5);

	}	//endless loop


	return(1);
}
