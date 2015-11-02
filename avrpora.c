#define F_CPU 7327000UL

#include <inttypes.h>
#include <avr/io.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <util/delay.h>
//#include <stdio.h>

#define cbi(x,y) x &= ~(1<<(y))
#define sbi(x,y) x |= (1<<(y))

// Move:
// OCF1A = Move x interrupt
// OCF3A = Move y interrupt

// Maximum speed is affected by NUM_ACC_STEPS, ACC_STEP_SIZE and START_INTERVAL
// ACC_STEP_SIZE controls rate of acceleration
// NUM_ACC_STEPS controls the physical distance during which the acceleration (and deceleration) occurs
// START_INTERVAL controls the starting speed.

#define NUM_ACC_STEPS 500
#define ACC_STEP_SIZE 8
#define INTERVAL_SCALE ((uint32_t)16)
#define START_INTERVAL (((uint32_t)6000)<<(INTERVAL_SCALE))


volatile uint32_t x_steps_left;
volatile uint32_t y_steps_left;

volatile uint32_t x_cur_interval;
volatile uint32_t y_cur_interval;

volatile uint16_t x_acc_left;
volatile uint16_t y_acc_left;

volatile uint16_t x_dec_left;
volatile uint16_t y_dec_left;

#define X_PULSE_ON() sbi(PORTF, 0)
#define X_PULSE_OFF() cbi(PORTF, 0)
#define Y_PULSE_ON() sbi(PORTF, 1)
#define Y_PULSE_OFF() cbi(PORTF, 1)
#define Z_PULSE_ON() sbi(PORTF, 2)
#define Z_PULSE_OFF() cbi(PORTF, 2)

#define X_DIR_POS() sbi(PORTF, 3)
#define X_DIR_NEG() cbi(PORTF, 3)
#define Y_DIR_POS() sbi(PORTF, 4)
#define Y_DIR_NEG() cbi(PORTF, 4)
#define Z_DIR_POS() sbi(PORTF, 5)
#define Z_DIR_NEG() cbi(PORTF, 5)

volatile uint8_t move_z;


ISR(TIMER1_COMPA_vect) // X
{
	X_PULSE_ON();

	x_steps_left--;

	if(x_steps_left < x_dec_left)
	{  // Brake
		x_cur_interval += x_cur_interval >> ACC_STEP_SIZE;
		OCR1A = x_cur_interval>>INTERVAL_SCALE;
		// x_dec_left-- not needed.
	}
	else if(x_acc_left)
	{  // Accelerate
		x_cur_interval -= x_cur_interval >> ACC_STEP_SIZE;
		OCR1A = x_cur_interval>>INTERVAL_SCALE;
		x_acc_left--;
	}
	// else go full speed.

	if(x_steps_left == 0)
	{
		cbi(TIMSK, 4);
	}

	X_PULSE_OFF();

}


ISR(TIMER3_COMPA_vect) // Y
{
	if(move_z)
		Z_PULSE_ON();
	else
		Y_PULSE_ON();

	y_steps_left--;

	if(y_steps_left < y_dec_left)
	{  // Brake
		y_cur_interval += y_cur_interval >> ACC_STEP_SIZE;
		OCR3A = y_cur_interval>>INTERVAL_SCALE;
		// y_dec_left-- not needed.
	}
	else if(y_acc_left)
	{  // Accelerate
		y_cur_interval -= y_cur_interval >> ACC_STEP_SIZE;
		OCR3A = y_cur_interval>>INTERVAL_SCALE;
		y_acc_left--;
	}
	// else go full speed.

	if(y_steps_left == 0)
	{
		cbi(ETIMSK, 4);
	}

	if(moze_z)
		Z_PULSE_OFF();
	else
		Y_PULSE_OFF();
}



void start_stepping(uint32_t steps_x, uint32_t steps_y)
{
	x_steps_left = steps_x;
	y_steps_left = steps_y;

	if(steps_x < NUM_ACC_STEPS*2)
	{
		x_acc_left = steps_x>>1;
		x_dec_left = steps_x - x_acc_left;
	}
	else
	{
		x_acc_left = NUM_ACC_STEPS;
		x_dec_left = NUM_ACC_STEPS;
	}

	if(steps_y < NUM_ACC_STEPS*2)
	{
		y_acc_left = steps_y>>1;
		y_dec_left = steps_y - y_acc_left;
	}
	else
	{
		y_acc_left = NUM_ACC_STEPS;
		y_dec_left = NUM_ACC_STEPS;
	}

	TCNT1 = 0;
	TCNT3 = 0;
	x_cur_interval = START_INTERVAL;
	y_cur_interval = START_INTERVAL;
	OCR1A = x_cur_interval>>INTERVAL_SCALE;
	OCR3A = y_cur_interval>>INTERVAL_SCALE;

	TIFR = 0b00010000;
	ETIFR = 0b00010000;

	if(steps_x)
		sbi(TIMSK, 4);
	if(steps_y)
		sbi(ETIMSK, 4);

}

int main()
{
	TCCR1A = 0;
	TCCR3A = 0;

	DDRF = 0b00111111; // step x,y,z, dir x,y,z as outputs
	DDRA = 0b00000010; // ENA as output.

	PORTF = 0;
	PORTA = 0;


	TCCR1B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.
	TCCR3B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.

	sei();
	_delay_ms(1000);

	while(1)
	{
/*	X_DIR_POS();
	start_stepping(300, 0);
	while(x_steps_left);
	_delay_ms(1000);
	X_DIR_NEG();
	start_stepping(300, 0);
	while(x_steps_left);
	_delay_ms(1000);
*/
	X_DIR_POS();
	Y_DIR_POS();
	start_stepping(0, 2000);
	while(x_steps_left || y_steps_left);
	_delay_ms(1000);
	X_DIR_NEG();
	Y_DIR_NEG();
	start_stepping(0, 2000);
	while(x_steps_left || y_steps_left);
	_delay_ms(1000);

	X_DIR_POS();
	Y_DIR_POS();
	start_stepping(2000, 6000);
	while(x_steps_left || y_steps_left);
	_delay_ms(1000);
	X_DIR_NEG();
	Y_DIR_NEG();
	start_stepping(2000, 6000);
	while(x_steps_left || y_steps_left);
	_delay_ms(2000);
	}

	return 0;
}
