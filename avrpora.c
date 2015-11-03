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
#define ACC_STEP_Z_EXTRA 1
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

#define X_DIR_POS() cbi(PORTF, 3)
#define X_DIR_NEG() sbi(PORTF, 3)
#define Y_DIR_POS() cbi(PORTF, 4)
#define Y_DIR_NEG() sbi(PORTF, 4)
#define Z_DIR_POS() sbi(PORTF, 5)
#define Z_DIR_NEG() cbi(PORTF, 5)

#define STOP_LED_ON()  cbi(PORTA, 4)
#define STOP_LED_OFF() sbi(PORTA, 4)

#define PAUSE_LED_ON()  cbi(PORTA, 2)
#define PAUSE_LED_OFF() sbi(PORTA, 2)

#define Z_LED_ON() cbi(PORTG, 1)
#define Z_LED_OFF() sbi(PORTG, 1)


volatile uint8_t move_z = 0;
volatile uint8_t move_z_slowly = 0;


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

	if(!move_z_slowly)
	{
		if(y_steps_left < y_dec_left)
		{  // Brake
			y_cur_interval += y_cur_interval >> ((move_z?1:0)+ACC_STEP_SIZE);
			OCR3A = y_cur_interval>>INTERVAL_SCALE;
			// y_dec_left-- not needed.
		}
		else if(y_acc_left)
		{  // Accelerate
			y_cur_interval -= y_cur_interval >> ((move_z?1:0)+ACC_STEP_SIZE);
			OCR3A = y_cur_interval>>INTERVAL_SCALE;
			y_acc_left--;
		}
		// else go full speed.
	}

	if(y_steps_left == 0)
	{
		cbi(ETIMSK, 4);
	}

	if(move_z)
		Z_PULSE_OFF();
	else
		Y_PULSE_OFF();
}

void wait_step()
{
	while(x_steps_left || y_steps_left) ;
}


void step_x_1()
{
	X_PULSE_ON();
	_delay_us(5);
	X_PULSE_OFF();
}

void step_y_1()
{
	if(move_z)
		Z_PULSE_ON();
	else
		Y_PULSE_ON();
	_delay_us(5);
	if(move_z)
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

	x_cur_interval = START_INTERVAL;
	y_cur_interval = START_INTERVAL;

	TCNT1 = 0;
	TCNT3 = 0;
	OCR1A = x_cur_interval>>INTERVAL_SCALE;
	OCR3A = y_cur_interval>>INTERVAL_SCALE;

	TIFR = 0b00010000;
	ETIFR = 0b00010000;

	if(steps_x)
		sbi(TIMSK, 4);
	if(steps_y)
		sbi(ETIMSK, 4);

}

void start_z(uint32_t steps)
{
	uint8_t tmp = move_z;
	move_z = 1;
	start_stepping(0, steps);
	wait_step();
	move_z = tmp;
}


#define BUT(port, pin) ((~(port)) & (1<<(pin)))

#define XNEG() BUT(PINA, 6)
#define XPOS() BUT(PING, 2)
#define YNEG() BUT(PINC, 6)
#define YPOS() BUT(PINC, 4)
#define ZBUT() BUT(PING, 0)
#define KESKI() BUT(PINC, 3)
#define TEACH() BUT(PINC, 2)
#define XHOME() BUT(PINC, 1)
#define YHOME() BUT(PINC, 0)
#define ZHOME() BUT(PINE, 3)


void find_home_xy()
{

	X_DIR_NEG();
	Y_DIR_NEG();

	while(YHOME())
	{
		step_y_1();
		_delay_us(500);
	}

	while(XHOME())
	{
		step_x_1();
		_delay_us(500);
	}

	_delay_ms(10);

	X_DIR_POS();
	Y_DIR_POS();
	start_stepping(400, 400);
	wait_step();
	_delay_ms(10);

	X_DIR_NEG();
	Y_DIR_NEG();

	while(YHOME())
	{
		step_y_1();
		_delay_us(4000);
	}

	while(XHOME())
	{
		step_x_1();
		_delay_us(4000);
	}


}

void find_home_z()
{
	uint8_t tmp = move_z;
	move_z = 1;
	Z_DIR_POS();
	while(!ZHOME())
	{
		step_y_1();
		_delay_us(700);
	}

	_delay_ms(10);
	Z_DIR_NEG();
	move_z_slowly = 1;
	start_z(1500);
	wait_step();
	_delay_ms(10);

	Z_DIR_POS();

	while(!ZHOME())
	{
		step_y_1();
		_delay_us(5000);
	}

	move_z = tmp;
}

int main()
{
	TCCR1A = 0;
	TCCR3A = 0;

	DDRF = 0b00111111; // step x,y,z, dir x,y,z as outputs
	DDRA = 0b00010110; // outputs: stop led, pause led, stepper ena
	DDRG = 0b00000010; // outputs: led z

	PORTF = 0;
	PORTA = 0b11101000; // button pull-ups
	PORTC = 0b11111111; // button pull-ups
	PORTG = 0b101; // button pull-ups
	PORTE = 0b1000; // Z home pull-up.

	TCCR1B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.
	TCCR3B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.

	sei();
	PAUSE_LED_OFF();
	STOP_LED_OFF();
	Z_LED_OFF();
	_delay_ms(100);

	find_home_xy();
	find_home_z();

	while(1)
	{
		if(XNEG() && !move_z)
		{
			X_DIR_NEG();
			step_x_1();
		}
		else if(XPOS() && !move_z)
		{
			X_DIR_POS();
			step_x_1();
		}

		if(YNEG())
		{
			if(move_z)
				Z_DIR_NEG();
			else
				Y_DIR_NEG();

			step_y_1();
		}
		else if(YPOS())
		{
			if(move_z)
				Z_DIR_POS();
			else
				Y_DIR_POS();
			step_y_1();
		}

		if(ZBUT())
		{
			if(move_z)
			{
				move_z = 0;
				Z_LED_OFF();
			}
			else
			{
				move_z = 1;
				Z_LED_ON();
			}
			_delay_ms(10);
			while(ZBUT());
			_delay_ms(10);
		}

		if(TEACH())
		{
			Z_DIR_NEG();
			move_z_slowly=1;
			start_z(1000);
			_delay_ms(50);
			Z_DIR_POS();
			move_z_slowly=0;
			start_z(1000);
			_delay_ms(100);
		}


		if(KESKI())
			_delay_us(600);
		else
			_delay_us(3000);

	}

	return 0;
}
