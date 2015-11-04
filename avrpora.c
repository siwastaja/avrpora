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


// 1989 mils = 4000 x steps
// 1996 mils = 4000 y steps


#define MAX_X 12000
#define MAX_Y 14000
#define MAX_Z 10000

volatile int16_t cur_x, cur_y, cur_z;
volatile uint8_t cur_x_dir, cur_y_dir, cur_z_dir;

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


void x_dir(uint8_t pos)
{
	cur_x_dir = pos;
	if(pos) X_DIR_POS(); else X_DIR_NEG();
}

void y_dir(uint8_t pos)
{
	cur_y_dir = pos;
	if(pos) Y_DIR_POS(); else Y_DIR_NEG();
}

void z_dir(uint8_t pos)
{
	cur_z_dir = pos;
	if(pos) Z_DIR_POS(); else Z_DIR_NEG();
}


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

	cur_x += cur_x_dir?1:-1;
}

void step_y_1()
{
	Y_PULSE_ON();
	_delay_us(5);
	Y_PULSE_OFF();

	cur_y += cur_y_dir?1:-1;
}

void step_z_1()
{
	Z_PULSE_ON();
	_delay_us(5);
	Z_PULSE_OFF();

	cur_z += cur_z_dir?1:-1;
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

void move_xy(int16_t x_mils, int16_t y_mils)
{
	int32_t x_tmp, y_tmp;

	x_tmp = x_mils;
	x_tmp *= 40000;
	x_tmp /= 19890;

	y_tmp = y_mils;
	y_tmp *= 40000;
	y_tmp /= 19960;


	if(x_tmp < 0) { x_tmp *= -1; x_dir(0);} else x_dir(1);
	if(y_tmp < 0) { y_tmp *= -1; y_dir(0);} else y_dir(1);

	start_stepping(x_tmp, y_tmp);
}


#define Z_ACC_STEPS 60

void step_z(uint16_t steps, uint8_t down, uint8_t interval)
{
	uint16_t acc_steps, dec_steps;

	if(interval < Z_ACC_STEPS+10)
		interval = Z_ACC_STEPS+10;
	if(interval > 250)
		interval = 250;

	if(down) Z_DIR_NEG(); else Z_DIR_POS();

	if(steps < Z_ACC_STEPS*2)
	{
		acc_steps = steps>>1;
		dec_steps = steps - acc_steps;
	}
	else
	{
		acc_steps = Z_ACC_STEPS;
		dec_steps = Z_ACC_STEPS;
	}

	while(steps--)
	{
		uint8_t interval_tmp;
		step_z_1();
		if(steps < dec_steps)
			interval++;
		else if(acc_steps)
		{
			interval--;
			acc_steps--;
		}

		interval_tmp = interval;
		while(interval_tmp--) _delay_us(15);
	}
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
#define GOTOBUT() BUT(PINC, 5)

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

	cur_x = 0;
	cur_y = 0;
}

void find_home_z()
{
	if(ZHOME())
	{
		Z_DIR_NEG();
		while(ZHOME())
		{
			step_z_1();
			_delay_us(800);
		}
		step_z(200, 1, 100);
		_delay_ms(50);
	}

	Z_DIR_POS();
	while(!ZHOME())
	{
		step_z_1();
		_delay_us(800);
	}

	_delay_ms(50);
	step_z(200, 1, 100);
	_delay_ms(50);

	Z_DIR_POS();

	while(!ZHOME())
	{
		step_z_1();
		_delay_us(5000);
	}

	cur_z = 200;

}

int main()
{
	uint8_t move_z = 0;
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
		if(KESKI() && GOTOBUT())
		{
			find_home_xy();
			find_home_z();
		}

		if(XNEG() && !move_z)
		{
			if(cur_x > 0)
			{
				x_dir(0);
				step_x_1();
			}
		}
		else if(XPOS() && !move_z)
		{
			if(cur_x < MAX_X)
			{
				x_dir(1);
				step_x_1();
			}
		}

		if(YNEG())
		{
			if(move_z)
			{
				if(cur_z > 0)
				{
					z_dir(0);
					step_z_1();
					cur_z--;
				}
			}
			else if(cur_y > 0)
			{
				y_dir(0);
				step_y_1();
			}
		}
		else if(YPOS())
		{
			if(move_z)
			{
				if(cur_z < MAX_Z)
				{
					z_dir(1);
					step_z_1();
					cur_z++;
				}
			}
			else if(cur_y < MAX_Y)
			{
				y_dir(1);
				step_y_1();
			}
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
			step_z(1000, 1, 130);
			_delay_ms(100);
			step_z(1000, 0, 80);
		}

		if(KESKI())
			_delay_us(600);
		else
			_delay_us(3000);

	}

	return 0;
}
