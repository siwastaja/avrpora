// Move:
// OCF1A = Move x interrupt
// OCF3A = Move y interrupt

// Maximum speed is affected by NUM_ACC_STEPS, ACC_STEP_SIZE and START_INTERVAL
// ACC_STEP_SIZE controls rate of acceleration
// NUM_ACC_STEPS controls the physical distance during which the acceleration (and deceleration) occurs
// START_INTERVAL controls the starting speed.

#define NUM_ACC_STEPS 500
#define ACC_STEP_SIZE 6
#define INTERVAL_SCALE 16
#define START_INTERVAL (5000<<(INTERVAL_SCALE))


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

#define X_DIR_POS() sbi(PORTF, 3)
#define X_DIR_NEG() cbi(PORTF, 3)
#define Y_DIR_POS() sbi(PORTF, 4)
#define Y_DIR_NEG() cbi(PORTF, 4)

ISR(TIMER1_COMPA_vect) // X
{
	X_PULSE_ON();

	x_steps_left--;
	
	if(x_steps_left < x_dec_left)
	{  // Brake
		x_cur_interval += x_cur_interval>>ACC_STEP_SIZE;
		OCR1A = x_cur_interval>>INTERVAL_SCALE;		
		// x_dec_left-- not needed.
	}  // Accelerate
	else if(x_acc_left)
	{
		x_cur_interval -= x_cur_interval>>ACC_STEP_SIZE;
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

/*
ISR(TIMER3_COMPA_vect) // Y
{
	
}

*/

void start_stepping(uint32_t steps_x, uint32_t steps_y)
{
	x_steps_left = steps_x;

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

	TCNT1 = 0;
	x_cur_interval = START_INTERVAL;
	OCR1A = x_cur_interval>>INTERVAL_SCALE;		

	TIFR = 0b00010000;
//	TCNT3 = 0;
//	ETIFR = 0b00010000;

	sbi(TIMSK, 4);
//	sbi(ETIMSK, 4);
}

int main()
{
	TCCR1A = 0;
	TCCR3A = 0;
	
	TCCR1B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.
//	TCCR3B = 0b01001; // Clear-On-CompareMatch, No prescaler, timer on.
	
	sei();
	_delay_ms(1000);

	while(1)
	{
	X_DIR_POS();
	start_stepping(300, 0);
	while(x_steps_left);
	_delay_ms(100);
	X_DIR_NEG();
	start_stepping(300, 0);
	while(x_steps_left);
	_delay_ms(100);

	X_DIR_POS();
	start_stepping(2000, 0);
	while(x_steps_left);
	_delay_ms(100);
	X_DIR_NEG();
	start_stepping(2000, 0);
	while(x_steps_left);
	_delay_ms(100);
	
	X_DIR_POS();
	start_stepping(15000, 0);
	while(x_steps_left);
	_delay_ms(100);
	X_DIR_NEG();
	start_stepping(15000, 0);
	while(x_steps_left);
	_delay_ms(500);
	}
	
	return 0;
}
