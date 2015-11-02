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


int main()
{

	DDRF = 0b00111111; // step x,y,z, dir x,y,z as outputs
	DDRA = 0b00000010; // ENA as output.

	PORTF = 0;
	PORTA = 0;

	uint16_t kak = 0;
	while(1)
	{
		sbi(PORTF, 0);
		sbi(PORTF, 1);
		_delay_us(5);
		cbi(PORTF, 0);
		cbi(PORTF, 1);

		if((kak > 8950 && kak < 9050) || kak > 17950 || kak < 50)
			_delay_ms(1.0);
		else if((kak > 8900 && kak < 9100) || kak > 17900 || kak < 100)
			_delay_ms(0.70);
		else if((kak > 8750 && kak < 9250) || kak > 17750 || kak < 250)
			_delay_ms(0.40);
		else if((kak > 8500 && kak < 9500) || kak > 17500 || kak < 500)
			_delay_ms(0.21);
		else if((kak > 8000 && kak < 10000) || kak > 17000 || kak < 1000)
			_delay_ms(0.125);
		else if((kak > 7750 && kak < 10250) || kak > 16750 || kak < 1250)
			_delay_ms(0.10);
		else if((kak > 7500 && kak < 10500) || kak > 16500 || kak < 1500)
			_delay_ms(0.08);
		else if((kak > 7000 && kak < 11000) || kak > 16000 || kak < 2000)
			_delay_ms(0.065);
		else if((kak > 6500 && kak < 11500) || kak > 15500 || kak < 2500)
			_delay_ms(0.05);
		else
			_delay_ms(0.035);


		if(kak == 9000)
		{
			sbi(PORTF, 3);
			sbi(PORTF, 4);

		}
		else if(kak == 18000)
		{
			kak = 0;
			cbi(PORTF, 3);
			cbi(PORTF, 4);
		}

		kak++;

		_delay_us(3);

	}

	return 0;
}
