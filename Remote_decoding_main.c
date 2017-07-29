/*
 * Remote_decoding_main.c
 *
 *  Created on: Jul 8, 2014
 *      Author: Barun
 */

/*
 * Ir_remote_16int0_test3.c
 *
 *  Created on: Jan 20, 2014
 *      Author: sparklab
 */

#include <avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
//#include <lcd_lib.h>
#include <timer0.h>
#include <ext_interrupt.h>
int A; char b[5]=" ";
int i=0;
ISR(INT0_vect)
{
	if(i>16){
			if(TCNT0>100)
				{
				A|=1<<(i-17);
				}
			else A&=~(1<<(i-17));
			}
	i++;
	TCNT0=0;
}

ISR(TIMER0_OVF_vect)
{ i=0;
}

int main()
{
	//LCDinit();
	//LCDclr();
	DDRD=0xf0;
	DDRB=255;
	set_timer0_normal();
	set_timer0_prescalar(3);
	enable_timer0_overflowint();
	enable_INT0(3);
	sei();
	while(1)
	{
	/*
	LCDGotoXY(0,0);
	sprintf(b,"%5x",A);
	LCDdisplay(b);*/
	if(A==0xf50a)
	{
		PORTB=0x0f;
		_delay_ms(1000);
		//LCDGotoXY(0,0);
		//LCDdisplay("Barun");
	}
	if(A==0xe41b)
		{
		PORTB=0xf0;
		_delay_ms(1000);
			//LCDGotoXY(0,0);
			//LCDdisplay("Prakash");
		}

	}

}


