/*
 * gsm_test2.c
 *
 *  Created on: Jun 9, 2014
 *      Author: Barun
 */

/*
 * gsm_test.c
 *
 *  Created on: Jun 9, 2014
 *      Author: Barun
 */

#include<avr/io.h>
#include<uart_lib.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<string.h>
#define bulb1R PA0
#define bulb2R PA1
#define bulb3R PA2
#define fan1R PA3
#define fan2R PA4
#define ac1R PA5
#define motor1R PA6
#define doorlockerR PA7
#define ctrl PORTA
unsigned char i;
ISR(USART_RCX_vect)
    {
	   static unsigned char Arr[12] ,n,*ptc;
	   Arr[12]=UDR;
	    n=strlen(Arr);
     ptc=&Arr[0]+n;
	 if(*ptc=='N')
	   {
		 if(strncmp(Arr,"bulb1ON", 7==0))
			   ctrl=(1<< bulb1R);
		   //  _delay_ms(1000);
		 if(strncmp( Arr,"bulb2ON", 7==0))
		 			ctrl=(1<< bulb2R);
		    //  _delay_ms(1000);
		 if(strncmp(Arr,"bulb3ON",7==0))
		 			ctrl=(1<< bulb3R);
		      //   _delay_ms(1000);
		 if(strncmp(Arr,"fan1ON", 6==0))
		 			ctrl=(1<<fan1R);
		        // _delay_ms(1000);
		 if(strncmp(Arr,"fan2ON", 6==0))
		 			 ctrl=(1<<fan2R);
		           // _delay_ms(1000);
		 if(strncmp(Arr,"ac1ON", 5==0))
		 			 ctrl=(1<<ac1R);
		          // _delay_ms(1000);
		 if(strncmp(Arr,"motor1ON", 8==0))
		 			ctrl=(1<< motor1R);
		         // _delay_ms(1000);
		 if(strncmp(Arr,"doorlockerON",12==0))
		 			ctrl=(1<< doorlockerR);
		          //  _delay_ms(1000);
	   }
	 else
	    {
		 if(strncmp(Arr,"bulb1OFF", 8==0))
		 			 ctrl=(0<<bulb1R);
		 		   //  _delay_ms(1000);
		 		 if(strncmp( Arr,"bulb2OFF", 8==0))
		 		 			 ctrl=(0<<bulb2R);
		 		    //  _delay_ms(1000);
		 		 if(strncmp(Arr,"bulb3OFF",8==0))
		 		 			ctrl=(0<< bulb3R);
		 		      //   _delay_ms(1000);
		 		 if(strncmp(Arr,"fan1OFF", 7==0))
		 		 			ctrl=(0<< fan1R);
		 		        // _delay_ms(1000);
		 		 if(strncmp(Arr,"fan2OFF", 7==0))
		 		 			ctrl=(0<< fan2R);
		 		            _delay_ms(1000);
		 		 if(strncmp(Arr,"ac1OFF", 6==0))
		 		 			ctrl=(0<< ac1R);
		 		          // _delay_ms(1000);
		 		 if(strncmp(Arr,"motor1ON", 8==0))
		 		 			 ctrl=(0<<motor1R);
		 		         // _delay_ms(1000);
		 		 if(strncmp(Arr,"doorlockerON",12==0))
		 		 			 ctrl=(0<<doorlockerR);
		 		          //  _delay_ms(1000);
	    }





    }

int main()
{
	// unsigned char x=0;
	PORTA=0xff;
	set_uartbaud(9600);
	sei();
	while(1){
	         sendstring_uart("AT+CMGF=1\r\n");
			 _delay_ms(2000);
		//	sendstring_uart("AT+CMGS=\"9023454253\"\r\n");
			_delay_ms(500);
	  if(PINA&(1<<bulb1R))
		 ctrl=(1<< bulb1R);
	  if(PINA&(0<<bulb1R))
	 	  ctrl=(0<<bulb1R);

		 if(PINA&(1<<bulb2R))
		  ctrl=(1<<bulb2R);
		if(PINA&(0<<bulb2R))
		   ctrl=(0<<bulb2R);
		 if(PINA&(1<<bulb3R))
			ctrl=(1<<bulb3R);
		 if(PINA&(1<<bulb3R))
			ctrl=(0<<bulb3R);
		 if(PINA&(1<<fan1R))
			ctrl=(1<< fan1R);
		 if(PINA&(0<<fan1R))
		 	ctrl=(0<<fan1R);
		 if(PINA&(1<<fan2R))
		 	ctrl=(1<<fan2R);
		 if(PINA&(0<<fan2R))
		 	ctrl=(0<<fan2R);
		 if(PINA&(1<<ac1R))
			 ctrl=(1<<ac1R);
		 if(PINA&(0<<ac1R))
		 	ctrl=(0<<ac1R);
		 if(PINA&(1<<motor1R))
			 ctrl=(1<<motor1R);
		 if(PINA&(0<<motor1R))
		 	ctrl=(0<<motor1R);
		 if(PINA&(1<<doorlockerR))
		 	ctrl=(1<<doorlockerR);
		 if(PINA&(0<<doorlockerR))
		 	ctrl=(0<<doorlockerR);














		/*	sendstring_uart("AT+CMGF=1\r\n");
				_delay_ms(2000);
			sendstring_uart("AT+CMGS=\"9560900906\"\r\n");
				_delay_ms(500);
			sendstring_uart("developed by barun");
			_delay_ms(500);
			sendchar_uart(0x1A);
			_delay_ms(3000);
			_delay_ms(3000);
			_delay_ms(3000);
			_delay_ms(3000);
			//x++;*/
	       }
	}
