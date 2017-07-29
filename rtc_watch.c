/*
 * rtc_watch.c
 *
 *  Created on: Jun 2, 2014
 *      Author: Barun
 */

#include<avr/io.h>
#include<util/delay.h>
#define ack		1
#define no_ack	0
#define rtc_delay 0x1F
unsigned char writearr[7]={0x00,0x14,0x61,0x08,0x04,0x06,0x14};
unsigned char readarr[7];
void i2c_delay()
{
unsigned char i;
for(i=0;i<rtc_delay;i++);
}
/*sbit rs=P2^2;
sbit lcde=P2^3;
sbit d4=P2^4;
sbit d5=P2^5;
sbit d6=P2^6;
sbit d7=P2^7;
sbit sda=P2^0;               // Set P1.3 = SDA
sbit scl=P2^1;               //Set P1.4 = SCL

void clr_lcd();
void send_command(unsigned char);
void delay();
void senddata(unsigned char);

#define ack		1
#define no_ack	0
#define rtc_delay 0x1F
unsigned char writearr[7]={0x00,0x14,0x61,0x08,0x01,0x07,0x13};
unsigned char readarr[7];
void i2c_delay()
{
unsigned char i;
for(i=0;i<rtc_delay;i++);
}
void i2c_start()
{
sda=1;
scl=1;
i2c_delay();
sda=0;
i2c_delay();
scl=0;
}
void i2c_stop()
{
sda=0;
scl=1;
i2c_delay();
sda=1;
i2c_delay();
scl=0;
}
void writei2c(unsigned char Data)
{
	  unsigned char i;
	for (i=0;i<8;i++)
	{
        sda = (Data & 0x80) ? 1:0;
		scl=1;
		i2c_delay();
		scl=0;
		Data<<=1;
	}

  	scl = 1;
	i2c_delay();
	scl = 0;

}

unsigned char readi2c(bit ack_bit)
{
    	unsigned char i;
    unsigned char Data=0;

    sda=1;
	for (i=0;i<8;i++)
	{
		scl=1;
		Data<<=1;
		Data=(Data |sda);
		i2c_delay();
		scl=0;
	}

 	if (ack_bit==1)
	sda = 0; // Send ACK
	else
	sda=1; // Send NO ACK

	i2c_delay();
	scl=1;
	i2c_delay();
	scl=0;
	return Data;
}*/

/*unsigned char readbyte(unsigned char Addr)
{
   	unsigned char Data;
	i2c_start();
	writei2c(0xD0);
	writei2c(Addr);
	i2c_start();
	writei2c(0xD1);
	Data = readi2c(no_ack);
	i2c_stop();
   	return(Data);
}


void writebyte(unsigned char Addr,unsigned char Data)
{
	i2c_start();
	writei2c(0xD0);
	writei2c(Addr);
	writei2c(Data);
	i2c_stop();
}		*/

void readrtc(unsigned char*buff)
{
I2CStart();
i2c_delay();
I2CWriteByte(0xD0);
i2c_delay();
//writei2c(0x00);
I2CWriteByte(0x00);
i2c_delay();
//i2c_start();
I2CStart();
i2c_delay();
//writei2c(0xD1);
I2CWriteByte(0xD1);
i2c_delay();
*(buff+0)=I2CReadByte(ack);	// Second
*(buff+1)=I2CReadByte(ack);	// Minute
*(buff+2)=I2CReadByte(ack);	// hour
*(buff+3)=I2CReadByte(ack);	// Day
*(buff+4)=I2CReadByte(ack);	// date
*(buff+5)=I2CReadByte(ack);	// month
*(buff+6)=I2CReadByte(no_ack);	// year
I2CStop();
i2c_delay();
}

void writertc(unsigned char *buff)
{

	I2CStart();
	i2c_delay();
	I2CWriteByte(0xD0);
	i2c_delay();
	I2CWriteByte(0x00);
	i2c_delay();
	I2CWriteByte(*(buff+0));
	I2CWriteByte(*(buff+1));
	I2CWriteByte(*(buff+2));
	I2CWriteByte(*(buff+3));
	I2CWriteByte(*(buff+4));
	I2CWriteByte(*(buff+5));
	I2CWriteByte(*(buff+6));
	i2c_delay();
	I2CStop();
	i2c_delay();
}



void hextoascii(unsigned char readarr)
{
unsigned char temp;
temp=readarr;
temp=temp>>4;
temp=temp+0x30;
LCDsendChar(temp);
readarr=readarr&0x0f;
readarr=readarr+0x30;
LCDsendChar(readarr);
}
/*void bcdtohex_display(unsigned char data1)
{
char i=0,j;
unsigned char chargedataarr[2];
while(data1)
{
chargedataarr[i++]=data1%10;
data1/=10;
}
i--;
j=i;
for(;i>=0;i--)
{
chargedataarr[i]+=48;
senddata(chargedataarr[i]);
if(j==1 && i==0)
senddata(' ');
if(j==0 && i==0)
{
senddata(' ');
senddata(' ');
}
}} */



/****************FUNCTION FOR SWAPPING LSBYTE AND MSBYTE OF THE DATA***************/

/*unsigned char xch(unsigned char data1)
{
unsigned char temp,temp1;
temp=data1;
data1=data1>>4;
temp1=data1;
data1=temp;
data1=data1<<4;
data1=data1|temp1;
return(data1);
}

void datatr(unsigned char data1)
{
d4=data1&0x01;
data1=data1>>1;
d5=data1&0x01;
data1=data1>>1;
d6=data1&0x01;
data1=data1>>1;
d7=data1&0x01;
}*/




/********************** INITIALIZATION   OF   LCD  ***********************************/
 /*void lcdinit()
 {
                clr_lcd();             FUNCTION SET
                send_command(0x28);
                delay();
                send_command(0x28);
                delay();
                send_command(0x28);
                delay();
                send_command(0x06);			//ENTRY MODE
                delay();

                send_command(0x0c);			   //DISPLAY ON/OFF
                delay();

                clr_lcd();

}
*/
/* FUNCTION  FOR  DISPLAYING  DATA  ON  THE   LCD *************************************/

/*void dispslogan(unsigned char *p)
{
   unsigned char data1;
   while(*p)
	{
	data1=*p;
	senddata(data1);
	p++;
	}
}*/

/***************** FUNCTION FOR SENDING LCD COMMANDS***********************************/

/* void send_command(unsigned char data1)
   {
        unsigned char data2;
	    rs=0;
        delay();
        lcde=1;
         delay();
		 data2=xch(data1);
         datatr(data2);
		 lcde=0;
         delay();
         lcde=1;
        datatr(data1);
         delay();
         lcde=0;
         delay();
         rs=1;

 }*/

/************************** FUNCTION FOR WRITING DATA ON THE LCD***********************/

/*void senddata(unsigned char data1)
{
        unsigned char data2;
		rs=1;
        delay();
        lcde=1;
         delay();
        data2=xch(data1);
         datatr(data2);
		  lcde=0;
          delay();
          lcde=1;
          datatr(data1);
          delay();
          lcde=0;
          delay();
          rs=0;
}*/


/************ delay for 20 micro second **********************************************/

void delay()
{
unsigned char i,j;
for(i=0;i<40;i++)
{
for(j=0;j<20;j++)
{}
}}

/*********** COMMAND FOR BRINGING LCD CURSOR ON SECOND LINE ***************************/

/*void next_line()
{
send_command(0xc0);
delay();
}
void first_line()
{
send_command(0x80);
delay();
}*/

/**************COMMAND FOR CLEARING LCD AND BRINGING LCD CURSOR ON FIRST LINE********/

/*void clr_lcd()
{
send_command(0x01);
delay();
send_command(0x02);
delay();
}*/
void longdelay()
{
unsigned char i;
unsigned int j;
for(i=0;i<100;i++)
for(j=0;j<1275;j++);
}

void chkday()
{
switch(readarr[3])
{
case 0x01:LCDdisplay("Mon.");
          break;
case 0x02:LCDdisplay("Tue.");
          break;
case 0x03:LCDdisplay("Wed.");
          break;
case 0x04:LCDdisplay("Thu.");
          break;
case 0x05:LCDdisplay("Fri.");
          break;
case 0x06:LCDdisplay("Sat.");
          break;
case 0x07:LCDdisplay("Sun.");
          break;
}}

void main()
{
	DDRD=0xff;
unsigned char temp;
I2CInit();
longdelay();
LCDinit();
longdelay();
delay();
LCDGotoXY(0,0);
_delay_ms(20);
LCDdisplay("BARUN");
_delay_ms(20);
I2CWriteByte(writearr);
longdelay();
while(1)
{
//first_line();
	LCDsendCommand(0x80);
chkday();
hextoascii(readarr[4]);
LCDsendChar('/');
hextoascii(readarr[5]);
LCDsendChar('/');
LCDdisplay("20");
hextoascii(readarr[6]);
//next_line();
LCDsendCommand(0xc0);
temp=readarr[2]&0x20;
readarr[2]&=0x9F;
hextoascii(readarr[2]);
LCDsendChar(':');
hextoascii(readarr[1]);
LCDsendChar(':');
hextoascii(readarr[0]);

switch(temp)
{
case 0x00: LCDdisplay(" AM");
           break;
case 0x20: LCDdisplay(" PM");
           break;
}
delay();
delay();
delay();
delay();
delay();
readrtc(readarr);

}}



