#define F_CPU 14745600

#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include "lcd.h"
unsigned int threshold=2500;//threshold for black color
unsigned char sharp, distance, adc_reading;
unsigned int value;

unsigned char rec,data,block; //to store received data from UDR0
unsigned char ADC_Value;
#include "CPPFile1.cpp"//color sencor code

SIGNAL(SIG_USART0_RECV) 		// ISR for receive complete interrupt
{
	rec = UDR0; 				//making copy of data from UDR0 in 'data' variable

	//UDR2 = rec; 				//echo data back to PC

}


//Function To Initialize UART0
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart0_init(void)
{
 UCSR0B = 0x00; //disable while setting baud rate
 UCSR0A = 0x00;
 UCSR0C = 0x06;
 UBRR0L = 0x5F; //set baud rate lo
 UBRR0H = 0x00; //set baud rate hi
 UCSR0B = 0x98;
}

//Function To Initialize UART1
// desired baud rate:9600
// actual baud rate:9600 (error 0.0%)
// char size: 8 bit
// parity: Disabled
void uart2_init(void)
{
 UCSR2B = 0x00; //disable while setting baud rate
 UCSR2A = 0x00;
 UCSR2C = 0x06;
 UBRR2L = 0x5F; //set baud rate lo
 UBRR2H = 0x00; //set baud rate hi
 UCSR2B = 0x98;
}
void led_config(void)  //configuration setting for led on port H 5,6,7
{
	
	DDRH=0b11100000;
	PORTH=0x00;
}

void red_on(void)    //for red led
{
	PORTH=0b10000000;
}
void led_off(void)			//for led off
{
	PORTH=0x00;
}
void blue_on(void)		//for blue led
{
	PORTH=0b00100000;
}
void green_on(void)		//for green led
{
	PORTH=0b01000000;
}
//Function to configure LCD port
void lcd_port_config (void)
{
	DDRC = DDRC | 0xF7; //all the LCD pin's direction set as output
	PORTC = PORTC & 0x80; // all the LCD pins are set to logic 0 except PORTC 7
}

//ADC pin configuration
void adc_pin_config (void)
{
	DDRF = 0x00; //set PORTF direction as input
	PORTF = 0x00; //set PORTF pins floating
	DDRK = 0x00; //set PORTK direction as input
	PORTK = 0x00; //set PORTK pins floating
}

void color_sensor_pin_config(void)
{
	DDRD  = DDRD | 0xFE; //set PD0 as input for color sensor output
	PORTD = PORTD | 0x01;//Enable internal pull-up for PORTD 0 pin
}
//Function to Initialize PORTS
void port_init()
{
	led_config();
	lcd_port_config();
	adc_pin_config();
	color_sensor_pin_config();//color sensor pin configuration
}

//Function to Initialize ADC
void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}



//This Function accepts the Channel Number and returns the corresponding Analog Value
unsigned char ADC_Conversion(unsigned char Ch)
{
	unsigned char a;
	if(Ch>7)
	{
		ADCSRB = 0x08;
	}
	Ch = Ch & 0x07;
	ADMUX= 0x20| Ch;
	ADCSRA = ADCSRA | 0x40;		//Set start conversion bit
	while((ADCSRA&0x10)==0);	//Wait for ADC conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}



// This Function prints the Analog Value Of Corresponding Channel No. at required Row
// and Coloumn Location.
void print_sensor(char row, char coloumn,unsigned char channel)
{
	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}



void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA = EICRA | 0x02; // INT0 is set to trigger with falling edge
	EIMSK = EIMSK | 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

//ISR for color sensor
ISR(INT0_vect)
{
	pulse++; //increment on receiving pulse from the color sensor
}
// This Function calculates the actual distance in millimeters(mm) from the input
// analog value of Sharp Sensor.
unsigned int Sharp_GP2D12_estimation(unsigned char adc_reading)
{
	float distance;
	unsigned int distanceInt;
	distance = (int)(10.00*(2799.6*(1.00/(pow(adc_reading,1.1546)))));
	distanceInt = (int)distance;
	if(distanceInt>800)
	{
		distanceInt=800;
	}
	return distanceInt;
}

//Function To Initialize all The Devices
void init_devices()
{
 cli(); //Clears the global interrupts
 color_sensor_pin_interrupt_init();
 port_init();
 adc_init();
 uart0_init(); //Initailize UART0 for serial communiaction
 uart2_init(); //Initailize UART1 for serial communiaction
 sei();   //Enables the global interrupts
}

//Main Function
int main(void)
{
	init_devices();
	
	lcd_set_4bit();
	lcd_init();

	data='A';    //default data value
	while(1){
		color();        // detecting color of patch  in CPPfile1
		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calsulated in a variable "value".
		lcd_print(2,1,data,2);
		UDR0=data;			//sending the color through XBEE
		_delay_ms(500);
		if(value>100 &&value <150)
		{
			block='B';
		}
		else block='U';
		UDR0=block;			//sending the block status through XBEE
		_delay_ms(500);
		if(rec=='B')		//if rescue bot is Blocked
		{
			
			lcd_print(1,1,rec,3);
			
			if(block=='B')		//both blocked
			{
				blue_on();
			
			}
			else if(data=='R')			//color of patch is red
			{
				red_on();
				
			}
			else if(data=='G')			//color of patch is green	
			{
				green_on();
			
			}
		}
		else        //unblocked rescue bot 
		{	
			led_off();
			
		}		
		
		
	}
}
