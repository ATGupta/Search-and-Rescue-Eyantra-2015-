#define F_CPU 14745600
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h> //included to support power function
#include "lcd.h"
#include "djikstra.cpp"//included for the implementation of djikstra's algorithm

volatile unsigned long int ShaftCountLeft = 0; //to keep track of left position encoder
volatile unsigned long int ShaftCountRight = 0; //to keep track of right position encoder
volatile unsigned int Degrees; //to accept angle in degrees for turning

unsigned char ADC_Conversion(unsigned char);
unsigned char sharp;
unsigned int value;
unsigned char ADC_Value;
unsigned int search[]={1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,2};//1 for places not searched yet, O for elements searched , 2 for end of program
unsigned int val[][4]={ {68,77,79,88},{66,75,77,86},{64,75,73,84},{62,73,71,82},
						{68,57,47,59},{66,55,57,45},{64,55,53,43},{62,51,53,41},
						{22,31,41,33},{24,33,35,43},{26,35,45,37},{28,37,47,39},
						{22,11,2,13},{24,13,15,4},{26,15,6,17},{28,17,8,19}};// possible access point for each of 16 white boxes place
unsigned int count=0,box = 0,x = 0;
unsigned char flag = 0;
unsigned char Left_white_line = 0;
unsigned char Center_white_line = 0;
unsigned char Right_white_line = 0;

//Function to configure ports to enable robot's motion
void motion_pin_config (void)
{
	DDRA = DDRA | 0x0F;
	PORTA = PORTA & 0xF0;
	DDRL = DDRL | 0x18;   //Setting PL3 and PL4 pins as output for PWM generation
	PORTL = PORTL | 0x18; //PL3 and PL4 pins are for velocity control using PWM.
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
	DDRF = 0x00;
	PORTF = 0x00;
	DDRK = 0x00;
	PORTK = 0x00;
}

//Function to configure INT4 (PORTE 4) pin as input for the left position encoder
void left_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xEF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x10; //Enable internal pull-up for PORTE 4 pin
}

//Function to configure INT5 (PORTE 5) pin as input for the right position encoder
void right_encoder_pin_config (void)
{
	DDRE  = DDRE & 0xDF;  //Set the direction of the PORTE 4 pin as input
	PORTE = PORTE | 0x20; //Enable internal pull-up for PORTE 4 pin
}
void buzzer_pin_config (void)
{
	DDRC = DDRC | 0x08;		//Setting PORTC 3 as output
	PORTC = PORTC & 0xF7;		//Setting PORTC 3 logic low to turnoff buzzer
}

//Function to initialize ports
void port_init()
{
	motion_pin_config(); //robot motion pins config
	left_encoder_pin_config(); //left encoder pin config
	right_encoder_pin_config(); //right encoder pin config
	lcd_port_config();
	adc_pin_config();
	buzzer_pin_config();
}

// Timer 5 initialized in PWM mode for velocity control
// Prescale:256
// PWM 8bit fast, TOP=0x00FF
// Timer Frequency:225.000Hz
void timer5_init()
{
	TCCR5B = 0x00;	//Stop
	TCNT5H = 0xFF;	//Counter higher 8-bit value to which OCR5xH value is compared with
	TCNT5L = 0x01;	//Counter lower 8-bit value to which OCR5xH value is compared with
	OCR5AH = 0x00;	//Output compare register high value for Left Motor
	OCR5AL = 0xFF;	//Output compare register low value for Left Motor
	OCR5BH = 0x00;	//Output compare register high value for Right Motor
	OCR5BL = 0xFF;	//Output compare register low value for Right Motor
	OCR5CH = 0x00;	//Output compare register high value for Motor C1
	OCR5CL = 0xFF;	//Output compare register low value for Motor C1
	TCCR5A = 0xA9;	/*{COM5A1=1, COM5A0=0; COM5B1=1, COM5B0=0; COM5C1=1 COM5C0=0}
 					  For Overriding normal port functionality to OCRnA outputs.
				  	  {WGM51=0, WGM50=1} Along With WGM52 in TCCR5B for Selecting FAST PWM 8-bit Mode*/

	TCCR5B = 0x0B;	//WGM12=1; CS12=0, CS11=1, CS10=1 (Prescaler=64)
}

void adc_init()
{
	ADCSRA = 0x00;
	ADCSRB = 0x00;		//MUX5 = 0
	ADMUX = 0x20;		//Vref=5V external --- ADLAR=1 --- MUX4:0 = 0000
	ACSR = 0x80;
	ADCSRA = 0x86;		//ADEN=1 --- ADIE=1 --- ADPS2:0 = 1 1 0
}

//Function For ADC Conversion
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
	while((ADCSRA&0x10)==0);	//Wait for conversion to complete
	a=ADCH;
	ADCSRA = ADCSRA|0x10; //clear ADIF (ADC Interrupt Flag) by writing 1 to it
	ADCSRB = 0x00;
	return a;
}

//Function To Print Sesor Values At Desired Row And Coloumn Location on LCD
void print_sensor(char row, char coloumn,unsigned char channel)
{

	ADC_Value = ADC_Conversion(channel);
	lcd_print(row, coloumn, ADC_Value, 3);
}

void left_position_encoder_interrupt_init (void) //Interrupt 4 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x02; // INT4 is set to trigger with falling edge
	EIMSK = EIMSK | 0x10; // Enable Interrupt INT4 for left position encoder
	sei();   // Enables the global interrupt
}

void right_position_encoder_interrupt_init (void) //Interrupt 5 enable
{
	cli(); //Clears the global interrupt
	EICRB = EICRB | 0x08; // INT5 is set to trigger with falling edge
	EIMSK = EIMSK | 0x20; // Enable Interrupt INT5 for right position encoder
	sei();   // Enables the global interrupt
}

//ISR for right position encoder
ISR(INT5_vect)
{
	ShaftCountRight++;  //increment right shaft position count
}


//ISR for left position encoder
ISR(INT4_vect)
{
	ShaftCountLeft++;  //increment left shaft position count
}
//Function for velocity control
void velocity (unsigned char left_motor, unsigned char right_motor)
{
	OCR5AL = (unsigned char)left_motor;
	OCR5BL = (unsigned char)right_motor;
}

//Function used for setting motor's direction
void motion_set (unsigned char Direction)
{
	unsigned char PortARestore = 0;

	Direction &= 0x0F; 		// removing upper nibbel for the protection
	PortARestore = PORTA; 		// reading the PORTA original status
	PortARestore &= 0xF0; 		// making lower direction nibbel to 0
	PortARestore |= Direction; // adding lower nibbel for forward command and restoring the PORTA status
	PORTA = PortARestore; 		// executing the command
}
void buzzer_on (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore | 0x08;
	PORTC = port_restore;
}

void buzzer_off (void)
{
	unsigned char port_restore = 0;
	port_restore = PINC;
	port_restore = port_restore & 0xF7;
	PORTC = port_restore;
}

void forward (void) //both wheels forward
{
	motion_set(0x06);
}

void back (void) //both wheels backward
{
	motion_set(0x09);
}

void left (void) //Left wheel backward, Right wheel forward
{
	motion_set(0x05);
}

void right (void) //Left wheel forward, Right wheel backward
{
	motion_set(0x0A);
}

void soft_left (void) //Left wheel stationary, Right wheel forward
{
	motion_set(0x04);
}

void soft_right (void) //Left wheel forward, Right wheel is stationary
{
	motion_set(0x02);
}

void soft_left_2 (void) //Left wheel backward, right wheel stationary
{
	motion_set(0x01);
}

void soft_right_2 (void) //Left wheel stationary, Right wheel backward
{
	motion_set(0x08);
}

void stop (void)
{
	motion_set(0x00);
}


//Function used for turning robot by specified degrees
void angle_rotate(unsigned int Degrees)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = (float) Degrees/ 4.090; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned int) ReqdShaftCount;
	ShaftCountRight = 0;
	ShaftCountLeft = 0;

	while (1)
	{
		if((ShaftCountRight >= ReqdShaftCountInt) | (ShaftCountLeft >= ReqdShaftCountInt))
		break;
	}
	stop(); //Stop robot
}

//Function used for moving robot forward by specified distance

void linear_distance_mm(unsigned int DistanceInMM)
{
	float ReqdShaftCount = 0;
	unsigned long int ReqdShaftCountInt = 0;

	ReqdShaftCount = DistanceInMM / 5.338; // division by resolution to get shaft count
	ReqdShaftCountInt = (unsigned long int) ReqdShaftCount;

	ShaftCountRight = 0;
	while(1)
	{
		if(ShaftCountRight > ReqdShaftCountInt)
		{
			break;
		}
	}
	stop(); //Stop robot
}

void forward_mm(unsigned int DistanceInMM)
{
	forward();
	linear_distance_mm(DistanceInMM);
}

void back_mm(unsigned int DistanceInMM)
{
	back();
	linear_distance_mm(DistanceInMM);
}

void left_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	left(); //Turn left
	angle_rotate(Degrees);
}



void right_degrees(unsigned int Degrees)
{
	// 88 pulses for 360 degrees rotation 4.090 degrees per count
	right(); //Turn right
	angle_rotate(Degrees);
}


void soft_left_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left(); //Turn soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right();  //Turn soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_left_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_left_2(); //Turn reverse soft left
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

void soft_right_2_degrees(unsigned int Degrees)
{
	// 176 pulses for 360 degrees rotation 2.045 degrees per count
	soft_right_2();  //Turn reverse soft right
	Degrees=Degrees*2;
	angle_rotate(Degrees);
}

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
void left_count()
{
	while(1)
	{
		Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
		left(); //Turn left
		velocity(200,200);
		if(Center_white_line>0x15)
		return;
	}

}
void right_count()
{
	int flag=1;
	while(1)
	{
		Center_white_line = ADC_Conversion(2);	        //Getting data of Center WL Sensor
		//print_sensor(1,1,2);
		sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
		value = Sharp_GP2D12_estimation(sharp);			//Stores Distance calculated in a variable "value".
		//lcd_print(2,1,value,3); 						//Prints Value Of Distance in MM measured by Sharp Sensor.
		if(value>80 && value<100 &&flag==1)				//detecting number of white debris present
		{
			if(box==0)                      
			{
				box=1;

			}
			else if(box==1)                 
			box=2;
			flag=0;
		}
		if(value>100 &&flag==0)
		{
			flag=1;
		}
		right(); //Turn right
		velocity(200,200);
		if(Center_white_line>0x15)
		return;
	}

}

/*
	function to find next destination to reach
*/
int nextdest()
{
	int pos=0;
	for(int i=0;i<17;i++)// selecting one of the 16 points
	{
		if(search[i]==1) //if not traversed yet
		{
			pos=i;
			break;
		}
		if(search[i]==2) // all possible position to search white debris are traversed
		{
			stop();
			_delay_ms(2000);
			buzzer_on();
			_delay_ms(20000);
			break;
		}
	}
	for(int j=0;j<4;j++) // selecting one of the 4 access points for a white box
	{
		if(val[pos][j]!=0)
		{
			return val[pos][j];
		}
	}
	search[pos]=0;
	return 0;
}

/*
	function to align bot in the direction it has to start traversing 
*/
void makeitright(int x)
{
	int a=4-(x%4);      //current difference between turning right
	lcd_print(1,7,a,2);
	lcd_print(1,10,a+path[0],2);
	if((a+path[0])%4==0)
	{
	}
	else if((a+path[0])%4==2)//it is back
	{

		right_degrees(135); //Rotate robot right by 135 degrees
		right_count(); //Rotate robot right till it reaches a black line
	}
	else if((a+path[0])%4==3)//it is right
	{
		right_degrees(45); //Rotate robot right by 45 degrees
		right_count(); //Rotate robot right till it reaches a black line
	}
	if((a+path[0])%4==1)//it is left
	{
		left_degrees(45); //Rotate robot left by 45 degrees
		left_count(); //Rotate robot left till it reaches a black line
	}

}

//Function to initialize all the devices
void init_devices()
{
	cli(); //Clears the global interrupt
	port_init();  //Initializes all the ports
	adc_init();
	timer5_init();
	left_position_encoder_interrupt_init();
	right_position_encoder_interrupt_init();
	sei();   // Enables the global interrupt
}


//Main Function

int main(void)
{
	init_devices();
	lcd_set_4bit();
	lcd_init();
	flag=4;
	x=0;
	int etc=0,s_dest=0;					// "etc" - variable for first traversal from start position
	unsigned int current=49;			//49 - node number given to the start point
	for(int k=0;k<100;k++,etc++)		//max number bot can change destinations
	{

		if(etc==0)
		count=0;
		else
		count=1;
		s_dest=nextdest();				//value of next destination where white box will be checked

		lcd_print(1,1,current,2);
		lcd_print(1,4,s_dest,2);
		shortest_path(current,s_dest);//finding shortest path between current node and next destination implemented in djikstra
		makeitright(x);					//aligning the bot
		x=path[0];						
		while(1)
		{

			Left_white_line = ADC_Conversion(3);	//Getting data of Left WL Sensor
			Center_white_line = ADC_Conversion(2);	//Getting data of Center WL Sensor
			Right_white_line = ADC_Conversion(1);	//Getting data of Right WL Sensor
			//print_sensor(1,1,2);
			sharp = ADC_Conversion(11);						//Stores the Analog value of front sharp connected to ADC channel 11 into variable "sharp"
			value = Sharp_GP2D12_estimation(sharp);				//Stores Distance calculated in a variable "value".
			if(value>80 && value<130)						//if the distance is between 80 and 130 detection of black box in path
			{
				stop();
				_delay_ms(1000);
				if(count==tot-1)							//if black box is at the destination selected
				{
					for(int i=0;i<16;i++)
					{
						for(int j=0;j<4;j++)
						{
							if(val[i][j]==s_dest)
							val[i][j]=0;
						}
					}
					lcd_print(2,5,s_dest,2);
					rem(s_dest); //removing the node that have black box in it implemented in djikstra.cpp
					current=original[count-1];
					lcd_print(2,8,original[count-1],2);
				}
				else        //black debris detected in path(not at destination)
				{	
					lcd_print(2,5,original[count],2);
					rem(original[count]);
					current=original[count-1];
					lcd_print(2,8,original[count-1],2);
				}
				break;
			}
			
			if(Center_white_line<0x15 && Left_white_line<0x15 && Right_white_line<0x15) //when all sensors are on white
			{
				if (flag==1)
				{
					forward();
					velocity(250,250);
				}
				if (flag==2)
				{
					forward();
					velocity(170,250);
				}
				if (flag==3)
				{
					forward();
					velocity(250,170);
				}
				if(flag==4)
				{
					forward();
					velocity(250,250);
				}

			}
			
			else if(Center_white_line>0x15 && Left_white_line<0x15 && Right_white_line<0x15)//center sensed color is black
			{
				forward();
				velocity(250,250);
				flag=1;
			}
			else if(Center_white_line<0x15 && Left_white_line>0x15 && Right_white_line<0x15)//left sensed color is black
			{
				flag=2;
				forward();
				velocity(170,250);
			}
			else if(Center_white_line<0x15 && Left_white_line<0x15 && Right_white_line>0x15)//right sensed color is black
			{
				flag=3;
				forward();
				velocity(250,170);
			}
			else if((Center_white_line>0x15 && Left_white_line>0x15 && Right_white_line>0x15)||(Center_white_line>0x15 && Left_white_line>0x15 && Right_white_line<0x15)||(Center_white_line>0x15 && Left_white_line<0x15 && Right_white_line>0x15)) //when all the three sensors are on black or center_sensor with any one of left or right sensor is on black  
			{
				flag=4;

				lcd_print(2,1,path[count],2);
				if(path[count]==1)//it is left
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 75mm
					left_degrees(45); //Rotate robot left by 45 degrees
					left_count();	//Rotate robot left till it reaches a black line
					//j++;
				}
				else if(path[count]==4)//it is straight
				{
					x+=path[count];
					//j++;
					forward_mm(45); //Moves robot forward 45mm
				}
				else if(path[count]==3)//it is right
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 75mm
					right_degrees(45); //Rotate robot right by 45 degrees
					right_count();		//Rotate robot right till it reaches a black line
					stop();
					_delay_ms(1000);
				}
				else if(path[count]==2)//it is back
				{
					x+=path[count];
					forward_mm(75); //Moves robot forward 75mm
					right_degrees(130); //Rotate robot right by 130 degrees
					right_count();		//Rotate robot right till it reaches a black line
				}
				else if(count==tot-1)//destination is reached
				{
					lcd_print(2,12,x,2);
					for(int i=0;i<16;i++)
					{
						for(int j=0;j<4;j++)
						{
							if(val[i][j]==s_dest)
							search[i]=0;
						}
					}
					forward_mm(75); //Moves robot forward 75mm

					box=0;
					right_degrees(45); //Rotate robot right by 90 degrees
					right_count();
					right_degrees(45); //Rotate robot right by 90 degrees
					right_count();
					stop();
					for(unsigned int j=0;j<box;j++)//using buzzer to indicate white debris detected
					{
						buzzer_on();
						_delay_ms(2000);
						buzzer_off();
						_delay_ms(500);
					}
					current=path[count];
					//makeitright(x);
					break;
				}
				count++;
			}
			//break;
		}


	}

}
