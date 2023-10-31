<<<<<<< HEAD
/*
 * Test.c
 *
 * Created: 2020-04-03 10:47:55
 * Author : Gustav
 */ 

// AVR2 (styr)

// MAX 1850 styrvinkel

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

// https://www.electronicwings.com/avr-atmega/interfacing-lcd-16x2-in-4-bit-mode-with-atmega-16-32-

#define F_CPU 1000000UL					/* Define CPU Frequency e.g. here its 8MHz */
#include <util/delay.h>					/* Include inbuilt defined Delay header file */

#define LCD_Dir DDRA					/* Define LCD data port direction */
#define LCD_Port PORTA					/* Define LCD data port */
#define RS PA4							/* Define Register Select (data reg./command reg.) signal pin */
#define EN PA5 							/* Define Enable signal pin */

char line1[16];
char line2[16];
char line3[16];
char line4[16]; // På rad 4 kan hallonpajen skriva vad den vill
volatile int line4_index_from_pi = 0;

volatile uint8_t ip_bytes[4];

volatile uint8_t paprika = 42;
volatile int16_t steering_angle = 0;
uint16_t actual_steering_angle = 0;
volatile int16_t speed = 0;

volatile uint8_t update_angle = 0;
volatile uint8_t update_speed = 0;

volatile enum state {IDLE, WAITING_FOR_STEERING_ANGLE_HIGH, WAITING_FOR_STEERING_ANGLE_LOW, 
	WAITING_FOR_IP_HIGHEST, WAITING_FOR_IP_HIGH, WAITING_FOR_IP_LOW, WAITING_FOR_IP_LOWEST,
	WAITING_FOR_SPEED_HIGH, WAITING_FOR_SPEED_LOW, WAITING_FOR_DISPLAY_DATA} current_state = IDLE;

// 0 motsvarar 1 ms
// 255 motsvarar 2 ms
void set_steering_pulse_width(uint8_t pulse_width)
{
	// OCR1A = 1000000 * 0.019 - pulse_width * (1000000 * 0.001 / 255);
	OCR1A = 1000000 * 0.019 - pulse_width * 4;
	//TCNT1 = 0;
}

// 0 motsvarar 1 ms		(full gas framåt?)
// 255 motsvarar 2 ms	(full gas bakåt?)
void set_speed_pulse_width(uint8_t pulse_width)
{
	OCR1B = 1000000 * 0.019 - pulse_width * 4;
}

void send_command_to_lcd(uint8_t command)
{
	LCD_Port = (LCD_Port & 0xF0) | ((command >> 4) & 0xF); /* sending upper nibble */
	LCD_Port &= ~(1 << RS);				/* RS=0, command reg. */
	LCD_Port |= (1 << EN);				/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~(1 << EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (command & 0xF);  /* sending lower nibble */
	LCD_Port |= (1 << EN);
	_delay_us(1);
	LCD_Port &= ~(1 << EN);
	_delay_ms(2);
}


void send_data_to_lcd(uint8_t data)
{
	LCD_Port = (LCD_Port & 0xF0) | ((data >> 4) & 0xF); /* sending upper nibble */
	LCD_Port |= (1<<RS);				/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (data & 0xF); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void init_lcd(void)					/* LCD Initialize function */
{
	LCD_Dir = 0xFF;						/* Make LCD command port direction as o/p */
	_delay_ms(20);						/* LCD Power ON delay always >15ms */
	
	send_command_to_lcd(0x33);
	send_command_to_lcd(0x32);		    		/* send for 4 bit initialization of LCD  */
	send_command_to_lcd(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	send_command_to_lcd(0x0c);              	/* Display on cursor off*/
	send_command_to_lcd(0x06);              	/* Increment cursor (shift cursor to right)*/
	send_command_to_lcd(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
}


void LCD_String (char *str)				/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)				/* Send each char of string till the NULL */
	{
		send_data_to_lcd(str[i]);
	}
}

void send_string_16(char* str)
{
	for (uint8_t  i = 0; i < 16; i++)
	{
		send_data_to_lcd(str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	send_command_to_lcd((pos & 0x0F)|0x80);		/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	send_command_to_lcd((pos & 0x0F)|0xC0);		/* Command of first row and required position<16 */
	LCD_String(str);					/* Call LCD string function */
}

void LCD_Clear()
{
	send_command_to_lcd (0x01);					/* Clear display */
	_delay_ms(2);
	send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
}

int main(void)
{
	/*TCCR1A = (1 << COM1A0) | (1 << COM1B0);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	DDRD = (1 << 5);
	OCR1A = 1000000 * 0.0185 - 1;
	ICR1 = 1000000 * 0.0200 - 1;		// 20 ms för 1 MHz
	TCNT1 = 0;*/
	
	// FastPWM som räknar upp till ICR1
	// Prescaler 1 (ingen delning)
	DDRD = (1 << 5) | (1 << 4);
	TCNT1 = 0;
	ICR1 = 19999;// 1000000 * 0.0200 - 1;		// 20 ms för 1 MHz
	TCCR1A = (1 << COM1A1)	| (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11);
	TCCR1B = (1 << WGM12)	| (1 << WGM13)	| (1 << CS10);
	set_steering_pulse_width(127);
	set_speed_pulse_width(127);
	
	// Starta SPI-slav
	//Set MISO output, all others input
	DDRB = (1 << DDB6);
	//Konfigurerar AVR:n till slav, sätter igång SPI etc.
	// SPI-klocka F_osc / 16 för att ge AVR tid att ändra SPDR
	SPCR = (1 << SPIE) | (1 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR0);
	
	sei();
	
	init_lcd();
	
	/*_delay_ms(10000);
	sprintf(line1, "Smaskig paprika");
	send_string_16(line1);
	set_speed_pulse_width(140);
	_delay_ms(1000);
	set_speed_pulse_width(127);
	sprintf(line1, "Smaskig paprika");
	send_string_16(line1);
	_delay_ms(1000);
	set_speed_pulse_width(110);
	_delay_ms(1000);
	set_speed_pulse_width(127);
	while (1)
	{
		;
	}*/
	
    /* Replace with your application code */
	//uint8_t sin_table_index = 0;
    while (1) 
    {		
		memset(line1, ' ', 16);
		memset(line2, ' ', 16);
		memset(line3, ' ', 16);
		//memset(line4, ' ', 16);
		uint8_t hax = sprintf(line1, "Styrvinkel:%d", actual_steering_angle); /*Save steering angle in a char buffer*/
		if (hax != 16) line1[hax] = ' ';
		hax = sprintf(line2, "Fart:%d", speed); /*Save speed in a char buffer*/
		if (hax != 16) line2[hax] = ' ';
		hax = sprintf(line3, "%d.%d.%d.%d", ip_bytes[3],ip_bytes[2],ip_bytes[1],ip_bytes[0]);
		if (hax != 16) line3[hax] = ' ';
		//LCD_Clear(); /*Clear display*/
		send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
		send_string_16(line1);	/* Write string on 1st line of LCD*/
		send_string_16(line3); /*Write string on 3rd line on LCD*/
		send_command_to_lcd(0xc0);			/* Go to 2nd line*/
		send_string_16(line2); /*Write string on 2nd line of LCD*/
		send_string_16(line4);
		_delay_ms(17); /*Delay for more time to write on LCD*/
		
		if (update_angle)
		{
			update_angle = 0;
			if (steering_angle < -1850)
				steering_angle = -1850;
			if (steering_angle > 1850)
				steering_angle = 1850;
				
			actual_steering_angle = steering_angle;
				
			uint8_t p = steering_angle * 0.04703703703 + 127; // * 127 / (45 * 60)
			set_steering_pulse_width(p);
		}
		if (update_speed)
		{
			update_speed = 0;
			if (speed > 5000)
				speed = 5000;
			if (speed < -5000)
				speed = -5000;
				
			uint8_t s = speed / 40;
			set_speed_pulse_width(127 + s);	
		}
    }
}

ISR(SPI_STC_vect)
{
	if (current_state == IDLE && SPDR == 5)
	{
		// Styrvinkel kommer härnäst
		current_state = WAITING_FOR_STEERING_ANGLE_LOW;
		//SPDR = 50;
	}
	else if (current_state == IDLE && SPDR == 13)
	{
		current_state = WAITING_FOR_STEERING_ANGLE_HIGH;
		//SPDR = 51;
	}
	else if (current_state == IDLE && SPDR == 6)
	{
		// Fyra lägsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_LOWEST;
		//SPDR = 55;
	}
	else if (current_state == IDLE && SPDR == 7)
	{
		// Fyra näst lägsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_LOW;
		//SPDR = 56;
	}
	else if (current_state == IDLE && SPDR == 8)
	{
		// Fyra näst högsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_HIGH;
		//SPDR = 57;
	}
	else if (current_state == IDLE && SPDR == 9)
	{
		// Fyra högsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_HIGHEST;
		//SPDR = 58;
	}
	else if (current_state == IDLE && SPDR == 10)
	{
		current_state = WAITING_FOR_SPEED_LOW;
	}
	else if (current_state == IDLE && SPDR == 11)
	{
		current_state = WAITING_FOR_SPEED_HIGH;
	}
	else if (current_state == IDLE && SPDR == 12)
	{
		current_state = WAITING_FOR_DISPLAY_DATA;
	}
	else if (current_state == WAITING_FOR_STEERING_ANGLE_LOW)
	{
		// Nu kom den låga delen av styrvinkeln
		steering_angle = SPDR;
		current_state = IDLE;
		update_angle = 0;
		//SPDR = 52;
	}
	else if (current_state == WAITING_FOR_STEERING_ANGLE_HIGH)
	{
		// Nu kom den höga delan av styrvinkeln
		steering_angle = (SPDR << 8) | steering_angle;
		current_state = IDLE;
		// Uppdatera pulser
		update_angle = 1;
		//SPDR = 53;
	}
	else if (current_state == WAITING_FOR_IP_LOWEST)
	{
		// Nu kom den lägsta delen av IP-adressen
		ip_bytes[0] = SPDR;
		current_state = IDLE;
		//SPDR = 59;
	}
	else if (current_state == WAITING_FOR_IP_LOW)
	{
		// Nu kom den näst lägsta delen av IP-adressen
		ip_bytes[1] = SPDR ;
		current_state = IDLE;
		//SPDR = 60;
	}
	else if (current_state == WAITING_FOR_IP_HIGH)
	{
		// Nu kom den näst högsta delen av IP-adressen
		ip_bytes[2] = SPDR;
		current_state = IDLE;
		//SPDR = 61;
	}
	else if (current_state == WAITING_FOR_IP_HIGHEST)
	{
		// Nu kom den högsta delen av IP-adressen
		ip_bytes[3] = SPDR;
		current_state = IDLE;
		//SPDR = 62;
	}
	else if (current_state == WAITING_FOR_SPEED_LOW)
	{
		speed = SPDR;
		current_state = IDLE;
		update_speed = 0;
	}
	else if (current_state == WAITING_FOR_SPEED_HIGH)
	{
		speed = (SPDR << 8) | speed;
		current_state = IDLE;
		update_speed = 1;
	}
	else if (current_state == WAITING_FOR_DISPLAY_DATA)
	{
		line4[line4_index_from_pi++] = SPDR;
		current_state = WAITING_FOR_DISPLAY_DATA;
		if (line4_index_from_pi == 16)
		{
			line4_index_from_pi = 0;
			current_state = IDLE;
		}
	}
	else
	{
				//			SPDR = 54;	
	}
}
=======
/*
 * Test.c
 *
 * Created: 2020-04-03 10:47:55
 * Author : Gustav
 */ 

// AVR2 (styr)

// MAX 1850 styrvinkel

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>

// https://www.electronicwings.com/avr-atmega/interfacing-lcd-16x2-in-4-bit-mode-with-atmega-16-32-

#define F_CPU 1000000UL					/* Define CPU Frequency e.g. here its 8MHz */
#include <util/delay.h>					/* Include inbuilt defined Delay header file */

#define LCD_Dir DDRA					/* Define LCD data port direction */
#define LCD_Port PORTA					/* Define LCD data port */
#define RS PA4							/* Define Register Select (data reg./command reg.) signal pin */
#define EN PA5 							/* Define Enable signal pin */

char line1[16];
char line2[16];
char line3[16];
char line4[16]; // På rad 4 kan hallonpajen skriva vad den vill
volatile int line4_index_from_pi = 0;

volatile uint8_t ip_bytes[4];

volatile uint8_t paprika = 42;
volatile int16_t steering_angle = 0;
uint16_t actual_steering_angle = 0;
volatile int16_t speed = 0;

volatile uint8_t update_angle = 0;
volatile uint8_t update_speed = 0;

volatile enum state {IDLE, WAITING_FOR_STEERING_ANGLE_HIGH, WAITING_FOR_STEERING_ANGLE_LOW, 
	WAITING_FOR_IP_HIGHEST, WAITING_FOR_IP_HIGH, WAITING_FOR_IP_LOW, WAITING_FOR_IP_LOWEST,
	WAITING_FOR_SPEED_HIGH, WAITING_FOR_SPEED_LOW, WAITING_FOR_DISPLAY_DATA} current_state = IDLE;

// 0 motsvarar 1 ms
// 255 motsvarar 2 ms
void set_steering_pulse_width(uint8_t pulse_width)
{
	// OCR1A = 1000000 * 0.019 - pulse_width * (1000000 * 0.001 / 255);
	OCR1A = 1000000 * 0.019 - pulse_width * 4;
	//TCNT1 = 0;
}

// 0 motsvarar 1 ms		(full gas framåt?)
// 255 motsvarar 2 ms	(full gas bakåt?)
void set_speed_pulse_width(uint8_t pulse_width)
{
	OCR1B = 1000000 * 0.019 - pulse_width * 4;
}

void send_command_to_lcd(uint8_t command)
{
	LCD_Port = (LCD_Port & 0xF0) | ((command >> 4) & 0xF); /* sending upper nibble */
	LCD_Port &= ~(1 << RS);				/* RS=0, command reg. */
	LCD_Port |= (1 << EN);				/* Enable pulse */
	_delay_us(1);
	LCD_Port &= ~(1 << EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (command & 0xF);  /* sending lower nibble */
	LCD_Port |= (1 << EN);
	_delay_us(1);
	LCD_Port &= ~(1 << EN);
	_delay_ms(2);
}


void send_data_to_lcd(uint8_t data)
{
	LCD_Port = (LCD_Port & 0xF0) | ((data >> 4) & 0xF); /* sending upper nibble */
	LCD_Port |= (1<<RS);				/* RS=1, data reg. */
	LCD_Port|= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);

	_delay_us(200);

	LCD_Port = (LCD_Port & 0xF0) | (data & 0xF); /* sending lower nibble */
	LCD_Port |= (1<<EN);
	_delay_us(1);
	LCD_Port &= ~ (1<<EN);
	_delay_ms(2);
}

void init_lcd(void)					/* LCD Initialize function */
{
	LCD_Dir = 0xFF;						/* Make LCD command port direction as o/p */
	_delay_ms(20);						/* LCD Power ON delay always >15ms */
	
	send_command_to_lcd(0x33);
	send_command_to_lcd(0x32);		    		/* send for 4 bit initialization of LCD  */
	send_command_to_lcd(0x28);              	/* Use 2 line and initialize 5*7 matrix in (4-bit mode)*/
	send_command_to_lcd(0x0c);              	/* Display on cursor off*/
	send_command_to_lcd(0x06);              	/* Increment cursor (shift cursor to right)*/
	send_command_to_lcd(0x01);              	/* Clear display screen*/
	_delay_ms(2);
	send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
}


void LCD_String (char *str)				/* Send string to LCD function */
{
	int i;
	for(i=0;str[i]!=0;i++)				/* Send each char of string till the NULL */
	{
		send_data_to_lcd(str[i]);
	}
}

void send_string_16(char* str)
{
	for (uint8_t  i = 0; i < 16; i++)
	{
		send_data_to_lcd(str[i]);
	}
}

void LCD_String_xy (char row, char pos, char *str)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	send_command_to_lcd((pos & 0x0F)|0x80);		/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	send_command_to_lcd((pos & 0x0F)|0xC0);		/* Command of first row and required position<16 */
	LCD_String(str);					/* Call LCD string function */
}

void LCD_Clear()
{
	send_command_to_lcd (0x01);					/* Clear display */
	_delay_ms(2);
	send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
}

int main(void)
{
	/*TCCR1A = (1 << COM1A0) | (1 << COM1B0);
	TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS10);
	DDRD = (1 << 5);
	OCR1A = 1000000 * 0.0185 - 1;
	ICR1 = 1000000 * 0.0200 - 1;		// 20 ms för 1 MHz
	TCNT1 = 0;*/
	
	// FastPWM som räknar upp till ICR1
	// Prescaler 1 (ingen delning)
	DDRD = (1 << 5) | (1 << 4);
	TCNT1 = 0;
	ICR1 = 19999;// 1000000 * 0.0200 - 1;		// 20 ms för 1 MHz
	TCCR1A = (1 << COM1A1)	| (1 << COM1A0) | (1 << COM1B1) | (1 << COM1B0) | (1 << WGM11);
	TCCR1B = (1 << WGM12)	| (1 << WGM13)	| (1 << CS10);
	set_steering_pulse_width(127);
	set_speed_pulse_width(127);
	
	// Starta SPI-slav
	//Set MISO output, all others input
	DDRB = (1 << DDB6);
	//Konfigurerar AVR:n till slav, sätter igång SPI etc.
	// SPI-klocka F_osc / 16 för att ge AVR tid att ändra SPDR
	SPCR = (1 << SPIE) | (1 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR0);
	
	sei();
	
	init_lcd();
	
	/*_delay_ms(10000);
	sprintf(line1, "Smaskig paprika");
	send_string_16(line1);
	set_speed_pulse_width(140);
	_delay_ms(1000);
	set_speed_pulse_width(127);
	sprintf(line1, "Smaskig paprika");
	send_string_16(line1);
	_delay_ms(1000);
	set_speed_pulse_width(110);
	_delay_ms(1000);
	set_speed_pulse_width(127);
	while (1)
	{
		;
	}*/
	
    /* Replace with your application code */
	//uint8_t sin_table_index = 0;
    while (1) 
    {		
		memset(line1, ' ', 16);
		memset(line2, ' ', 16);
		memset(line3, ' ', 16);
		//memset(line4, ' ', 16);
		uint8_t hax = sprintf(line1, "Styrvinkel:%d", actual_steering_angle); /*Save steering angle in a char buffer*/
		if (hax != 16) line1[hax] = ' ';
		hax = sprintf(line2, "Fart:%d", speed); /*Save speed in a char buffer*/
		if (hax != 16) line2[hax] = ' ';
		hax = sprintf(line3, "%d.%d.%d.%d", ip_bytes[3],ip_bytes[2],ip_bytes[1],ip_bytes[0]);
		if (hax != 16) line3[hax] = ' ';
		//LCD_Clear(); /*Clear display*/
		send_command_to_lcd (0x80);					/* Cursor 1st row 0th position */
		send_string_16(line1);	/* Write string on 1st line of LCD*/
		send_string_16(line3); /*Write string on 3rd line on LCD*/
		send_command_to_lcd(0xc0);			/* Go to 2nd line*/
		send_string_16(line2); /*Write string on 2nd line of LCD*/
		send_string_16(line4);
		_delay_ms(17); /*Delay for more time to write on LCD*/
		
		if (update_angle)
		{
			update_angle = 0;
			if (steering_angle < -1850)
				steering_angle = -1850;
			if (steering_angle > 1850)
				steering_angle = 1850;
				
			actual_steering_angle = steering_angle;
				
			uint8_t p = steering_angle * 0.04703703703 + 127; // * 127 / (45 * 60)
			set_steering_pulse_width(p);
		}
		if (update_speed)
		{
			update_speed = 0;
			if (speed > 5000)
				speed = 5000;
			if (speed < -5000)
				speed = -5000;
				
			uint8_t s = speed / 40;
			set_speed_pulse_width(127 + s);	
		}
    }
}

ISR(SPI_STC_vect)
{
	if (current_state == IDLE && SPDR == 5)
	{
		// Styrvinkel kommer härnäst
		current_state = WAITING_FOR_STEERING_ANGLE_LOW;
		//SPDR = 50;
	}
	else if (current_state == IDLE && SPDR == 13)
	{
		current_state = WAITING_FOR_STEERING_ANGLE_HIGH;
		//SPDR = 51;
	}
	else if (current_state == IDLE && SPDR == 6)
	{
		// Fyra lägsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_LOWEST;
		//SPDR = 55;
	}
	else if (current_state == IDLE && SPDR == 7)
	{
		// Fyra näst lägsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_LOW;
		//SPDR = 56;
	}
	else if (current_state == IDLE && SPDR == 8)
	{
		// Fyra näst högsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_HIGH;
		//SPDR = 57;
	}
	else if (current_state == IDLE && SPDR == 9)
	{
		// Fyra högsta bitarna av IP kommer härnäst
		current_state = WAITING_FOR_IP_HIGHEST;
		//SPDR = 58;
	}
	else if (current_state == IDLE && SPDR == 10)
	{
		current_state = WAITING_FOR_SPEED_LOW;
	}
	else if (current_state == IDLE && SPDR == 11)
	{
		current_state = WAITING_FOR_SPEED_HIGH;
	}
	else if (current_state == IDLE && SPDR == 12)
	{
		current_state = WAITING_FOR_DISPLAY_DATA;
	}
	else if (current_state == WAITING_FOR_STEERING_ANGLE_LOW)
	{
		// Nu kom den låga delen av styrvinkeln
		steering_angle = SPDR;
		current_state = IDLE;
		update_angle = 0;
		//SPDR = 52;
	}
	else if (current_state == WAITING_FOR_STEERING_ANGLE_HIGH)
	{
		// Nu kom den höga delan av styrvinkeln
		steering_angle = (SPDR << 8) | steering_angle;
		current_state = IDLE;
		// Uppdatera pulser
		update_angle = 1;
		//SPDR = 53;
	}
	else if (current_state == WAITING_FOR_IP_LOWEST)
	{
		// Nu kom den lägsta delen av IP-adressen
		ip_bytes[0] = SPDR;
		current_state = IDLE;
		//SPDR = 59;
	}
	else if (current_state == WAITING_FOR_IP_LOW)
	{
		// Nu kom den näst lägsta delen av IP-adressen
		ip_bytes[1] = SPDR ;
		current_state = IDLE;
		//SPDR = 60;
	}
	else if (current_state == WAITING_FOR_IP_HIGH)
	{
		// Nu kom den näst högsta delen av IP-adressen
		ip_bytes[2] = SPDR;
		current_state = IDLE;
		//SPDR = 61;
	}
	else if (current_state == WAITING_FOR_IP_HIGHEST)
	{
		// Nu kom den högsta delen av IP-adressen
		ip_bytes[3] = SPDR;
		current_state = IDLE;
		//SPDR = 62;
	}
	else if (current_state == WAITING_FOR_SPEED_LOW)
	{
		speed = SPDR;
		current_state = IDLE;
		update_speed = 0;
	}
	else if (current_state == WAITING_FOR_SPEED_HIGH)
	{
		speed = (SPDR << 8) | speed;
		current_state = IDLE;
		update_speed = 1;
	}
	else if (current_state == WAITING_FOR_DISPLAY_DATA)
	{
		line4[line4_index_from_pi++] = SPDR;
		current_state = WAITING_FOR_DISPLAY_DATA;
		if (line4_index_from_pi == 16)
		{
			line4_index_from_pi = 0;
			current_state = IDLE;
		}
	}
	else
	{
				//			SPDR = 54;	
	}
}
>>>>>>> c3c616ba5f3678cf7172979354acfe96c56a70a0
