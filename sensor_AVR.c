
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>

// PRogram för AVR1 (sensormodulen)

// ****************************************************************************
// *							SPI-saker									  *
// ****************************************************************************
#define SPI_BUFF_SIZE 32	//Definierar storleken på bufferten (kan ändras)

volatile uint8_t SPI_buffer[SPI_BUFF_SIZE]; //Skapar en buffert
volatile uint8_t SPI_buff_ptr = 0; //Skapar en buffertpekare
volatile uint8_t SPI_buff_len = SPI_BUFF_SIZE; //Skapar en variabel som säger hur många element som finns i bufferten

// ****************************************************************************
// *							Hastighets-saker							  *
// ****************************************************************************

volatile int wheel_diameter = 80;
volatile uint16_t vel_r = 0;
volatile uint8_t update_r = 0;
uint16_t clicks_r[256];
uint8_t alive_r[256];
volatile uint8_t last_click_r = 0;
uint8_t oldest_click_r = 0;
uint16_t total_clicks_r = 0;
float time_r = 0;
float distance_r = 0;
uint16_t paprika_r_t = 0;
uint8_t paprika_r_d = 0;
//volatile uint16_t vel_l = 0;
volatile uint8_t update_l = 0;
//uint16_t clicks_l[256];
//uint8_t alive_l[256];
//volatile uint8_t last_click_l = 0;
//uint8_t oldest_click_l = 0;
//uint16_t total_clicks_l = 0;
uint16_t vel = 0;
uint16_t total_clicks = 0;
float time_l = 0;
float distance_l = 0;
volatile uint8_t temp_half_time = 0;
volatile uint8_t temp_half_clicks = 0;

int main(void)
{
	// Starta SPI
	// Set MISO output, all others input
	DDRB = (1 << DDB6);
	// Konfigurerar AVR:n till slav, sätter igång SPI etc.
	// SPI-klocka = klocka / 64
	SPCR = (1 << SPIE) | (1 << SPE) | (0 << DORD) | (0 << MSTR) | (0 << CPOL) | (0 << CPHA) | (1 << SPR1) | (1 << SPR0);
	SPSR = 0;
	
	// Starta timer1 och ge oss en spark då timern flödar över
	GICR   = (1 << INT1) | (1 << INT0);
	MCUCR  = (1 << ISC10) | (1 << ISC11) | (1 << ISC00) | (1 << ISC01);
	TCCR1B = (1 << CS10) | (1 << CS11);
	
	sei();
	while (1)
	{
		if(TCNT1 - clicks_r[oldest_click_r] > 30000 && alive_r[oldest_click_r])
		{
			clicks_r[oldest_click_r] = 0;
			alive_r[oldest_click_r] = 0;
			oldest_click_r++;	
		}
		
		uint16_t temp_paprika_r_t = clicks_r[last_click_r] - clicks_r[oldest_click_r];
		uint8_t temp_paprika_r_d = last_click_r -oldest_click_r;
		//time_r =  paprika_r / 125000.0; //* 8 / 1000000.0;
		//distance_r = wheel_diameter * 0.314 * (last_click_r - oldest_click_r);
		//uint16_t  temp = (uint16_t )(distance_r /time_r);
		cli();
			paprika_r_t = temp_paprika_r_t;
			paprika_r_d = temp_paprika_r_d;
		sei();
		
		/*if(TCNT1 - clicks_l[oldest_click_l] > 60000 && alive_l[oldest_click_l])
		{
			uint16_t paprika_l;
			clicks_l[oldest_click_l] = 0;
			alive_l[oldest_click_l] = 0;
			oldest_click_l++;	
			paprika_l = clicks_l[last_click_l - 1] - clicks_l[oldest_click_l];
			time_l =  paprika_l / 125000.0; //* 8 / 1000000.0;
			distance_l = wheel_diameter * 0.314 * (last_click_l - 1 - oldest_click_l);
			uint16_t  temp = (uint16_t )(distance_l /time_l);
			cli();
			vel_l = temp;
			sei();
		}*/
		if (update_r == 1)
		{
			update_r = 0;
			last_click_r++;
			total_clicks_r++;
			clicks_r[last_click_r] = TCNT1;
			alive_r[last_click_r] = 1;
			
			/*tim1_counter++;
			uint16_t temp = (uint16_t)(wheel_diameter * 0.314 * r_wheel_counter / (tim1_counter * 0.524288));
			cli();
			vel = temp;
			sei();
			SPI_buffer[0] = vel >> 8;
			SPI_buffer[1] = vel & 0xff;
			SPI_buff_len = 2;
			SPI_buff_ptr = 0;
		if(tim1_counter > 2)//&& vel < 60)
			{
				tim1_counter = 0;
				r_wheel_counter = 0;
			}
			if(vel > 60 && tim1_counter > 15)
			{
				tim1_counter = 0;
				r_wheel_counter = 0;
			}*/
		}
	/*	if (update_l == 1)
		{
			update_l = 0;
			clicks_l[last_click_l] = TCNT1;
			alive_l[last_click_l] = 1;
			time_l =  (clicks_l[last_click_l] - clicks_l[oldest_click_l]) / 125000.0;
			distance_l = wheel_diameter * 0.314 * (last_click_l - oldest_click_l);
			uint16_t  temp = (uint16_t )(distance_l /time_l);
			total_clicks_l++;
			last_click_l++;
			cli();
			vel_l = temp;
			sei();
			/*tim1_counter++;
			uint16_t temp = (uint16_t)(wheel_diameter * 0.314 * r_wheel_counter / (tim1_counter * 0.524288));
			cli();
			vel = temp;
			sei();
			SPI_buffer[0] = vel >> 8;
			SPI_buffer[1] = vel & 0xff;
			SPI_buff_len = 2;
			SPI_buff_ptr = 0;
		if(tim1_counter > 2)//&& vel < 60)
			{
				tim1_counter = 0;
				r_wheel_counter = 0;
			}
			if(vel > 60 && tim1_counter > 15)
			{
				tim1_counter = 0;
				r_wheel_counter = 0;
			}
		}*/
		//vel = vel_r; // + vel_l / 2 ;
		//total_clicks = total_clicks_r; // + total_clicks_l / 2;
	}
}

ISR(INT1_vect)
{
	update_r = 1;
}

ISR(INT0_vect)
{
	update_r = 1;
}

ISR(SPI_STC_vect)
{		
		if (SPDR == 17)
		{	
			SPDR = paprika_r_t & 0xff;
			temp_half_time = paprika_r_t >> 8;
		}
		else if (SPDR == 42)
			SPDR = temp_half_time;
		else if (SPDR == 43)
		{
			SPDR = paprika_r_d;
		}
		else if (SPDR == 69)
		{
			SPDR = total_clicks_r & 0xff;
			temp_half_clicks = total_clicks_r >> 8;
		}
		else if (SPDR == 22)
			SPDR = temp_half_clicks;
		else
			SPDR = 70;
}



































/*
int main()
{
	//SPI för slav AVR1
	//Set MISO output, all others input
	DDRB = (1<<DDB6);
	//Konfigurerar AVR:n till slav, sätter igång SPI etc.
	SPCR = (1<<SPIE) | (1<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1 << SPR0);
	SPSR = 0;//(1 << SPI2X);
	sei(); //Aktiverar avbrott
	
	for (uint8_t i = 0; i < SPI_BUFF_SIZE; i++)
	{
		SPI_buffer[i] = 69 + i;
	}
	
	while(1)
	{}
}

ISR(SPI_STC_vect)
{
	//uint8_t mottagen_byte = SPDR;
	
	if (SPI_buff_ptr < SPI_buff_len - 1){ //Kollar om bufferten är tom
		SPDR = SPI_buffer[SPI_buff_ptr]; //Lägger ut datan på bussen
		SPI_buff_ptr++;
	}
	else
	{
		SPDR = 0;
	}
}
*/