#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>


#define SPI_BUFF_SIZE 32 //Definierar storleken på bufferten (kan ändras)

volatile uint8_t SPI_buffer[SPI_BUFF_SIZE]; //Skapar en buffert
volatile uint8_t SPI_buff_ptr = 0; //Skapar en buffertpekare
volatile uint8_t SPI_buff_len = 0; //Skapar en variabel som säger hur många element som finns i bufferten

int main()
{
    //SPI för slav AVR1
    //Set MISO output, all others input
    DDRB = (1<<DDB6);
    //Konfigurerar AVR:n till slav, sätter igång SPI etc.
    SPCR = (1<<SPIE) | (1<<SPE) | (0<<DORD) | (0<<MSTR) | (0<<CPOL) | (0<<CPHA);
    sei(); //Aktiverar avbrott
    while(1)
    {}
}

ISR(SPISTC_vect)
{
    if (SPI_buff_ptr < SPI_buff_len - 1){ //Kollar om bufferten är tom
    SPDR = SPI_buffer[SPI_buff_ptr]; //Lägger ut datan på bussen
    SPI_buff_ptr++; //Ökar värdet på buffertpekaren
    };
}
