/*
 * Slav1_Proyecto.c
 *
 * Created: 9/08/2024 11:45:17
 * Author : luisa
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

#include "I2C/I2C.h"
#include "ADC/ADC.h"

#define SlaveAddress 0x30

uint8_t buffer;

#define MIN_ADC_VALUE 69    // Valor mínimo observado cuando se usa ADCH
#define MAX_ADC_VALUE 169   // Valor máximo observado cuando se usa ADCH
#define LOW_THRESHOLD 100   // Ajusta según el comportamiento deseado

volatile uint8_t POT1 = 0;

uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);



int main(void) {
	cli();
	DDRD |= (1<<DDD2);
	PORTD &= ~(1<<PORTD2);
	
	I2C_Slave_Init(SlaveAddress);
	SetupADC(PC6);
	sei();

	while(1) {
		ADCSRA |= (1<<ADSC);
		_delay_ms(100);
		buffer = POT1; 
		/*uint8_t mappedValue = map(POT1, MIN_ADC_VALUE, MAX_ADC_VALUE, 0, 255);
		if (POT1 < LOW_THRESHOLD) {
			PORTD |= (1<<PORTD2);  // Enciende el LED
			buffer = 'a';
			//buffer = mappedValue; 
			} else{
			PORTD &= ~(1<<PORTD2); // Apaga el LED
			buffer = 'b';
		}*/
	}
}


uint8_t map(uint8_t x, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
	return (uint8_t)((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min);
}

ISR(TWI_vect){
	uint8_t estado = TWSR & 0xF8;
	switch(estado){
		case 0x60:  // SLA+W recibido, ACK enviado
		case 0x70:  // Dirección de transmisión general recibida, ACK enviado
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para recibir más datos
		break;
		case 0x80:  // Dato recibido, ACK enviado
		case 0x90:  // Dato recibido en modo transmisión general, ACK enviado
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para recibir más datos
		break;
		case 0xA8:  // SLA+R recibido, ACK enviado
		case 0xB8:  // Dato transmitido, ACK recibido
		
		TWDR = buffer;  // Enviar señal de cambio de estado
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para la siguiente transmisión
		break;
		case 0xC0:  // Dato transmitido, NACK recibido
		case 0xC8:  // Último dato transmitido, ACK recibido
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para la próxima recepción
		break;
		default:  // Se libera el bus de cualquier error
		TWCR |= (1<<TWINT) | (1<<TWSTO);
	}
}


ISR(ADC_vect){
	//POT1 = ADCL;
	POT1 = ADCH;
	
	//POT1 = (ADCL | (ADCH << 8));
	
	ADCSRA |= (1<<ADIF);
}