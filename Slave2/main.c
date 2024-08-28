//C�digo Slave 2


#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>

#include "I2C/I2C.h"
#include "LCD/LCD.h"

#define SlaveAddress 0x40

#define TRIG PD3
#define ECHO PD2
#define BUZZER PD4

volatile uint16_t ultrasonico = 0;

void init_ultrasonic(void)
{
	cli(); // Deshabilitar interrupciones globales

	// Configurar TRIG como salida
	DDRD |= (1 << TRIG);
	
	// Configurar ECHO como entrada
	DDRD &= ~(1 << ECHO);
	
	// Configurar BUZZER como salida
	DDRD |= (1 << BUZZER);
	
	// Asegurarse de que el buzzer est� apagado inicialmente
	PORTD &= ~(1 << BUZZER);

	// Inicializaci�n del I2C
	I2C_Slave_Init(SlaveAddress);
	
	sei(); // Habilitar interrupciones globales
}



uint16_t read_distance(void) {
	uint16_t count = 0;
	
	// Enviar un pulso de 10us en el pin TRIG
	PORTD &= ~(1 << TRIG);
	_delay_us(2);
	PORTD |= (1 << TRIG);
	_delay_us(10);
	PORTD &= ~(1 << TRIG);

	// Esperar a que el pin ECHO se ponga en alto
	while (!(PIND & (1 << ECHO)));

	// Contar el tiempo que el pin ECHO se mantiene en alto
	while (PIND & (1 << ECHO)) {
		count++;
		_delay_us(1);
	}

	// Convertir el tiempo a distancia en cm
	return (count / 58);
}

int main(void) {
	init_ultrasonic();

	while (1) {
		ultrasonico = read_distance();

		if (ultrasonico < 10) {  // Si el objeto est� a menos de 10 cm
			PORTD |= (1 << BUZZER);  // Encender el buzzer
			_delay_ms(35);
			PORTD &= ~(1 << BUZZER);  // Apagar el buzzer
			_delay_ms(35);
			PORTD &= ~(1 << BUZZER);
			_delay_ms(35);
			} else {
			PORTD &= ~(1 << BUZZER);  // Apagar el buzzer
			_delay_ms(35); // Encender el buzzer
			
		}

		_delay_ms(200);  // Peque�o retardo antes de la siguiente medici�n
	}
}


ISR(TWI_vect)
{
	uint8_t estado = TWSR & 0xF8;
	switch (estado) {
		case 0x60:  // SLA+W recibido, ACK enviado
		case 0x70:  // Direcci�n de transmisi�n general recibida, ACK enviado
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para recibir m�s datos
		break;
		case 0x80:  // Dato recibido, ACK enviado
		case 0x90:  // Dato recibido en modo transmisi�n general, ACK enviado
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para recibir m�s datos
		break;
		case 0xA8:  // SLA+R recibido, ACK enviado
		case 0xB8:  // Dato transmitido, ACK recibido
		TWDR = ultrasonico;  // Enviar valor del contador
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para la siguiente transmisi�n
		break;
		case 0xC0:  // Dato transmitido, NACK recibido
		case 0xC8:  // �ltimo dato transmitido, ACK recibido
		TWCR |= (1<<TWINT) | (1<<TWEA);  // Preparar para la pr�xima recepci�n
		break;
		default:  // Se libera el bus de cualquier error
		TWCR |= (1<<TWINT) | (1<<TWSTO);
	}
}