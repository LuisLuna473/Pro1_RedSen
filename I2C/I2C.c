/*
 * I2C.c
 *
 * Created: 1/08/2024 16:16:45
 *  Author: luisa
 */ 
#include "I2C.h"

//Función para inicializar I2C Maestro
void I2C_Master_Init(unsigned SCL_Clock, uint8_t Prescaler){
	DDRC &= ~((1<<DDC4)|(1<<DDC5)); 
	switch(Prescaler){
		case 1:
			TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
		break; 
		
		case 4:
			TWSR &= ~(1<<TWPS1);
			TWSR |= (1<<TWPS0);
		break; 
		case 16:
			TWSR &= ~(1<<TWPS0);
			TWSR |= (1<<TWPS1);
		break;
		
		case 64:
			TWSR |= (1<<TWPS1)|(1<<TWPS0);
		break;
		
		default:
			TWSR &= ~((1<<TWPS1)|(1<<TWPS0));
			Prescaler = 1;
		break;
	}
	TWBR = ((F_CPU/SCL_Clock)-16)/(2*Prescaler);
	TWCR |= (1<<TWEN);
}

//Función para inicio de la comunicación I2C Maestro 
uint8_t I2C_Master_Start(void){
	uint8_t estado;
	TWCR = (1<< TWINT)|(1<<TWSTA)|(1<<TWEN); //Iniciar condiciones de start
	while(!(TWCR &(1<<TWINT))); //Espera a que termine la flag TWINT
	
	estado = TWSR & 0xF8; //Verifica estado
	if((estado != 0x08) || (estado != 0x10)){
		return 1; 
	}else{
		return estado;
	}
}

//Función de parada I2C Maestro
void I2C_Master_Stop(void){
	TWCR = (1<<TWEN)|(1<<TWINT)|(1<<TWSTO); //Inicia el envio secuencia de parada
	while(TWCR & (1<<TWSTO)); //Esperamos a que el bit se limpie
}

//Maestro escribe
uint8_t I2C_Master_Write(uint8_t dato){
	uint8_t estado; 
	TWDR = dato; //Cargar dato
	TWCR = (1<<TWEN)|(1<<TWINT); //inicia el envio
	
	while(!(TWCR & (1<<TWINT))); //Espera al flag TWINT
	estado = TWSR & 0xF8;
	//Verificar si se transmitio una SLA +W con ACK, SLA +R con ACK, o un Dato 
	if(estado == 0x18 || estado == 0x28 || estado == 0x40){
		return 1; 
	}else{
		return estado;
	}
}

// Escribe un byte en el bus I2C
uint8_t I2C_Master_Write_SEN(uint8_t dato) {
	TWDR = dato; // Carga el dato en el registro de datos
	TWCR = (1 << TWINT) | (1 << TWEN); // Inicia la transmisión
	while (!(TWCR & (1 << TWINT))); // Espera a que la transmisión se complete

	uint8_t estado = TWSR & 0xF8; // Lee el estado del TWI
	if (estado != 0x18 && estado != 0x28 && estado != 0x40) {
		return estado; // Error
	}
	return 0; // Éxito
}

//Maestro escribe a 16 bits
/*uint16_t I2C_Master_Write16(uint16_t data){
	uint8_t msb, lsb; 
	uint16_t raw_temp;
	uint16_t temp;
	I2C_Master_Write(0x48<<1 | 1);
	msb = I2C_Master_Read(&msb, 1);
	lsb = I2C_Master_Read(&lsb, 1);
	
	raw_temp = (msb << 8) | lsb; 
	temp = raw_temp/255; 
	
}*/
//Función de recepción de los datos enviados por el esclavo al maestro 
//Esta función es para leer los datos que estan en el esclavo
uint8_t I2C_Master_Read(uint8_t *buffer, uint8_t ack){
	uint8_t estado;
	if(ack){
		TWCR |= (1<<TWEA); //Lectura con ACK
	}else{
		TWCR &= ~(1<<TWEA); //Lectura sin ACK
	}

	TWCR |= (1<<TWINT); //Iniciamos la Lectura
	while (!(TWCR & (1<<TWINT))); //Espera al flag TWINT
	estado = TWSR & 0xF8; //Verifica el estado
	//Verifica dato leido con ACK o sin ACK
	if(estado == 0x58 || estado == 0x50){
		*buffer = TWDR;
		return 1;
	}else{
		return estado;
	}
}

//Función para inicializar I2C esclavo
void I2C_Slave_Init(uint8_t address){
	DDRC &= ~((1<<DDC4)|(1<<DDC5));
	
	TWAR = address << 1 & 0b11111110; //Se asigna la dirección que tendra 
	//TWAR = (address << 1| 0x01); //Se asigna la dirección que tendra y habilita
	
	//Se habilita la interfaz, ACK automatico, se habilita la ISR
	TWCR = (1<<TWEA)|(1<<TWEN)|(1<<TWIE);
} 

uint8_t I2C_Master_Start_Sens(void) {
	TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN); // Envía una señal de START
	while (!(TWCR & (1 << TWINT))); // Espera a que el START se complete

	uint8_t estado = TWSR & 0xF8; // Lee el estado del TWI
	if (estado != 0x08 && estado != 0x10) {
		return 1; // Error
	}
	return 0; // Éxito
}

uint8_t I2C_Master_Read_Sen(uint8_t *buffer, uint8_t ack) {
	if (ack) {
		TWCR |= (1 << TWEA); // Configura ACK para la próxima lectura
		} else {
		TWCR &= ~(1 << TWEA); // Desactiva ACK
	}
	TWCR |= (1 << TWINT); // Inicia la lectura
	while (!(TWCR & (1 << TWINT))); // Espera a que el dato sea recibido

	uint8_t estado = TWSR & 0xF8; // Lee el estado del TWI
	if (estado == 0x58 || estado == 0x50) { // Verifica estado de recepción
		*buffer = TWDR; // Lee el dato recibido
		return 0; // Recepción exitosa
		} else {
		return estado; // Retorna el código de error
	}
}

// Funcion para leer un valor de 16 bits de datos desde el esclavo para el maestro.
uint16_t I2C_read_data_16bits(uint8_t direccion_esclavo, uint16_t *dato){
	// Iniciar condicion de START
	TWCR = (1<<TWSTA)|(1<<TWEN)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT)));	// Esperar a que se complete
	
	// Verifica la condicion de start
	if ((TWSR & 0xF8) != TW_START)
	{
		return 1;	// No se pudo iniciar la condicion de start
	}
	
	// Enviar direccion del esclavo con bit de lectura
	TWDR = SLA_R(direccion_esclavo);
	TWCR = (1<<TWEN)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT)));	// Espera a que se complete la transmision
	
	// Verifica ACK/NACK de la direccion
	if ((TWSR & 0xF8) == TW_MR_SLA_NACK)
	{
		I2C_Master_Stop();	// Detiene la transmision si es un NACK
		return 1;
	} else if ((TWSR & 0xF8) != TW_MR_SLA_ACK)
	{
		I2C_Master_Stop();	// Detener si recibe otro dato erroneo
		return 1;
	}
	
	// Lee el byte alto del Slave
	TWCR = (1<<TWEN)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT)));	// Esperar a que se complete la recepcion
	
	// Verifica la recepcion
	if ((TWSR & 0xF8) != TW_MR_DATA_NACK && (TWSR & 0xF8) != TW_MR_DATA_ACK)
	{
		I2C_Master_Stop();	// Detener la recepcion
		return 1;
	}
	
	// Lee dato recibido (byte alto)
	uint8_t byte_alto = TWDR;
	
	// Lee el byte bajo del Slave
	TWCR = (1<<TWEN)|(1<<TWINT);
	while(!(TWCR & (1<<TWINT)));	// Esperar a que se complete la recepcion
	
	// Verifica la recepcion
	if ((TWSR & 0xF8) != TW_MR_DATA_NACK && (TWSR & 0xF8) != TW_MR_DATA_ACK)
	{
		I2C_Master_Stop();	// Detener la recepcion
		return 1;
	}
	
	// Lee dato recibido (byte bajo)
	uint8_t byte_bajo = TWDR;
	
	// Combina los bytes para formar un valor de 16 bits
	*dato = (uint16_t)byte_alto << 8 | byte_bajo;
	
	I2C_Master_Stop();
	
	return *dato;
}
