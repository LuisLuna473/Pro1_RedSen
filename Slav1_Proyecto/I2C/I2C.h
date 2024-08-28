/*
 * I2C.h
 *
 * Created: 1/08/2024 16:16:21
 *  Author: luisa
 */ 


#ifndef I2C_H_
#define I2C_H_

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
//Funci�n para inicializar el maestro I2C
void I2C_Master_Init(unsigned SCL_Clock, uint8_t Prescaler);

//Maestro escribe
uint8_t I2C_Master_Write(uint8_t dato);

//Funci�n para inicializar I2C esclavo
void I2C_Slave_Init(uint8_t address);

//Funci�n de recepci�n de los datos enviados por el esclavo al maestro
//Esta funci�n es para leer los datos que estan en el esclavo
uint8_t I2C_Master_Read(uint8_t *buffer, uint8_t ack); 

//Funci�n de parada I2C Maestro
void I2C_Master_Stop(void); 

//Funci�n para inicio de la comunicaci�n I2C Maestro
uint8_t I2C_Master_Start(void);

#endif /* I2C_H_ */