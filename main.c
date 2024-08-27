/*
 * PruebasMasPro1.c
 *
 * Created: 27/08/2024 11:05:55
 * Author : luisa
 */ 

#define F_CPU 16000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#include <stdio.h>
#include <util/twi.h>

#include "I2C/I2C.h"
#include "PWM/PWM1.h"
#include "LCD/LCD.h"

//Direcciones esclavos 
#define slave1 0x30
#define slave2 0x40
#define LM75_ADDRESS 0x48
#define esp32Slav 0x50


#define LOW_THRESHOLD 100   // Ajusta según el comportamiento deseado

char lista[10] = {'0','1','2','3','4','5','6','7','8','9'};
char lista1[4], lista2[4], lista3[4] = {'0', '0', '0'};
char ULTRAlista[13] = {'0','1','2','3','4','5','6','7','8','9','.','c','m'};

char listaULTRA1[4], listaULTRA2[4], listaULTRA3[4] = {'0'};

char buffer[128];
int16_t temp_value = 0;
int16_t last_temp_value = -1;
int16_t temp_display = 0;
	
uint8_t bufferI2C; //Bus de datos
uint8_t receivedValue = 0; //buffer que recibe valores Sensor Luz
uint8_t LecEsp32 = 0; //Lectura push 1
uint8_t LecEspSer = 0; //Lectura push 2
uint8_t receivedVU = 0;  // Variable para almacenar el valor del sensor ultrasónico recibido del esclavo

uint16_t TempRaw; //Variable que recibe los 16 bits del sensor temp
uint8_t TempS; //Variable para guardar el valor del sensor temp
static uint8_t led_state = 0;
static uint8_t ten = 0;

void setup(void);
void Slave1(void); //Sensor de Luz 
void Esp32Slave_RDC(void); //Función mandar comando de lectura DC
void Esp32Slave_RSer(void); //Funcion mandar comando de lectura Servo

void Esp32_DCLec(void); //Función para leer lo que hay en DC
void Esp32_ServLec(void); //Función para leer lo que hay en servo

void Esp32Slave_W(void); //Función mandar valores a Adafruit
void UpdateULTRA(char *ULTRAlista, int valorULTRA);
void Slave2ULTRA(void);

uint8_t mapValue(uint8_t value, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max);

uint8_t read_temperature(uint8_t *high_byte, uint8_t *low_byte);
void sensorTemp();

void Esp32Slave_WTemp(void);
void Esp32Slave_WUltra(void);


int main(void)
{
	setup();
	SetupPWM1(Fast8,Positivo,A,1024);   //Salida D9
    I2C_Master_Init(100000,1); //inicializar como Master Fscl 100Khz, preescaler 1
	initLCD4bits();
	LCD_Set_Cursor(1,1);
	LCD_Write_String("Dis:");
	LCD_Set_Cursor(12,1); //
	LCD_Write_String("ADC:");
	
	LCD_Set_Cursor(6, 1);
	LCD_Write_String("temp:");
	
    while (1) 
    {
		Slave1();
		// Convertir el valor recibido a un formato de cadena para el LCD
		newLista(lista1, receivedValue);
		
		LCD_Set_Cursor(12,2);
		LCD_Write_String(lista1);
		
		Slave2ULTRA();
		// Convertir el valor recibido a un formato de cadena para el LCD
		UpdateULTRA(listaULTRA1, receivedVU);
		
		LCD_Set_Cursor(1,2);
		LCD_Write_String(listaULTRA1);
		
		
		
		Esp32Slave_W(); 
		Esp32Slave_WTemp();
		Esp32Slave_WUltra();
		
		switch(LecEsp32){
			case 0:
				PORTB &= ~(1<<PORTB3);
				
				
				if (receivedValue < LOW_THRESHOLD  ) {
					PORTD |= (1<<PORTD2);  // Enciende el LED
					//ciclo_traba1A(36);
					ciclo_traba1A(26);
					
					
					} else{
					PORTD &= ~(1<<PORTD2); // Apaga el LED
					//ciclo_traba1A(22);
					ciclo_traba1A(38);
				}
				break;
			case 1:
				PORTB |= (1<<PORTB3);
				PORTD &= ~(1<<PORTD2); // Apaga el LED
				ciclo_traba1A(38);
				break;
		}
		
		switch(LecEspSer){
			case 0:
				//PORTB &= ~(1<<PORTB0);
				PORTB &= ~(1<<PORTB4);
				if (TempS > 28 && !led_state || LecEsp32 == '1') {  // Encender LED por encima de 27°C
					PORTB |= (1 << PORTB0);
					led_state = 1;
					} else if (TempS <= 27 && led_state) {  // Apagar LED por debajo de 25°C
					PORTB &= ~(1 << PORTB0);
					led_state = 0;
				}
				break;
			case 1:
				PORTB |= (1<<PORTB4);
				
				PORTB |= (1<<PORTB0);
				break;
		}
		
		
		TempRaw = I2C_read_data_16bits(LM75_ADDRESS, &TempRaw);
		TempS = TempRaw/256; 
		
		LCD_Set_Cursor(6,2);
		snprintf(buffer, sizeof(buffer), "%02d", TempS);
		LCD_Write_String(buffer); 
		
		
		Esp32_DCLec();
		Esp32_ServLec(); 
		
    }
}

void Slave1(void){
	I2C_Master_Start();
	bufferI2C = (slave1 << 1) | 0b00000001;  // Mandando Maestro a leer (SLA + R)
	if (I2C_Master_Write(bufferI2C) != 1) {  // Si no se recibe ACK, detener la comunicación
		I2C_Master_Stop();
	}
	
	if (I2C_Master_Read(&receivedValue, 0) == 1) {  // Leer el valor enviado por el esclavo
		// El valor recibido está en receivedValue
	}
	
	I2C_Master_Stop();
}

void setup(void){
	
	DDRD |= (1<<DDD2);
	PORTD &= ~(1<<PORTD2);
	DDRC |= (1<<DDC2);
	PORTC &= ~(1<<PORTC2);
	DDRB |= (1<<DDB0);
	PORTB &= ~(1<<PORTB0);
	
	DDRB |= (1<<DDB3);
	PORTB &= ~(1<<PORTB3);
	 
	DDRB |= (1<<DDB4);
	PORTB &= ~(1<<PORTB4);
	sei();
}

void Esp32_DCLec(void){
	// Iniciar una nueva comunicación para leer
	I2C_Master_Start();
	
	// Enviar la dirección del esclavo para lectura (SLA + R)
	bufferI2C = (esp32Slav << 1) | 0b00000001;  // Enviar la dirección del esclavo con bit de lectura
	if (I2C_Master_Write(bufferI2C) != 1) {  // Verificar ACK
		I2C_Master_Stop();
		return;
	}
	
	// Leer el valor del esclavo
	if (I2C_Master_Read(&LecEsp32, 0) != 1) {  // Leer el valor y verificar éxito
		I2C_Master_Stop();
		return;
	}
	
	// Finalizar la comunicación
	I2C_Master_Stop();
	// El valor recibido está en LecEsp32
}

void Esp32_ServLec(void){
	// Iniciar una nueva comunicación para leer
	I2C_Master_Start();
	
	// Enviar la dirección del esclavo para lectura (SLA + R)
	bufferI2C = (esp32Slav << 1) | 0b00000001;  // Enviar la dirección del esclavo con bit de lectura
	if (I2C_Master_Write(bufferI2C) != 1) {  // Verificar ACK
		I2C_Master_Stop();
		return;
	}
	
	// Leer el valor del esclavo
	if (I2C_Master_Read(&LecEspSer, 0) != 1) {  // Leer el valor y verificar éxito
		I2C_Master_Stop();
		return;
	}
	
	// Finalizar la comunicación
	I2C_Master_Stop();
	// El valor recibido está en LecEsp32
}

void Esp32Slave_W(void){

	I2C_Master_Start();
	bufferI2C = (esp32Slav << 1) & 0b11111110;  // Mandando Maestro a Escribir (SLA + R)
	if (I2C_Master_Write(bufferI2C) != 1) {  // Si no se recibe ACK, detener la comunicación
		I2C_Master_Stop();
	}
	if(I2C_Master_Write(receivedValue) != 1){
		I2C_Master_Stop();
	}
	
	
	I2C_Master_Stop();
}

void Esp32Slave_WTemp(void){
	
	I2C_Master_Start();
	bufferI2C = (esp32Slav << 1) & 0b11111110;  // Mandando Maestro a Escribir (SLA + R)
	if (I2C_Master_Write(bufferI2C) != 1) {  // Si no se recibe ACK, detener la comunicación
		I2C_Master_Stop();
	}
	if(I2C_Master_Write(TempS) != 1){
		I2C_Master_Stop();
	}
	
	
	I2C_Master_Stop();
}

void Esp32Slave_WUltra(void){
	I2C_Master_Start();
	bufferI2C = (esp32Slav << 1) & 0b11111110;  // Mandando Maestro a Escribir (SLA + R)
	if (I2C_Master_Write(bufferI2C) != 1) {  // Si no se recibe ACK, detener la comunicación
		I2C_Master_Stop();
	}
	if(I2C_Master_Write(receivedVU) != 1){
		I2C_Master_Stop();
	}
	
	
	I2C_Master_Stop();
}

void newLista(char *lista, int valor) {
	lista[0] = '0' + (valor / 100);
	lista[1] = '0' + ((valor / 10) % 10);
	lista[2] = '0' + (valor % 10);
	lista[3] = '\0';
}

uint8_t mapValue(uint8_t value, uint8_t in_min, uint8_t in_max, uint8_t out_min, uint8_t out_max) {
	return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void UpdateULTRA(char *ULTRAlista, int valorULTRA) {
	// Construir la cadena en formato [ENTERO].[DECIMAL]
	int entero = valorULTRA / 10;   // Parte entera de la distancia
	int decimal = valorULTRA % 10;  // Parte decimal

	// Para valores menores a 10 (0-9), mostramos 0 en la parte entera
	if (valorULTRA < 10) {
		ULTRAlista[0] = '0';               // Parte entera (cero inicial)
		ULTRAlista[1] = '0' + valorULTRA;  // Parte decimal
		ULTRAlista[2] = 'c';
		ULTRAlista[3] = 'm';               // Parte decimal siempre 0
		} else {
		ULTRAlista[0] = '0' + entero;      // Parte entera
		ULTRAlista[1] = '0' + decimal;     // Parte decimal
		ULTRAlista[2] = 'c';
		ULTRAlista[3] = 'm';               // Parte decimal siempre 0
	}
	ULTRAlista[4] = '\0';  // Terminar la cadena
}


void Slave2ULTRA(void) {
	I2C_Master_Start();
	bufferI2C = (slave2 << 1) | 0b00000001;  // Mandando Maestro a leer (SLA + R)
	if (I2C_Master_Write(bufferI2C) != 1) {  // Si no se recibe ACK, detener la comunicación
		I2C_Master_Stop();
	}
	
	if (I2C_Master_Read(&receivedVU, 0) == 1) {  // Leer el valor enviado por el esclavo
		// El valor recibido está en receivedVU
	}
	
	I2C_Master_Stop();
}



