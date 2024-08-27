/*
 * PWM1.h
 *
 * Created: 19/05/2024 13:25:59
 *  Author: luisa
 */ 


#ifndef PWM1_H_
#define PWM1_H_

#define F_CPU 16000000000
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <util/delay.h>
#define Positivo 1
#define Invertido 0
#define Externo 1
#define Interno 0
#define Fast 1
#define Phase 0
#define Todo 1
#define A 0
#define B 1
#define C 2
#define D 3
#define Phase8 0
#define Phase9 1
#define Phase10 2
#define PhaseICR 3
#define PhaseOCR 4
#define Fast8 5
#define Fast9 6
#define Fast10 7
#define FastICR 8
#define FastOCR 9

void SetupPWM1(uint8_t ModoPWM1, uint8_t Sentido1, uint8_t SaidaPMW1, uint16_t PrescalerPWM1);
void SetupPWM2(uint8_t ModoPWM2, uint8_t Sentido2, uint8_t SaidaPMW2, uint16_t PrescalerPWM2);

void ciclo_traba1A(int ciclo);
void ciclo_traba1B(int ciclo);
void ciclo_traba2A(int ciclo);
void ciclo_traba2B(int ciclo);



#endif /* PWM1_H_ */