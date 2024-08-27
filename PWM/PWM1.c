/*
 * PWM1.c
 *
 * Created: 19/05/2024 13:26:07
 *  Author: luisa
 */ 
#include "PWM1.h"
void SetupPWM1(uint8_t ModoPWM1, uint8_t Sentido1, uint8_t SaidaPMW1, uint16_t PrescalerPWM1){
	//MODOS
	switch (ModoPWM1)
	{
		case Phase8:
		TCCR1A|=(1<<WGM10);
		break;
		case Phase9:
		TCCR1A|=(1<<WGM11);
		break;
		case Phase10:
		TCCR1A|=(1<<WGM10)|(1<<WGM11);
		break;
		case PhaseICR:
		TCCR1B|=(1<<WGM13);
		break;
		case PhaseOCR:
		TCCR1B|=(1<<WGM13);
		TCCR1A |=(1<<WGM12);
		break;
		case Fast8:
		TCCR1B|=(1<<WGM12);
		TCCR1A|=(1<<WGM10);
		break;
		case Fast9:
		TCCR1B|=(1<<WGM12);
		TCCR1A|=(1<<WGM11);
		break;
		case Fast10:
		TCCR1B|=(1<<WGM12);
		TCCR1A|=(1<<WGM10)|(1<<WGM11);
		break;
		case FastICR:
		TCCR1B|=(1<<WGM13)|(1<<WGM12);
		TCCR1A|=(1<<WGM11);
		break;
		case FastOCR:
		TCCR1B|=(1<<WGM13)|(1<<WGM12);
		TCCR1A|=(1<<WGM11)|(1<<WGM10);
		break;
	}
	// SENTIDO Y SALIDA
	if ((ModoPWM1==Fast8)|(ModoPWM1==Fast9)|(ModoPWM1==Fast10)|(ModoPWM1==FastICR)|(ModoPWM1==FastOCR))
	{
		switch (SaidaPMW1)
		{
			case A:
			switch (Sentido1)
			{
				case Positivo:
				TCCR1A|=(1<<COM1A1);
				break;
				case Invertido:
				TCCR1A|=(1<<COM1A1)|(1<<COM1A0);
				break;
			}
			DDRB |= (1<<DDB1);
			PORTB &= ~(1<<PORTB1);
			//TIMSK1 |= (1<<OCIE1A);
			break;
			case B:
			switch (Sentido1)
			{
				case Positivo:
				TCCR1A|=(1<<COM1B1);
				break;
				case Invertido:
				TCCR1A|=(1<<COM1B1)|(1<<COM1B0);
				break;
			}
			DDRB |= (1<<DDB2);
			PORTB &= ~(1<<PORTB2);
			//TIMSK1 |= (1<<OCIE1B);
			break;
		}
	}
	// PRESCALER
	switch (PrescalerPWM1)
	{
		case 0:
		TCCR1B |= (1<<CS10);
		break;
		case 8:
		TCCR1B |= (1<<CS11);
		break;
		case 64:
		TCCR1B |= (1<<CS10)|(1<<CS11);
		break;
		case 256:
		TCCR1B |= (1<<CS12);
		break;
		case 1024:
		TCCR1B |= (1<<CS10)|(1<<CS12);
		break;
	}
	//TIMSK1 |= (1<<TOIE1);
}

void SetupPWM2(uint8_t ModoPWM2, uint8_t Sentido2, uint8_t SaidaPMW2, uint16_t PrescalerPWM2){
	//MODOS
	switch (ModoPWM2)
	{
		case Phase8:
		TCCR2A|=(1<<WGM20);
		break;
		case PhaseOCR:
		TCCR2B|=(1<<WGM20);
		TCCR2A |=(1<<WGM22);
		break;
		case Fast8:
		TCCR2A|=(1<<WGM20)|(1<<WGM21);
		break;
		case FastOCR:
		TCCR2B|=(1<<WGM22);
		TCCR2A|=(1<<WGM21)|(1<<WGM20);
		break;
	}
	// SENTIDO Y SALIDA
	if ((ModoPWM2==Fast8)|(ModoPWM2==Fast9)|(ModoPWM2==Fast10)|(ModoPWM2==FastICR)|(ModoPWM2==FastOCR))
	{
		switch (SaidaPMW2)
		{
			case A:
			switch (Sentido2)
			{
				case Positivo:
				TCCR2A|=(1<<COM2A1);
				break;
				case Invertido:
				TCCR2A|=(1<<COM2A1)|(1<<COM2A0);
				break;
			}
			DDRB |= (1<<DDB3);
			PORTB &= ~(1<<PORTB3);
			//TIMSK2 |= (1<<OCIE2A);
			break;
			case B:
			switch (Sentido2)
			{
				case Positivo:
				TCCR2A|=(1<<COM2B1);
				break;
				case Invertido:
				TCCR2A|=(1<<COM2B1)|(1<<COM2B0);
				break;
			}
			DDRD |= (1<<DDD3);
			PORTD &= ~(1<<PORTD3);
			//TIMSK2 |= (1<<OCIE2B);
			break;
		}
	}
	// PRESCALER
	switch (PrescalerPWM2)
	{
		case 0:
		TCCR2B |= (1<<CS20);
		break;
		case 8:
		TCCR2B |= (1<<CS21);
		break;
		case 32:
		TCCR2B |= (1<<CS20)|(1<<CS21);
		break;
		case 64:
		TCCR2B |= (1<<CS22);
		break;
		case 128:
		TCCR2B |= (1<<CS22)|(1<<CS20);
		break;
		case 256:
		TCCR2B |= (1<<CS22)|(1<<CS21);
		break;
		case 1024:
		TCCR2B |= (1<<CS20)|(1<<CS22)|(1<<CS21);
		break;
	}
	//TIMSK2 |= (1<<TOIE2);
}

void ciclo_traba1A(int ciclo){
	OCR1A = ciclo;
}

void ciclo_traba1B(int ciclo){
	OCR1B = ciclo;
}

void ciclo_traba2A(int ciclo){
	OCR2A = ciclo;
}

void ciclo_traba2B(int ciclo){
	OCR2B = ciclo;
}