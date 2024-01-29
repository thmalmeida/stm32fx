/*
 * loadCell.h
 *
 *  Created on: 23 de mai de 2017
 *      Author: titi
 * 	Modified: 19/01/2024
 */

#pragma once

#include "hx711.h"


//#define pin_data_HX711	32
//#define pin_sck_HX711	31
// #define pin_tare_HX711	32
// #define pin_led			2
// #define pin_beep		8

class LOAD_CELL {
public:

	static const int nWeight = 50;			// number of samples to count into average summation;
	int stabWeight = 501;		//
	int unstWeight = 25000;	//

	const double betaV[2][11] = {{0.01, 0.02, 0.03, 0.04, 0.07, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0},
								 {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1, 0.2, 0.3}};

	int WeightIndex = 0;

	uint8_t pin_data_HX711;
	uint8_t pin_sck_HX711;

	uint8_t stable = 0;

	int timeD = 1;							// time/frequency of HX711 interface;
	int offset = 0;							// offset after tare;
	int Weight = 0;							// currently weight x10;

	int P;									// currently weight found;
	int convCount;							// counter to fast convergence find;

	int WeightVect[nWeight];				// vector of weights calculated;
	int signalVect[nWeight];				// digital values obtained from HX711 24 bits;
	int signal;
	int error;

	double beta = 0.05;
	static const int Waccu = 100;			//
//	static const int Werror = Waccu*0.10;	// 1000;
//	double A = 1.3299;						// mV/V signal response;
	double A;//= 3.0012;						// mV/V signal response;
	double Kp;// = 1.1030;						// proportional constant;
	double Vrange = 20.0;					// Small signal scale range [mV];
	double scaleHalf = 8388607.0;			// ((2^24)/2)-1;
	double Wmax = 1000.0;					// Sensor max weight [g];
//	double Vref = 5.23;						// Voltage reference [V];
//	double Vref = 5.14;						// Voltage reference [V];
	double Vref = 5.04;						// Voltage reference [V];

	void drive_beep(uint8_t beeps, uint8_t timeH, uint8_t timeL);
	void drive_led_blink();
	void drive_led(uint8_t status);
	void begin_loadcell(uint8_t pin_data, uint8_t pin_sck, double _A, double _Kp);
	int readInput();
	void pin_sck_set(uint8_t status);
	void pin_data_set(uint8_t status);
	uint32_t pin_data_get();
	int get_weight();

	void tareSystem();
	void tareSystem2();
	void tareSystem3();
	int readTareButton();

	void example1(void);

private:
	HX711 load;
};

//#define pin_data_HX711	32
//#define pin_sck_HX711		31
//#define pin_tare_HX711	30
//#define pin_led			1

void LOADCELL::init(uint8_t pin_data, uint8_t pin_sck, double _A, double _Kp) {

	gateConfig(pin_data, 0);		// data pin
	gateConfig(pin_sck, 1);			// sck pin
	gateConfig(pin_tare_HX711, 0);	// data pin
	gateConfig(pin_led, 1);			// led
	gateConfig(pin_beep, 1);		// beep

	pin_data_HX711 = pin_data;
	pin_sck_HX711 = pin_sck;

	A = _A;
	Kp = _Kp;

//	glcd_init();
//	tareSystem3();
}
//void LOADCELL::tareSystem()
//{
//	int i, n = 100;
//
//	for(i=0;i<n;i++)
//	{
//		offset += readInput();
//	}
//	offset = offset/n;
//}
//void LOADCELL::tareSystem2()
//{
//	while(get_weight() != 0)
//	{
//		get_weight();
//
//		int Ssum = 0;
//		for(int i=0; i< nWeight ;i++)
//		{
//			Ssum+= signalVect[i];
//		}
//		offset = Ssum/nWeight;
//	}
//}
void LOADCELL::tareSystem3()
{
	int Ssum = 0;
	for(int i=0; i<nWeight; i++)
	{
		Ssum+= signalVect[i];
	}
	offset = Ssum/nWeight;

	Weight = 0;

	drive_led_blink();
}
void LOADCELL::pin_sck_set(uint8_t status)
{
	gateSet(pin_sck_HX711, status);
}
uint32_t LOADCELL::pin_data_get()
{
	return (uint32_t) gateRead(pin_data_HX711, 0);
}
void LOADCELL::pin_data_set(uint8_t status)
{
	gateSet(pin_data_HX711, status);
}
int LOADCELL::readInput()
{
	int i, cycles = 24;
	int Count = 0;

	pin_data_set(1);
	pin_sck_set(0);
	while(pin_data_get());

	for(i=0;i<cycles;i++)
	{
		pin_sck_set(1);
		_delay_us(timeD);

		Count = Count << 1;

		pin_sck_set(0);
		_delay_us(timeD);

		if(pin_data_get())
		{
//			Count++;
			Count |= 1;
		}

//		weigth = ((weigth << 1) | (pin_data_get()));
	}

	// 25 clk
	pin_sck_set(1);
	_delay_us(timeD);
	pin_sck_set(0);
	_delay_us(timeD);

//	// 26 clk
//	pin_sck_set(1);
//	_delay_us(timeD);
//	pin_sck_set(0);
//	_delay_us(timeD);
//
//	// 27 clk
//	pin_sck_set(1);
//	_delay_us(timeD);
//	pin_sck_set(0);
//	_delay_us(timeD);

//	Count ^= 0xFF800000;
//	Count = ~Count + 1 ;
//	Count &= 0x007FFFFF;

	return Count;
}
int LOADCELL::readTareButton()
{
	uint8_t status = 0;
	status = !gateRead(pin_tare_HX711, 0);

	if(status)
	{
		_delay_ms(50);
	}

	return status;
}
int LOADCELL::get_weight(void)
{
	signal = readInput();						// Read the ADC channel;
	double Vdig = (signal - offset);			// Remove the offset on pure signal;
	double a = (Kp*A*Vrange*Vdig)/scaleHalf;	// Apply equation and obtain the weight in floating point format;
	int WeightTemp = (int) Waccu*(a*Wmax/Vref);	// Amplifier the value to remove floating point and get an integer number;



	error = WeightTemp - Weight;				// This process accelerate to the outcome convergence;

	if(abs(error) < 100)
	{
		beta = betaV[WeightIndex][0];
	}
	else if(abs(error) < 200)
	{
		beta = betaV[WeightIndex][1];
	}
	else if(abs(error) < 500)
	{
		beta = betaV[WeightIndex][2];
	}
	else if(abs(error) > 500 && abs(error) < 1000)
	{
		beta = betaV[WeightIndex][3];
	}
	else if(abs(error) > 1000 && abs(error) < 2000)
	{
		beta = betaV[WeightIndex][4];
	}
	else if(abs(error) > 2000 && abs(error) < 3000)
	{
		beta = betaV[WeightIndex][5];
	}
	else if(abs(error) > 3000 && abs(error) < 5000)
	{
		beta = betaV[WeightIndex][6];
	}
	else if(abs(error) > 5000 && abs(error) < 10000)
	{
		beta = betaV[WeightIndex][7];
	}
	else if(abs(error) > 10000 && abs(error) < 20000)
	{
		beta = betaV[WeightIndex][8];
	}
	else if(abs(error) > 20000 && abs(error) < 25000)
	{
		beta = betaV[WeightIndex][9];
	}
	else
	{
		beta = betaV[WeightIndex][9];		// suppose to beta = 1.00;
	}

	if(beta == 1.0)
	{
		for(int i=1; i<nWeight;i++)
		{
			signalVect[i] = signal;
//			WeightVect[i] = WeightTemp;
		}
//		Weight = WeightTemp;
	}
	else
	{
		//	int Wsum = 0;
			for(int i=(nWeight-1);i>0;i--)
			{
			signalVect[i] = signalVect[i-1];
		//		WeightVect[i] = WeightVect[i-1];
		//		Wsum+= WeightVect[i];
			}
			signalVect[0] = signal;
		//	WeightVect[0] = WeightTemp;
		//	Wsum+= WeightVect[0];
		//	Weight = Wsum/nWeight;
	}

	if(abs(error) < 1000)	// If weight found is less then stabWeight we take stable weight
	{
		stable = 1;
		WeightIndex = 1;
	}
	else if((abs(error) > 20000) && stable == 1) // else, if goes bigger than unstWeight we don't have the weight yet
	{
//		convCount++;
//		if(convCount > 500)
//		{
//			convCount = 0;
			stable = 0;
			WeightIndex = 0;
//		}
	}

	Weight = beta*WeightTemp + Weight - beta*Weight;	// Low Pass Filter

	return Weight;
}
void LOADCELL::drive_beep(uint8_t beeps, uint8_t timeH, uint8_t timeL)
{
	int i;
	for(i=0;i<beeps;i++)
	{
		gateSet(pin_beep, 1);
		_delay_ms(timeH);
		gateSet(pin_beep, 0);
		_delay_ms(timeL);
	}
}
void LOADCELL::drive_led_blink()
{
	 gateToggle(pin_led);
}
void LOADCELL::drive_led(uint8_t status)
{
	gateSet(pin_led, !status);
}
void LOADCELL::example1(void)
{
	P = get_weight();

	if(readTareButton())
	{
		tareSystem3();
//		glcd_clear2();
	}

//		sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu))/100);	// 2 digitos
	sprintf(Serial.buffer,"%5.1d.%.1d", P/(Waccu), abs(P%(Waccu))/1000);	// 1 digito para Waccur 10000
//		if(P>0)
//		{
////			sprintf(Serial.buffer,"%4.1d.%.2d", P/weight.Waccu, P%weight.Waccu);
//			sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu*10)));
//		}
//		else
//		{
////			sprintf(Serial.buffer,"%4.1d.%.2d", P/weight.Waccu, abs(P%weight.Waccu));
////			sprintf(Serial.buffer,"%3.1d.%.3d", P/weight.Waccu, abs(P%(weight.Waccu)));
//			sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu*10)));
//		}

//		sprintf(Serial.buffer,"%4.1d", P/100);
//	Serial.println(Serial.buffer);
//	glcd_big_str(0,2,Serial.buffer);
//	glcd_put_string(70,4," g");
}

#endif /* HARDWARE_H_ */

#endif /* ACIONNA_H_ */
