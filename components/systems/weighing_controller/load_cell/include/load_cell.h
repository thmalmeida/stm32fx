/*
 * LOAD_CELL.h
 *
 *  Created on: 23 de mai de 2017
 *      Author: titi
 * 	Modified: 19/01/2024
 */

#ifndef __LOAD_CELL_H_
#define __LOAD_CELL_H_

#include "hx711.h"
#include "math.h"

class LOAD_CELL {
public:

	int stabWeight = 501;		//
	int unstWeight = 25000;	//

	const double betaV[2][11] = {{0.01, 0.02, 0.03, 0.04, 0.07, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0},
								 {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1, 0.2, 0.3}};

	int WeightIndex = 0;

	uint8_t stable = 0;

	int Weight = 0;							// currently weight x10;
	int P;									// currently weight found;
	int convCount;							// counter to fast convergence find;

	static const int nWeight = 50;			// number of samples to count into average summation;
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

	// void drive_beep(uint8_t beeps, uint8_t timeH, uint8_t timeL);
	// void drive_led_blink();
	// void drive_led(uint8_t status);

	LOAD_CELL(int pin_data, int pin_sck) : hx711_{pin_data, pin_sck} {

	}

	/* @brief initialize with pins and constants
	* @param pin_data input pin to receive data on serial mode
	* @param pin_sck clock output to hx711 module
	*
	*/
	void init(double _A, double _Kp) {
		A = _A;
		Kp = _Kp;
	}
	int read_hx711() {
		return hx711_.read();
	}
	int weight_kg(void) {

		signal = read_hx711();						// Read the ADC channel from HX711 module;
		double Vdig = (signal - offset_);			// Remove the offset on pure signal;
		double a = (Kp*A*Vrange*Vdig)/scaleHalf;	// Apply equation and obtain the weight in floating point format;
		int WeightTemp = (int) Waccu*(a*Wmax/Vref);	// Amplifier the value to remove floating point and get an integer number;

		error = WeightTemp - Weight;				// This process accelerate to the outcome convergence;

		if(abs(error) < 100) {
			beta = betaV[WeightIndex][0];
		}
		else if(abs(error) < 200) {
			beta = betaV[WeightIndex][1];
		}
		else if(abs(error) < 500) {
			beta = betaV[WeightIndex][2];
		}
		else if(abs(error) > 500 && abs(error) < 1000) {
			beta = betaV[WeightIndex][3];
		}
		else if(abs(error) > 1000 && abs(error) < 2000) {
			beta = betaV[WeightIndex][4];
		}
		else if(abs(error) > 2000 && abs(error) < 3000) {
			beta = betaV[WeightIndex][5];
		}
		else if(abs(error) > 3000 && abs(error) < 5000) {
			beta = betaV[WeightIndex][6];
		}
		else if(abs(error) > 5000 && abs(error) < 10000) {
			beta = betaV[WeightIndex][7];
		}
		else if(abs(error) > 10000 && abs(error) < 20000) {
			beta = betaV[WeightIndex][8];
		}
		else if(abs(error) > 20000 && abs(error) < 25000) {
			beta = betaV[WeightIndex][9];
		}
		else {
			beta = betaV[WeightIndex][9];		// suppose to beta = 1.00;
		}

		if(beta == 1.0) {
			for(int i=1; i<nWeight;i++) {
				signalVect[i] = signal;
				// WeightVect[i] = WeightTemp;
			}
			// Weight = WeightTemp;
		}
		else {
			//	int Wsum = 0;
			for(int i=(nWeight-1);i>0;i--) {
				signalVect[i] = signalVect[i-1];
				// WeightVect[i] = WeightVect[i-1];
				// Wsum+= WeightVect[i];
			}
			signalVect[0] = signal;
			//	WeightVect[0] = WeightTemp;
			//	Wsum+= WeightVect[0];
			//	Weight = Wsum/nWeight;
		}

		if(abs(error) < 1000) { // If weight found is less then stabWeight we take stable weight
			stable = 1;
			WeightIndex = 1;
		}
		else if((abs(error) > 20000) && stable == 1) { // else, if goes bigger than unstWeight we don't have the weight yet
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
	void offset(int _offset) {
		offset_ = _offset;
	}
	int offset(void) {
		return offset_;
	}

private:
	HX711 hx711_;
	int offset_ = 0;							// offset after tare;
};

#endif /* __LOAD_CELL_H_ */

//void LOAD_CELL::tareSystem()
//{
//	int i, n = 100;
//
//	for(i=0;i<n;i++)
//	{
//		offset += readInput();
//	}
//	offset = offset/n;
//}
//void LOAD_CELL::tareSystem2()
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
// void LOAD_CELL::example1(void)
// {
// 	P = get_weight();

// 	if(readTareButton())
// 	{
// 		tareSystem3();
// //		glcd_clear2();
// 	}

// //		sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu))/100);	// 2 digitos
// 	sprintf(Serial.buffer,"%5.1d.%.1d", P/(Waccu), abs(P%(Waccu))/1000);	// 1 digito para Waccur 10000
// //		if(P>0)
// //		{
// ////			sprintf(Serial.buffer,"%4.1d.%.2d", P/weight.Waccu, P%weight.Waccu);
// //			sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu*10)));
// //		}
// //		else
// //		{
// ////			sprintf(Serial.buffer,"%4.1d.%.2d", P/weight.Waccu, abs(P%weight.Waccu));
// ////			sprintf(Serial.buffer,"%3.1d.%.3d", P/weight.Waccu, abs(P%(weight.Waccu)));
// //			sprintf(Serial.buffer,"%4.1d.%.2d", P/(weight.Waccu), abs(P%(weight.Waccu*10)));
// //		}

// //		sprintf(Serial.buffer,"%4.1d", P/100);
// //	Serial.println(Serial.buffer);
// //	glcd_big_str(0,2,Serial.buffer);
// //	glcd_put_string(70,4," g");
// }
