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

	// void drive_beep(uint8_t beeps, uint8_t timeH, uint8_t timeL);
	// void drive_led_blink();
	// void drive_led(uint8_t status);

	/* @brief initialize with pins and constants
	* @param pin_data input pin to receive data on serial mode
	* @param pin_sck clock output to hx711 module
	*
	*/
	LOAD_CELL(int pin_data, int pin_sck) : hx711_{pin_data, pin_sck} {

	}

	void A(double _A) {
		A_ = _A;
	}
	void Kp(double _Kp) {
		Kp_ = _Kp;
	}
	int read_hx711() {
		return hx711_.read();
	}
	int weight_kg(void) {

		signal = read_hx711();									// Read the ADC channel from HX711 module;
		double Vdig = (signal - offset_);						// Remove the offset on pure signal;
		double a = (Kp_*A_*Vrange*Vdig)/scale_half_;			// Apply equation and obtain the weight in floating point format;
		int WeightTemp = static_cast<int>(Waccu*(a*Wmax/Vref));	// Amplifier the value to remove floating point and get an integer number;

		error = WeightTemp - Weight;							// This process accelerate to the outcome convergence;

		if(abs(error) < 100) {
			beta_ = beta_v_[beta_index_][0];
		}
		else if(abs(error) < 200) {
			beta_ = beta_v_[beta_index_][1];
		}
		else if(abs(error) < 500) {
			beta_ = beta_v_[beta_index_][2];
		}
		else if(abs(error) > 500 && abs(error) < 1000) {
			beta_ = beta_v_[beta_index_][3];
		}
		else if(abs(error) > 1000 && abs(error) < 2000) {
			beta_ = beta_v_[beta_index_][4];
		}
		else if(abs(error) > 2000 && abs(error) < 3000) {
			beta_ = beta_v_[beta_index_][5];
		}
		else if(abs(error) > 3000 && abs(error) < 5000) {
			beta_ = beta_v_[beta_index_][6];
		}
		else if(abs(error) > 5000 && abs(error) < 10000) {
			beta_ = beta_v_[beta_index_][7];
		}
		else if(abs(error) > 10000 && abs(error) < 20000) {
			beta_ = beta_v_[beta_index_][8];
		}
		else if(abs(error) > 20000 && abs(error) < 25000) {
			beta_ = beta_v_[beta_index_][9];
		}
		else {
			beta_ = beta_v_[beta_index_][9];		// suppose to beta_ = 1.00;
		}

		if(beta_ == 1.0) {
			for(int i=1; i<nWeight;i++) {
				signal_v_[i] = signal;
				// WeightVect[i] = WeightTemp;
			}
			// Weight = WeightTemp;
		}
		else {
			//	int Wsum = 0;
			for(int i=(nWeight-1);i>0;i--) {
				signal_v_[i] = signal_v_[i-1];
				// WeightVect[i] = WeightVect[i-1];
				// Wsum+= WeightVect[i];
			}
			signal_v_[0] = signal;
			//	WeightVect[0] = WeightTemp;
			//	Wsum+= WeightVect[0];
			//	Weight = Wsum/nWeight;
		}

		if(abs(error) < stabWeight) { // If weight found is less then stabWeight we take stable_ weight
			stable_ = 1;
			beta_index_ = 1;
		}
		else if((abs(error) > unstWeight) && stable_ == 1) { // else, if goes bigger than this we don't have the weight yet
		//		convCount++;
		//		if(convCount > 500) {
		//			convCount = 0;
			stable_ = 0;
			beta_index_ = 0;
		//		}
		}

		// Low Pass Filter
		Weight = beta_*WeightTemp + Weight - beta_*Weight;	

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

	// Sensor parameters
	double A_;								// mV/V signal response;
	double Kp_;								// proportional constant to calibrate the load cell (1.1030)
	int offset_ = 0;						// offset after tare;

	// Beta values to low pass filter
	const double beta_v_[2][11] = {{0.01, 0.02, 0.03, 0.04, 0.07, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0},
								 {0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.1, 0.2, 0.3}};

	int beta_index_ = 0;					// index of beta_ array values
	double beta_ = 0.05;					// initial beta value for low pass filter calculation

	int stabWeight = 1000;					//
	int unstWeight = 20000;					//
	uint8_t stable_ = 0;					//

	int Weight = 0;							// currently weight x10;
	int P;									// currently weight found;
	int convCount;							// counter to fast convergence find;

	static const int nWeight = 50;			// number of samples to count into average summation;
	int WeightVect[nWeight];				// vector of weights calculated;
	int signal_v_[nWeight];					// digital values obtained from HX711 24 bits;
	
	int signal;								// raw signal readed from hx711
	int error;								// max error between signals reads

	static const int Waccu = 100;			//
//	static const int Werror = Waccu*0.10;	// 1000;
	double Vrange = 20.0;					// Small signal scale range [mV];
	double scale_half_ = 8388607.0;			// ((2^24)/2)-1;
	double Wmax = 1000.0;					// Sensor max weight [g];
	double Vref = 5.04;						// Voltage reference [V];
//	double Vref = 5.23;						// Voltage reference [V];
//	double Vref = 5.14;						// Voltage reference [V];
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
//			Ssum+= signal_v_[i];
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
