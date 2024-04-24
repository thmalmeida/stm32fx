/* 
* writed by thmalmeida on 20240424
*/

#ifndef WEIGHING_SCALE_HPP__
#define WEIGHING_SCALE_HPP__

#include "load_cell.hpp"

/*
* This class defines the controller of weighing scale. It uses each load cell as an object with it's parameters.
* 
* Features:
* Get weight from each load cell 
* sum each weight scale and process the information
* Tare function and your external push button
* Debug and find erros on each load cell
* print to LCD?
*
* Initial weight of cattle contention of each load cell (foot) (Iracema's Farm)
* P1:  420.41 kg
* P2:  340.86 kg
* P3:  401.14 kg
* P4:  360.24 kg
* Pt: 1522.65 kg
*
*/

#define DEBUG_WEIGHING_SCALE 1

/* Load cell pins definition */
#define PIN_DATA_1	27
#define PIN_SCK_1	28
#define PIN_DATA_2	31
#define PIN_SCK_2	32
#define PIN_DATA_3	24
#define PIN_SCK_3	23
#define PIN_DATA_4	22
#define PIN_SCK_4	21

#define N_SENSORS_	1
// #define pin_tare	32
// #define pin_led		2
// #define pin_beep	8

class Weighing_Scale {
public:

	Weighing_Scale(void) : load_cell_{{PIN_DATA_1, PIN_SCK_1}} {
		init();
	}

	// Weighing_Scale(void) : load_cell_{{PIN_DATA_1, PIN_SCK_1}, {PIN_DATA_2, PIN_SCK_2}, {PIN_DATA_3, PIN_SCK_3}, {PIN_DATA_4, PIN_SCK_4}} {
	// 	init();
	// }
	// Weighing_Scale(int pin_data_1, int pin_sck_1,
	// 				int pin_data_2, int pin_sck_2,
	// 				int pin_data_3, int pin_sck_3,
	// 				int pin_data_4, int pin_sck_4) :
	// 					load_cell_{
	// 						{pin_data_1, pin_sck_1},
	// 						{pin_data_2, pin_sck_2},
	// 						{pin_data_3, pin_sck_3},
	// 						{pin_data_4, pin_sck_4}} {
	// 	init();
	// }

	void init(void) {

		/* ---- Load cell sensor parameters ---- */
		
		// mV/V signal response;
		A_[0] = 1.0000;			// 1kg load bar (small load cell sensor)
		// A_[0] = 2.9997;			// s1
		A_[1] = 3.0000;			// s2
		A_[2] = 3.0017;			// s3
		A_[3] = 3.0012;			// s4

		Kp_[0] = 10*1.3629;		// proportional constant of small 1kg load bar sensor
		// Kp_[0] = 1.0872;
		Kp_[1] = 1.6200;			// YZC-320 tested on 20170816 with 3.0mV/V and Kc = 1.6985;
		Kp_[2] = 1.0872;			//10.33%
		Kp_[3] = 1.0872;

		offset_[0] = 0;	// s0 small sensor;
		// offset_[0] = 11500;	// s1
		offset_[1] = 5000;	// s2
		offset_[2] = 11500; // s3
		offset_[3] = 11500;	// s4

		for(int i=0; i<N_SENSORS_; i++) {
			load_cell_[i].offset(offset_[i]);
			load_cell_[i].A(A_[i]);
			load_cell_[i].Kp(Kp_[i]);
		}

		#ifdef DEBUG_Weighing_Scale
		printf("Weighing scale init\n");
		#endif

		#ifdef DEBUG_Weighing_Scale
		printf("Weighing scale: tare process\n");
		#endif
		delay_ms(1000);
		for(int i=0; i<N_SENSORS_;i++) {
			load_cell_[i].tare();
		}


	}
	void run(void) {
		// Get weight and print to lcd
		printf("W:%.2f kg, RAW:%lu\n", static_cast<double>(weight()/1000.0), raw());
	}
	void test(void) {

	}
	uint32_t raw(void) {
		return load_cell_[0].read_hx711();
	}
	int weight(void) {
		int P = 0;
		// Do the weights sum
		for(int i=0; i<N_SENSORS_; i++) {
			P +=load_cell_[i].weight_kg();
		}
		return P;
	}
    void tare_system3(void) {
		// int Ssum = 0;
		// for(int i=0; i<nWeight; i++) {
		// 	Ssum+= signalVect[i];
		// }
		// offset = Ssum/nWeight;
	}
private:

	Load_Cell load_cell_[N_SENSORS_];

	/* Load cell parameters */
	double A_[4], Kp_[4];
	uint32_t offset_[4];

};

#endif //__Weighing_Scale_HPP__