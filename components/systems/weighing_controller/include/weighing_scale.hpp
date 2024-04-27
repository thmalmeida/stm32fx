/* 
* writed by thmalmeida on 20240424
*/

#ifndef WEIGHING_SCALE_HPP__
#define WEIGHING_SCALE_HPP__

#include "load_cell.hpp"
// #include "lcd.hpp"
#include "ssd1306.hpp"
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

#define N_SENSORS_	4

/* Load cell pins definition */
#define PIN_DATA_1	11
#define PIN_SCK_1	12
#define PIN_DATA_2	9
#define PIN_SCK_2	10
#define PIN_DATA_3	7
#define PIN_SCK_3	8
#define PIN_DATA_4	5
#define PIN_SCK_4	6

#define PIN_TARE	13
#define PIN_LED		1
#define PIN_BEEP	4

class Weighing_Scale {
public:

	Weighing_Scale(I2C_Driver *i2c) : d0{i2c}, pin_{{PIN_TARE, 0}, {PIN_LED, 1}, {PIN_BEEP, 1}}, load_cell_{{PIN_DATA_1, PIN_SCK_1, 1}, {PIN_DATA_2, PIN_SCK_2, 2}, {PIN_DATA_3, PIN_SCK_3, 3}, {PIN_DATA_4, PIN_SCK_4, 4}} {
		init();
	}
	
	void init(void) {

		/* ---- print debug welcome mesages ---- */
		#ifdef DEBUG_Weighing_Scale
		printf("Weighing scale init\n");
		#endif

		#ifdef DEBUG_Weighing_Scale
		printf("Weighing scale: tare process\n");
		#endif

		/* ---- output set: Clear lcd and set led indicator to low level ---- */
		d0.clear();
		indicator_(0);

		// little wait before start setup parameters
		delay_ms(1000);

		/* ---- Load cell sensor parameters ---- */
		
		// mV/V signal response;
		// A_[0] = 1.0000;			// 1kg load bar (small load cell sensor)
		A_[0] = 2.9997;			// s1
		A_[1] = 3.0000;			// s2
		A_[2] = 3.0017;			// s3
		A_[3] = 3.0012;			// s4

		// Proportional constant. Specific for each load cell. Must calibrate before use and this is inerent to the part
		// Kp_[0] = 10*1.3629;		// proportional constant of small 1kg load bar sensor
		Kp_[0] = 1.0872;
		Kp_[1] = 1.6200;			// YZC-320 tested on 20170816 with 3.0mV/V and Kc = 1.6985;
		Kp_[2] = 1.0872;			//10.33%
		Kp_[3] = 1.0872;

		// Test small sensor on all 4 HX711
		// for(int i=0; i<N_SENSORS_; i++) {
		// 	A_[i] = A_[0];
		// 	Kp_[i] = 10*1.3629;
		// }

		// Offset to tare to 0 kg on init;
		// offset_[0] = 0;	// s0 small sensor;
		offset_[0] = 11500;	// s1
		offset_[1] = 5000;	// s2
		offset_[2] = 11500; // s3
		offset_[3] = 11500;	// s4

		// Setup load cell parameters
		for(int i=0; i<N_SENSORS_; i++) {
			load_cell_[i].A(A_[i]);
			load_cell_[i].Kp(Kp_[i]);
			load_cell_[i].offset(offset_[i]);
		}

	}
	void run(void) {
		if(tare_request()) {
			tare();
			d0.clear();
			indicator_(1);
			while(tare_request());
			delay_ms(1000);
			indicator_(0);
		}
	}
	void update(void) {
		// Get weight and print to d0
		// For mini load cell
		// sprintf(str, "W:%.0f kg, RAW:%lu\n", static_cast<double>(weight()/1000.0), raw());
		char str[10];
		sprintf(str, "%5d",  weight());
		printf(str);
		d0.print_Arial24x32_Numbers(str, 0, 0);
		// d0.print_Arial16x24("kg", 96, 24);

		// For cattle weight system with 4 load cells
		printf("\e[1;1H\e[2J\n");
		sprintf(str, "P:%5d kg", weight());
		printf(str);
		printf("\n");
		// d0.print(str, 0, 0);
		// d0.print_Arial16x24_Numbers(str, 0, 0);

		for(int i=0; i<N_SENSORS_; i++) {
			sprintf(str, "P%d:%8.d kg", i, load_cell_[i].weight_kg()/1000);
			d0.print(str, 0, (i+4)*8);
			printf(str);
			printf("\n");
		}
	}
	uint32_t raw(void) {
		return load_cell_[0].read_hx711();
	}
	/* @brief Sum of all weight from load cell
	*  @return total weight in kg
	*/
	int weight(void) {
		int P = 0;
		// Do the weights sum
		for(int i=0; i<N_SENSORS_; i++) {
			P +=load_cell_[i].weight_kg()/1000;
		}
		return P;
	}
    void tare(void) {
		for(int i=0; i<N_SENSORS_;i++) {
			load_cell_[i].tare();
		}
	}
	
	int tare_request(void) {
		return !pin_[0].read();
	}

private:

	// For print purpose only
	SSD1306 d0;				// OLED display
	// LCD_Driver d0;			// 16x2 character display
	
	// input tare button, output beep;
	GPIO_Driver pin_[3];	

	// load cells instatiate
	Load_Cell load_cell_[N_SENSORS_];

	// Load cell parameters
	double A_[4], Kp_[4];
	uint32_t offset_[4];

	void indicator_(uint8_t status) {
		pin_[1].write(!status);
	}

};

#endif //__Weighing_Scale_HPP__