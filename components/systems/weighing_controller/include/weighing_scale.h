#ifndef __WEIGHING_SCALE_H__
#define __WEIGHING_SCALE_H__

#include "load_cell.h"

/* This class defines the controller of weighing scale. It uses each load cell as an object with it's parameters.
* 
* Features:
* Get weight from each load cell 
* sum each weight scale and process the information
* Tare function and your external push button
* Debug and find erros on each load cell
* print to LCD?
*
*/

class WEIGHING_SCALE {
public:

	WEIGHING_SCALE(int pin_data_1, int pin_sck_1,
					int pin_data_2, int pin_sck_2,
					int pin_data_3, int pin_sck_3,
					int pin_data_4, int pin_sck_4) : load_cell_{{pin_data_1, pin_sck_1}, {pin_data_2, pin_sck_2}, {pin_data_3, pin_sck_3}, {pin_data_4, pin_sck_4}} {
		init();
	}

	void init(void) {

	}

	void run(void) {
		
	}

    void tare_system3(void) {
		// int Ssum = 0;
		// for(int i=0; i<nWeight; i++) {
		// 	Ssum+= signalVect[i];
		// }
		// offset = Ssum/nWeight;
	}
private:
	LOAD_CELL load_cell_[4];
};

#endif //__WEIGHING_SCALE_H__