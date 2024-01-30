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

	WEIGHING_SCALE(void) {
		init();
	}

	void init(void) {

	}

    void tare_system3(void) {
		// int Ssum = 0;
		// for(int i=0; i<nWeight; i++) {
		// 	Ssum+= signalVect[i];
		// }
		// offset = Ssum/nWeight;
	}
};

#endif //__WEIGHING_SCALE_H__