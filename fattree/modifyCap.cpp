// Header
#include "fattree.h"
#include "../basic_lib.h"

// Modify capacity
void Fattree::modifyCap(vector<Entry>& vent, double dataRate, bool isWireless){

	// Variable
	int nowID, nxtID, portID;

	// Wireless path
	if(isWireless){

		// Wireless AP along the path
		for(int i = 0; i < (int)vent.size()-1; i++){
			nowID = vent[i].getSID();
			portID = vent[i].getOutputPort();
			nxtID = sw[nowID]->wlink[portID].id;

			// Transmission
			sw[nowID]->APrate += dataRate;
			sw[nxtID]->APrate += dataRate;

			// Interference
			for(int j = 0; j < sw[nowID]->iList[portID].size(); j++)
				sw[ sw[nowID]->iList[portID][j] ]->APrate += dataRate;
		}
	}

	// Wired path
	else{

		// Wired links along the path
		for(int i = 0; i < (int)vent.size()-1; i++){
			nowID = vent[i].getSID();
			portID = vent[i].getOutputPort();
			sw[nowID]->link[portID].cap += dataRate;
		}
	}
}
