// Header
#include "fattree.h"
#include "../basic_lib.h"

// Modify capacity
void Fattree::modifyCap(vector<Entry>& vent, double dataRate){

	// Variable
	int nowID, nxtID, portID;

	// Wired links along the path
	for(int i = 0; i < (int)vent.size()-1; i++){
		nowID = vent[i].getSID();
		portID = vent[i].getOutputPort();
		sw[nowID]->link[portID].cap += dataRate;
	}
}
