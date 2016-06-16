// Header
#include "fattree.h"
#include "../event/event.h"

// Cumulate
void Fattree::cumulate(Event evt){

	// Cumulate the event
	cumQue.push_back(evt);
}
