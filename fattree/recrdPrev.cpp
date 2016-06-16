// Header
#include "fattree.h"
#include "../basic_lib.h"
#include "../event/event.h"

// Record previous hop information
void Fattree::recrdPrev(Event preEvt, Event nowEvt){

	// Packet sequence
	int seq = preEvt.getPacket().getSequence();

	// Set previous hop information
	int preID = preEvt.getID();
	this->prevHop[seq].id = preID;

	// Wired
	int port;
	int nowID = nowEvt.getID();
	for(port = 0; port < node[preID]->link.size(); port++)
		if(node[preID]->link[port].id == nowID) break;
	if(port < node[preID]->link.size()){
		this->prevHop[seq].port = port;
		this->prevHop[seq].tranType = PREV_WIRED;
	}

	// Wireless
	else{
		for(port = 0; port < sw[preID]->wlink.size(); port++)
			if(sw[preID]->wlink[port].id == nowID) break;
		this->prevHop[seq].port = port;
		this->prevHop[seq].tranType = PREV_WIRELESS;
	}
}
