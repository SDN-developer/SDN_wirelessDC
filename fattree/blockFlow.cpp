// Header
#include "fattree.h"
#include "../basic_lib.h"
#include "../event/event.h"

// Block flow check
bool Fattree::blockFlow(Event preEvt, Event nowEvt){

	int port;

	// Host
	if(preEvt.getID() >= numberOfCore + numberOfAggregate + numberOfEdge){
		if(node[preEvt.getID()]->link[0].cap < preEvt.getPacket().getDataRate())
			return true;
	}

	// Switch
	else{
		for(port = 0; port < node[preEvt.getID()]->link.size(); port++)
			if(node[preEvt.getID()]->link[port].id == nowEvt.getID()) break;
		if(node[preEvt.getID()]->link[port].cap < preEvt.getPacket().getDataRate())
			return true;
		for(port = 0; port < node[nowEvt.getID()]->link.size(); port++)
			if(node[nowEvt.getID()]->link[port].id == preEvt.getID()) break;
		if(node[nowEvt.getID()]->link[port].cap < preEvt.getPacket().getDataRate())
			return true;
	}

	// Pass
	return false;
}
