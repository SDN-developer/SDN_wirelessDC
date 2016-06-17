// Header
#include "fattree.h"
#include "../basic_lib.h"
#include "../event/event.h"
#include "../packet/packet.h"

// Resume forwarding flow
void Fattree::resumeFlow(int nid, int timeStamp){

	// Variables
	double forwardDelay;
	double dataRate;
	Event nowEvt, nxtEvt;
	Packet pkt;

	// All queued flows
	for(int i = 0; i < node[nid]->blockEvt.size(); i++){

		// Get info
		nowEvt = node[nid]->blockEvt[i].first;
		nxtEvt = node[nid]->blockEvt[i].second;
		if(!blockFlow(nowEvt, nxtEvt)){

			// Info
			pkt = nxtEvt.getPacket();

			// Pass, update time stamp of event
			forwardDelay = pkt.getFlowSize() / pkt.getDataRate();
			nxtEvt.setTimeStamp(timeStamp + forwardDelay);
			eventQueue.push(nxtEvt);

			// Record previous hop and consume capacity
			recrdPrev(nowEvt, nxtEvt);
			modCap(nowEvt.getID(), nowEvt.getPacket().getSequence(), nowEvt.getPacket().getDataRate()*(-1.0));

			// Remove block flow
			node[nid]->blockEvt.erase( node[nid]->blockEvt.begin() + i);
			i--;
		}
	}
}
