// Headers
#include "fattree.h"
#include "../basic_lib.h"

// Remove expired entries
void Fattree::updateTCAM(int nid, int timeStamp){

	Packet tmpPkt;

	// The most front entries (Active)
	while(!sw[nid]->TCAMactive.empty()){
		if(sw[nid]->TCAMactive.front().isExpired(timeStamp)){
			tmpPkt = sw[nid]->TCAMactive.front().getSample();
			sw[nid]->TCAMmapA.erase(tmpPkt);
			sw[nid]->TCAMactive.pop_front();
		}
		else break;
	}

	// The most front entries (Inactive)
	while(!sw[nid]->TCAMinactive.empty()){
		if(sw[nid]->TCAMinactive.front().isExpired(timeStamp)){
			tmpPkt = sw[nid]->TCAMinactive.front().getSample();
			sw[nid]->TCAMmapI.erase(tmpPkt);
			sw[nid]->TCAMinactive.pop_front();
		}
		else break;
	}
}
