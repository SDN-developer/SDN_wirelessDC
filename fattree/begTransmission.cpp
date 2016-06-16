
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../fattree/fattree.h"
#include "../entry/entry.h"

// Install event and forward
void Fattree::begTransmission(double timeStamp, Packet pkt){

	// Variables
	int nowFlowID;
	int nid;
	Entry ent;

	// First flow into the network
	if(aliveFlow[pkt] == 0){

		// If such flow had been arrived to network before
		nowFlowID = rcdFlowID[pkt];
		if(nowFlowID != 0){

			// All switches along the path
			for(int i = 0; i < allEntry[nowFlowID].size(); i++){

				// Move from inactive to active
				ent = allEntry[nowFlowID][i];
				nid = ent.getSID();

				// Remove from inactive
				if(sw[nid]->TCAMmapI.count(pkt) > 0){
					sw[nid]->TCAMinactive.remove(sw[nid]->TCAMmapI[pkt]);
					sw[nid]->TCAMmapI.erase(pkt);

					// Update timestamp & Install at the tail (LRU)
					ent.setExpire(timeStamp + INF);
					sw[nid]->TCAMmapA[pkt] = sw[nid]->TCAMactive.push_back(ent);
				}
			}
		}
	}

	// Update count of alive flows
	aliveFlow[pkt] ++;
}
