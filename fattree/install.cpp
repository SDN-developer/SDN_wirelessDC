
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../node/node.h"
#include "../switch/core.h"
#include "../switch/aggregate.h"
#include "../switch/edge.h"
#include "../host/host.h"
#include "../fattree/fattree.h"
#include "../event/event.h"
#include "../event/eventType.h"

// Install event and forward
void Fattree::install(Event evt){

	// Variables
	int ts = evt.getTimeStamp();
	int nid = evt.getID();
	Entry ent = evt.getEntry();
	Packet pkt = evt.getPacket();
	Packet tmpPkt;
	Event ret;

	// Remove the expired entries
	updateTCAM(nid, ts);

	// Remove the entries of the same flow ID (active)
	if(sw[nid]->TCAMmapA.count(pkt) > 0){
		sw[nid]->TCAMactive.remove( sw[nid]->TCAMmapA[pkt] );
		sw[nid]->TCAMmapA.erase(pkt);
	}

	// Remove the entries of the same flow ID (inactive)
	if(sw[nid]->TCAMmapI.count(pkt) > 0){
		sw[nid]->TCAMinactive.remove( sw[nid]->TCAMmapI[pkt] );
		sw[nid]->TCAMmapI.erase(pkt);
	}

	// If TCAM is full
	if(sw[nid]->TCAMactive.size() + sw[nid]->TCAMinactive.size() >= MAX_TCAM_ENTRY){

		// Inactive TCAM first
		if(sw[nid]->TCAMinactive.size() > 0) {

			// Remove from TCAM
			tmpPkt = sw[nid]->TCAMinactive.front().getSample();
			sw[nid]->TCAMmapI.erase(tmpPkt);
			sw[nid]->TCAMinactive.pop_front();

			// Send flow setup request event for this rule
			ret.setTimeStamp(evt.getTimeStamp() + TCAM_SEARCH_DELAY);
			ret.setEventType(EVENT_FLOWSETUP);
			ret.setPacket(tmpPkt);
			ret.setID(nid);
			eventQueue.push(ret);
		}

		// Active TCAM
		else {

			// Remove from TCAM
			tmpPkt = sw[nid]->TCAMactive.front().getSample();
			sw[nid]->TCAMmapA.erase(tmpPkt);
			sw[nid]->TCAMactive.pop_front();

			// Send flow setup request event for this rule
			ret.setTimeStamp(evt.getTimeStamp() + TCAM_SEARCH_DELAY);
			ret.setEventType(EVENT_FLOWSETUP);
			ret.setPacket(tmpPkt);
			ret.setID(nid);
			eventQueue.push(ret);
		}

		// Count
		if(nid < numberOfCore) ruleReplacementCore ++;
		else if(nid < numberOfCore + numberOfAggregate) ruleReplacementAggr ++;
		else ruleReplacementEdge ++;
	}

	// Install at the tail
	sw[nid]->TCAMmapA[pkt] = sw[nid]->TCAMactive.push_back(ent);
}
