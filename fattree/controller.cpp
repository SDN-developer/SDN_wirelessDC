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
#include "../mysort/mysort.h"

// Controller
void Fattree::controller(Event ctrEvt){

	// Variables
	int nid;
	int pathLen;
	int nowFlowID;
	int temp;
	double delay;
	double flowSetupDelay = FLOW_SETUP_DELAY;
	double computePathDelay = CONTROL_PATH_DELAY;
	Event evt, ret;
	Packet pkt;
	Entry ent;
	vector<Event>flowSetupEvent;
	vector<Entry>vent;
	bool hasHandle = false;
	int k;

	int lastPacketSize;
	int flowSize;
	int curSwitchID;
	int nowHeaderID;
	double dataRate;
	double cumulatedDelay;

	// Classify events
	for(int i = 0; i < cumQue.size(); i++){
		evt = cumQue[i];
		nid = evt.getID();
		pkt = evt.getPacket();

		// Flow Setup Request
		if(evt.getEventType() == EVENT_FLOWSETUP){

			// Illegal destination address
			if(!legalAddr(pkt.getDstIP())){
				delay = 1.0;
				ret.setTimeStamp(ctrEvt.getTimeStamp() + delay);
				ret.setEventType(EVENT_UNKNOWN);
				this->eventQueue.push(ret);
				continue;
			}

			// Known flow
			else if(rcdFlowID[pkt]){
				nowHeaderID = rcdFlowID[pkt];
				vent.clear();

				// It's a repeated flow with new flowID
				// Currently we don't handle repeated flows
				if(pkt.getSequence() != -1){
					fprintf(stderr, "[Error] Currently we don't handle repeated flows\n");
					exit(1);
				}

				// For all flow ID of the current header
				bool found = false;
				for(int j = 0; j < (int)headerList[nowHeaderID].size(); j++){
					nowFlowID = headerList[nowHeaderID][j];

					// Last packet already passed through this switch: skip
					if(evt.getTimeStamp() >= sw[nid]->flowLeaveTime[nowFlowID]) continue;
					found = true;

					// Update the flow leave time @ remaining switches along the path
					for(int k = 0; k < (int)allEntry[nowHeaderID].size(); k++){
						curSwitchID = allEntry[nowHeaderID][k].getSID();
						if(evt.getTimeStamp() < sw[curSwitchID]->flowLeaveTime[nowFlowID])
							sw[curSwitchID]->flowLeaveTime[nowFlowID] += (ctrEvt.getTimeStamp() - evt.getTimeStamp()) + flowSetupDelay;
					}

					// Update the flow completion time
					flowCompTime[nowFlowID] += (ctrEvt.getTimeStamp() - evt.getTimeStamp()) + flowSetupDelay;
				}

				// At least one flow with this header passes through this switch
				if(found){

					// Increment flow setup request count
					metric_flowSetupRequest ++;

					// Extract original entry
					if(rule(nid, allEntry[nowHeaderID], ent)){
						ent.setExpire(ctrEvt.getTimeStamp() + flowSetupDelay + ENTRY_EXPIRE_TIME);

						// Install the new entry
						ret.setEventType(EVENT_INSTALL);
						ret.setTimeStamp(ctrEvt.getTimeStamp() + flowSetupDelay);
						ret.setID(nid);
						ret.setPacket(pkt);
						ret.setEntry(ent);
						eventQueue.push(ret);
					}

					// Rule not found
					else{
						/* Maybe it's caused by rule deletion of starting edge policy? */
						fprintf(stderr, "Error: extract original flow entry failed.\n");
					}
				}
			}

			// Require to setup along the path
			else flowSetupEvent.push_back(evt);
		}

		// Unknown events
		else{
			delay = 1;
			ret.setTimeStamp(ctrEvt.getTimeStamp() + delay);
			ret.setEventType(EVENT_UNKNOWN);
			eventQueue.push(ret);
		}
	}
	if(((int)cumQue.size()) > 0) hasHandle = true;
	cumQue.clear();

	// Sort with the largest gap between wired & wireless
	mySort msrt(this);
	sort(flowSetupEvent.begin(), flowSetupEvent.end(), msrt);

	// Currently, all flow setup apply wired policy
	for(int j = 0; j < flowSetupEvent.size(); j++){

		// Increment flow setup request
		metric_flowSetupRequest ++;

		// Information
		evt = flowSetupEvent[j];
		nid = evt.getID();
		pkt = evt.getPacket();
		flowSize = pkt.getFlowSize();
		dataRate = pkt.getDataRate();
		lastPacketSize = (flowSize % PKT_SIZE) ? (flowSize % PKT_SIZE) : PKT_SIZE;
	
		// Assign flow ID
		nowHeaderID = flowIDCount ++;
		rcdFlowID[pkt] = nowHeaderID;

		// Record flow ID for the current header
		headerList[nowHeaderID].push_back(pkt.getSequence());

		// Time Stamp
		temp = ctrEvt.getTimeStamp() + flowSetupDelay + computePathDelay;

		// Pure RANDOM!!!
		bool found = false;
		bool isWireless;
		if(rand()%2 && wireless(nid, pkt, vent, temp)){
			isWireless = true;
			found = true;
		}
		else if(wired(nid, pkt, vent, temp)){
			isWireless = false;
			found = true;
		}
		if(found){

			// Reserve capacity
			modifyCap(vent, -pkt.getDataRate(), isWireless);

			// For each rule entry
			cumulatedDelay = 0;
			for(int i = 0; i < vent.size(); i++){

				// Record the finish time for this flow at current switch
				sw[vent[i].getSID()]->flowLeaveTime[pkt.getSequence()] = 
					ctrEvt.getTimeStamp() + computePathDelay + flowSetupDelay + cumulatedDelay;
				cumulatedDelay += lastPacketSize/dataRate;

				// Install rule
				ret.setEventType(EVENT_INSTALL);
				ret.setTimeStamp(ctrEvt.getTimeStamp() + flowSetupDelay + computePathDelay);
				ret.setID(vent[i].getSID());
				ret.setPacket(pkt);
				ret.setEntry(vent[i]);
				eventQueue.push(ret);
			}

			// Update flow completion time (at host)
			flowCompTime[pkt.getSequence()] = 
				ctrEvt.getTimeStamp() + computePathDelay + flowSetupDelay + cumulatedDelay;

			// Record inserted entries
			allEntry.push_back(vent);
		}

		// No route!!!!??
		else{
			fprintf(stderr, "Error: %s to %s: ", pkt.getSrcIP().fullIP.c_str(), pkt.getDstIP().fullIP.c_str());
			fprintf(stderr, "No such path exists.\n");
			/* Here we may need to handle such situation */
		}

		// Clear Entry
		vent.clear();
	}

	// DEBUG: if no event handled, stop
	if(!eventQueue.size()) return;

	// DEBUG log
//if(hasHandle) printf("[%6.1lf] Controller: Waiting for next handle...\n", ctrEvt.getTimeStamp());

	// The next timeout time
	evt = ctrEvt;
	evt.setEventType(EVENT_INTERVAL);
	evt.setTimeStamp(evt.getTimeStamp()+CONTROL_BATCH);
	eventQueue.push(evt);
	return;
}
