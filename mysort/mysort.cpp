// Headers
#include "mysort.h"
#include "../basic_lib.h"
#include "../event/event.h"
#include "../packet/packet.h"
#include "../IP/IP.h"
#include "../node/node.h"

// Comparator
bool mySort::operator()(Event e1, Event e2){
	int wiredHop[2];
	int wirelessHop[2];
	int srcID;
	int dstID;
	int hostID;
	int sft = fPtr->getNumberOfCore() + fPtr->getNumberOfAggregate();
	int pod = fPtr->getNumberOfPod();
	Node **node = fPtr->getNodePtr();
	Event evt[2];
	Packet pkt;
	IP srcIP;
	IP dstIP;

	// Initialize
	evt[0] = e1, evt[1] = e2;

	// Wired hop
	for(int i = 0; i < 2; i++){
		pkt = evt[i].getPacket();
		srcIP = pkt.getSrcIP();
		dstIP = pkt.getDstIP();
		if(srcIP.byte[1] != dstIP.byte[1]) wiredHop[i] = 4;
		else if(srcIP.byte[2] != dstIP.byte[2]) wiredHop[i] = 2;
		else wiredHop[i] = 0;
	}

	// Wireless hop
	for(int i = 0; i < 2; i++){
		srcID = evt[i].getID();
		dstIP = evt[i].getPacket().getDstIP();
		hostID = sft + fPtr->getNumberOfEdge() + dstIP.byte[1]*pod*pod/4 + dstIP.byte[2]*pod/2 + dstIP.byte[3]-2;
		dstID = node[hostID]->link[0].id;
		wirelessHop[i] = fPtr->wlPath[srcID - sft][dstID - sft].size() - 1;
	}
	return wiredHop[0] - myAbs(wirelessHop[0]) > wiredHop[1] - myAbs(wirelessHop[1]);
}
