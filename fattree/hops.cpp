// Headers
#include "fattree.h"
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../IP/IP.h"

// Wired hop
int Fattree::wiredHop(Packet pkt){

	// IP
	IP srcIP = pkt.getSrcIP();
	IP dstIP = pkt.getDstIP();

	// Wired hop
	if(srcIP.byte[1] != dstIP.byte[1]) return 4;
	else if(srcIP.byte[2] != dstIP.byte[2]) return 2;
	else return 0;
}

// Wireless hop
int Fattree::wirelessHop(Packet pkt){

	// IP
	IP srcIP = pkt.getSrcIP();
	IP dstIP = pkt.getDstIP();

	// Src & Dst ID
	int hostID;
	int sft = numberOfCore + numberOfAggregate;
	hostID = sft + numberOfEdge + srcIP.byte[1]*pod*pod/4 + srcIP.byte[2]*pod/2 + srcIP.byte[3]-2;
	int	srcID = node[hostID]->link[0].id;
	hostID = sft + numberOfEdge + dstIP.byte[1]*pod*pod/4 + dstIP.byte[2]*pod/2 + dstIP.byte[3]-2;
	int dstID = node[hostID]->link[0].id;
	int hops = wlPath[srcID - sft][dstID - sft].size()-1;
	if(hops < 0) return 0;
	return hops;
}
