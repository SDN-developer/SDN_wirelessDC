
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../entry/entry.h"
#include "../IP/IP.h"

bool Entry::isMatch(Packet pkt){

	// IPs
	IP srcIP, dstIP;
	srcIP = pkt.getSrcIP();
	dstIP = pkt.getDstIP();
	
	// Source IP subnet
	bool ok = true;
	for(int i = 0; i < 4; i++)
		if(srcMask[i] != -1 && srcMask[i] != srcIP.byte[i]){
			ok = false;
			break;
		}
	
	// Destination IP subnet
	for(int i = 0; i < 4; i++)
		if(dstMask[i] != -1 && dstMask[i] != dstIP.byte[i]){
			ok=false;
			break;
		}
	
	// Source/Destination port
	if(srcPort != -1 && srcPort != pkt.getSrcPort()) ok = false;
	if(dstPort != -1 && dstPort != pkt.getDstPort()) ok = false;

	// Protocol
	if(protocol != -1 && protocol != pkt.getProtocol()) ok = false;

	// Matching result
	return ok;
}
