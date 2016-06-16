// Packet class
#include "../packet/packet.h"
#include "../IP/IP.h"

// Packet class definition
bool Packet::operator<(const Packet& pp)const{

	// IP
	for(int i = 0; i < 4; i++)
		if(srcIP.byte[i] != pp.srcIP.byte[i])
			return srcIP.byte[i] < pp.srcIP.byte[i];
	for(int i = 0; i < 4; i++)
		if(dstIP.byte[i] != pp.dstIP.byte[i])
			return dstIP.byte[i] < pp.dstIP.byte[i];

	// Port
	if(srcPort != pp.srcPort) return srcPort < pp.srcPort;
	if(dstPort != pp.dstPort) return dstPort < pp.dstPort;

	// Protocol (otherwise)
	return protocol < pp.protocol;
}
