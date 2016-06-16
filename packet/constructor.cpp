// Packet class
#include "../packet/packet.h"
#include "../IP/IP.h"

// Packet class definition
Packet::Packet(void){
}
Packet::Packet(IP srcIP, IP dstIP, unsigned int srcPort, unsigned int dstPort, unsigned int protocol){
	this->srcIP = srcIP;
	this->dstIP = dstIP;
	this->srcPort = srcPort;
	this->dstPort = dstPort;
	this->protocol = protocol;
}
