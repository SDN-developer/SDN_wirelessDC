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

// Set function
void Packet::setSrcIP(IP srcIP){
	this->srcIP = srcIP;
}
void Packet::setDstIP(IP dstIP){
	this->dstIP = dstIP;
}
void Packet::setSrcPort(unsigned int srcPort){
	this->srcPort = srcPort;
}
void Packet::setDstPort(unsigned int dstPort){
	this->dstPort = dstPort;
}
void Packet::setProtocol(unsigned int protocol){
	this->protocol = protocol;
}
void Packet::setSequence(int ss){
	this->seq = ss;
}

// Get function
IP Packet::getSrcIP(void){
	return srcIP;
}
IP Packet::getDstIP(void){
	return dstIP;
}
unsigned int Packet::getSrcPort(void){
	return srcPort;
}
unsigned int Packet::getDstPort(void){
	return dstPort;
}
unsigned int Packet::getProtocol(void){
	return protocol;
}
int Packet::getSequence(void){
	return seq;
}

// Comparator function
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
