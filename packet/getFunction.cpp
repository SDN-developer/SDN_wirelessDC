// Packet class
#include "../packet/packet.h"
#include "../IP/IP.h"

// Packet class definition
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
unsigned int Packet::getFlowSize(void){
	return flowSize;
}
double Packet::getDataRate(void){
	return dataRate;
}
