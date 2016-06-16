
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../entry/entry.h"
#include "../IP/IP.h"

void Entry::setSrcMask(int m0, int m1, int m2, int m3){
	srcMask[0] = m0;
	srcMask[1] = m1;
	srcMask[2] = m2;
	srcMask[3] = m3;
}
void Entry::setDstMask(int m0, int m1, int m2, int m3){
	dstMask[0] = m0;
	dstMask[1] = m1;
	dstMask[2] = m2;
	dstMask[3] = m3;
}
void Entry::setSrcPort(unsigned int sp){
	this->srcPort = sp;
}
void Entry::setDstPort(unsigned int dp){
	this->dstPort = dp;
}
void Entry::setProtocol(unsigned int pro){
	this->protocol = pro;
}
void Entry::setOutputPort(int op){
	outputPort = op;
}
void Entry::setPriority(int pp){
	priority = pp;
}
void Entry::setSID(int ss){
	this->sid = ss;
}
void Entry::setRate(double rr){
	this->rate = rr;
}
void Entry::isWireless(bool ff){
	this->wireless = ff;
}
void Entry::setExpire(int tt){
	this->expireTime = tt;
}
