// Header
#include "fattree.h"
#include "../IP/IP.h"

// Legal destination address
bool Fattree::legalAddr(IP dstIP){

	// Out of range
	if(dstIP.byte[0] != 10 ||
			dstIP.byte[1] < 0 || dstIP.byte[1] > pod-1 ||
			dstIP.byte[2] < 0 || dstIP.byte[2] > pod/2-1 ||
			dstIP.byte[3] < 2 || dstIP.byte[3] > pod/2+1){
		return false;
	}
	return true;
}
