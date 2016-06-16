
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../entry/entry.h"
#include "../IP/IP.h"

// Entry
Entry::Entry(int s0, int s1, int s2, int s3, 
		int d0, int d1, int d2, int d3, int out, int pri,
		unsigned int sp, unsigned int dp, unsigned int pro){
	srcMask[0] = s0;
	srcMask[1] = s1;
	srcMask[2] = s2;
	srcMask[3] = s3;
	dstMask[0] = d0;
	dstMask[1] = d1;
	dstMask[2] = d2;
	dstMask[3] = d3;
	srcPort = sp;
	dstPort = dp;
	protocol = pro;
	outputPort = out;
	priority = pri;
	wireless = false;
}
