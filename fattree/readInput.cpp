
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../node/node.h"
#include "../switch/core.h"
#include "../switch/aggregate.h"
#include "../switch/edge.h"
#include "../host/host.h"
#include "../fattree/fattree.h"
#include "../event/event.h"
#include "../event/eventType.h"

// Read packet/flow
void Fattree::readInput(void){

	// Variables
	int byte[4], srcPort, dstPort, protocol, seq, flowSize;
	int hostID;
	int firstPktSize;
	int lastPktSize;
	char charSrcIP[20], charDstIP[20];
	double timeStamp, dataRate;
	IP dstIP, srcIP;
	Event evt;
	Packet pkt;

	// Packets (5-tuples and arrival time)
	seq = 1;
	totFlow = 0;
	while(scanf("%s %s %d %d %d %lf %d %lf", charSrcIP, charDstIP, 
				&srcPort, &dstPort, &protocol, &timeStamp, &flowSize, &dataRate) == 8){

		// Setup Packet Info
		srcIP.setIP(charSrcIP);
		dstIP.setIP(charDstIP);
		pkt.setSrcIP(srcIP);
		pkt.setDstIP(dstIP);
		pkt.setSrcPort(srcPort);
		pkt.setDstPort(dstPort);
		pkt.setProtocol(protocol);
		pkt.setSequence(seq++);
		pkt.setFlowSize(flowSize);
		pkt.setDataRate(dataRate);

		// Transmission variables
		hostID = numberOfCore + numberOfAggregate + numberOfEdge
				+ srcIP.byte[1]*pod*pod/4 + srcIP.byte[2]*pod/2 + srcIP.byte[3]-2;
		firstPktSize = myMin(flowSize, PKT_SIZE);
		lastPktSize = (flowSize % PKT_SIZE) ? (flowSize % PKT_SIZE) : PKT_SIZE;

		// Flow setup request event for first switch
		evt.setTimeStamp(timeStamp + firstPktSize/dataRate);
		evt.setEventType(EVENT_FLOWSETUP);
		evt.setPacket(pkt);
		evt.setID(node[hostID]->link[0].id);
		eventQueue.push(evt);

		// Flow done checking event (at least two hops: src->Edge->dst)
		evt.setTimeStamp(timeStamp + flowSize/dataRate + lastPktSize/dataRate);
		evt.setEventType(EVENT_CHECK);
		evt.setPacket(pkt);
		eventQueue.push(evt);

		// Initialize flow completion time as -1 (impossible)
		flowCompTime[seq-1] = -1;

		// Record flow arrival time
		metric_flowArrivalTime[seq-1] = timeStamp;
		
		// Record last flow setup time 
		lastflowSetupTime[seq-1] = 0.0;

		// Begin transmission:
		// Change inactive rule to active rule if it's the first flow of this header
		begTransmission(timeStamp, pkt);

		// Update total number of flow
		totFlow ++;
	}
}
