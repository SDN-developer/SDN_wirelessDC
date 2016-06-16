// Headers
#include "../basic_lib.h"
#include "fattree.h"
#include "../packet/packet.h"
#include "../IP/IP.h"

// Path initialization
int Fattree::pathInit(Packet pkt, map<int,int>& prev){
	int hostID, srcID, dstID, nowID;
	IP srcIP = pkt.getSrcIP();
	IP dstIP = pkt.getDstIP();
	queue<int>BFS;

	// Source switch ID
	hostID = numberOfCore + numberOfAggregate + numberOfEdge +
		srcIP.byte[1]*pod*pod/4 + srcIP.byte[2]*pod/2 + srcIP.byte[3]-2;
	srcID = node[hostID]->link[0].id;

	// BFS start
	BFS.push(srcID);
	prev[srcID] = -1;

	// Edge -> Aggregate
	if(srcIP.byte[1] != dstIP.byte[1] || srcIP.byte[2] != dstIP.byte[2]){
		nowID = BFS.front();
		BFS.pop();
		for(int i = 0; i < pod/2; i++){
			dstID = node[nowID]->link[i].id;
			BFS.push(dstID);
			prev[dstID] = -1;
		}
	}

	// Aggregate -> Core
	if(srcIP.byte[1] != dstIP.byte[1]){
		for(int i = 0; i < pod/2; i++){
			nowID = BFS.front();
			BFS.pop();
			for(int j = 0; j < pod/2; j++){
				dstID = node[nowID]->link[j].id;
				prev[dstID] = -1;
				if(!j) BFS.push(dstID);
			}
		}
	}

	// Core -> Aggregate
	int siz;
	if(srcIP.byte[1] != dstIP.byte[1]){
		for(int i = 0; i < pod/2; i++){
			nowID = BFS.front();
			BFS.pop();
			dstID = node[nowID]->link[ dstIP.byte[1] ].id;
			prev[dstID] = -1;
			if(!i) BFS.push(dstID);
		}
	}

	// Aggregate -> Edge
	if(srcIP.byte[1] != dstIP.byte[1] || srcIP.byte[2] != dstIP.byte[2]){
		nowID = BFS.front();
		BFS.pop();
		dstID = node[nowID]->link[ pod/2 + dstIP.byte[2] ].id;
		prev[dstID] = -1;
	}

	// Destination switch ID
	hostID = numberOfCore + numberOfAggregate + numberOfEdge +
		dstIP.byte[1]*pod*pod/4 + dstIP.byte[2]*pod/2 + dstIP.byte[3]-2;
	dstID = node[hostID]->link[0].id;
	return dstID;
}
