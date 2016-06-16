// Headers
#include "fattree.h"
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../entry/entry.h"
#include "../IP/IP.h"

// Wired policy
bool Fattree::wired(int nid, Packet pkt, vector<Entry>& vent, int timeStamp){

	// IP
	IP srcIP = pkt.getSrcIP();
	IP dstIP = pkt.getDstIP();

	// Maximum flow entry
	map<int, vector<int> >prevNode;
	prevNode.clear();

	// Enumerate all path
	int nowID;
	int srcID = nid;
	int dstID;
	int tmpMax;
	int queSiz;
	int endID;
	double dataRate = pkt.getDataRate();
	queue<int>BFS;
	BFS.push(srcID);
	prevNode[srcID].push_back(srcID);

	// Destination switch ID
	int hostID = numberOfCore + numberOfAggregate + numberOfEdge +
		dstIP.byte[1]*pod*pod/4 + dstIP.byte[2]*pod/2 + dstIP.byte[3]-2;
	endID = node[hostID]->link[0].id;

	// Edge -> Aggregate
	if(srcIP.byte[1] != dstIP.byte[1] || srcIP.byte[2] != dstIP.byte[2]){
		nowID = BFS.front();
		BFS.pop();

		// Remove expired entries
		updateTCAM(nowID, timeStamp);
		for(int i = 0; i < pod/2; i++){
			if(node[nowID]->link[i].cap >= dataRate){
				dstID = node[nowID]->link[i].id;
				BFS.push(dstID);
				prevNode[dstID].push_back(nowID);
			}
		}
	}

	// Aggregate -> Core
	if(srcIP.byte[1] != dstIP.byte[1]){
		queSiz = BFS.size();
		for(int i = 0; i < queSiz; i++){
			nowID = BFS.front();
			BFS.pop();

			// Remove expired entries
			updateTCAM(nowID, timeStamp);
			for(int j = 0; j < pod/2; j++){
				if(node[nowID]->link[j].cap >= dataRate){
					dstID = node[nowID]->link[j].id;
					BFS.push(dstID);
					prevNode[dstID].push_back(nowID);
				}
			}
		}
	}

	// Core -> Aggregate
	if(srcIP.byte[1] != dstIP.byte[1]){
		queSiz = BFS.size();
		for(int i = 0; i < queSiz; i++){
			nowID = BFS.front();
			BFS.pop();
			
			// Remove expired entries
			updateTCAM(nowID, timeStamp);
			if(node[nowID]->link[ dstIP.byte[1] ].cap >= dataRate){
				dstID = node[nowID]->link[ dstIP.byte[1] ].id;

				// Record all possible candidates
				if(!prevNode[dstID].size()) BFS.push(dstID);
				prevNode[dstID].push_back(nowID);
			}
		}
	}

	// Aggregate -> Edge
	if(srcIP.byte[1] != dstIP.byte[1] || srcIP.byte[2] != dstIP.byte[2]){
		queSiz = BFS.size();
		for(int i = 0; i < queSiz; i++){
			nowID = BFS.front();
			BFS.pop();

			// Remove expired entries
			updateTCAM(nowID, timeStamp);
			if(node[nowID]->link[ pod/2 + dstIP.byte[2] ].cap >= dataRate){
				dstID = node[nowID]->link[ pod/2 + dstIP.byte[2] ].id;

				// Record all possible candidates
				if(!prevNode[dstID].size()) BFS.push(dstID);
				prevNode[dstID].push_back(nowID);
			}
		}
	}

	// Create entries along these switches
	int port, ranID;
	Entry ent;
	vector<int>revSeq;
	if(prevNode[endID].size() > 0){
		vent.clear();
		nowID = endID;
		revSeq.push_back(endID);
		while(nowID != srcID){
			ranID = rand()%(prevNode[nowID].size());
			nowID = prevNode[nowID][ranID];
			revSeq.push_back(nowID);
		}
		ent.setSrcMask(srcIP.byte[0], srcIP.byte[1], srcIP.byte[2], srcIP.byte[3]);
		ent.setDstMask(dstIP.byte[0], dstIP.byte[1], dstIP.byte[2], dstIP.byte[3]);
		ent.setSrcPort(pkt.getSrcPort());
		ent.setDstPort(pkt.getDstPort());
		ent.setProtocol(pkt.getProtocol());
		ent.setExpire(timeStamp + ENTRY_EXPIRE_TIME);
		for(int i = revSeq.size()-1; i > 0; i--){
			for(port = 0; port < node[revSeq[i]]->link.size(); port++)
				if(node[revSeq[i]]->link[port].id == revSeq[i-1]) break;
			ent.setSID(revSeq[i]);
			ent.setOutputPort(port);
			vent.push_back(ent);
		}
		ent.setSID(revSeq[0]);
		ent.setOutputPort(pod/2 + dstIP.byte[3] - 2);
		vent.push_back(ent);
		return true;
	}

	// No such path
	else return false;
}
