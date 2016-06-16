
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
#include "../entry/entry.h"

// Fat Tree class
Fattree::Fattree(int k){

	// TCAM entries
	maxEntry = MAX_TCAM_ENTRY;

	// Clear alive flow count
	aliveFlow.clear();

	// Controller available time
	compAvail = 0.0;

	// 1 feet = 0.3048 meter
	// 1 inch = 0.0254 meter
	double feet = 0.3048;
	double inch = 0.0254;
	
	// Width/Length of switch
	double widSw = 24*inch;
	double lenSw = 48*inch;
	
	// Prevent illegal k
	if(k<2) k=2;
	if(k%2) k--;
	pod = k;

	// Number of nodes
	numberOfCore = (k/2)*(k/2);
	numberOfAggregate = k*k/2;
	numberOfEdge = numberOfAggregate;
	numberOfHost = k*k*k/4;
	totalNode = numberOfCore + numberOfAggregate + numberOfEdge + numberOfHost;
	node = new Node*[totalNode];
	sw = new Switch*[totalNode - numberOfHost];

	// All path flow entry reset
	flowIDCount = 1;
	vector<Entry>vent;
	allEntry.push_back(vent);
	rcdFlowID.clear();

	// Core
	IP ip;
	int now = 0;
	for(int i = 0; i < numberOfCore; i++){
		sw[now] = new Core(now);
		ip.setIP(10, pod, i/(pod/2)+1, i%(pod/2)+1);
		sw[now]->setIP(ip);
		node[now] = sw[now];
		now++;
	}

	// Aggregate
	for(int i = 0; i < numberOfAggregate; i++){
		sw[now] = new Aggregate(now);
		ip.setIP(10, i/(pod/2), i%(pod/2)+(pod/2), 1);
		sw[now]->setIP(ip);
		node[now] = sw[now];
		now++;
	}

	// Edge
	for(int i = 0; i < numberOfEdge; i++){
		sw[now] = new Edge(now);
		ip.setIP(10, i/(pod/2), i%(pod/2), 1);
		sw[now]->setIP(ip);
		node[now] = sw[now];

		// Position X, Y
		sw[now]->posXY[0] = (i%(k/2))*widSw + 0.5*widSw + ((i/(k/2))%4)*(10*feet+(k/2)*widSw);
		sw[now]->posXY[1] = 0.5*lenSw + (i/(k*4/2))*(lenSw+8*feet);
		
		// AP data rate
		sw[now]->APrate = LINK_CAPACITY;
		now++;
	}

	// Host
	for(int i = 0; i < numberOfHost; i++){
		node[now] = new Host(now);
		ip.setIP(10, i/(pod*pod/4), (i%(pod*pod/4))/(pod/2), (i%(pod*pod/4))%(pod/2)+2);
		node[now]->setIP(ip);
		now++;
	}

	// Link initialization
	int src, dst;
	EDGE etmp;
	etmp.cap = LINK_CAPACITY;

	// Core - aggregrate links
	for(int i = 0; i < numberOfAggregate; i++)
		for(int j = 0; j < pod/2; j++){
			src = numberOfCore + i;
			dst = (i%(pod/2))*pod/2 + j;
			etmp.id = dst;
			node[src]->link.push_back(etmp);
			etmp.id = src;
			node[dst]->link.push_back(etmp);
		}
	
	// Aggregate - Edge links
	for(int i = 0; i < numberOfAggregate; i++)
		for(int j = 0; j < pod/2; j++){
			src = numberOfCore + i;
			dst = numberOfCore + numberOfAggregate + (i/(pod/2))*pod/2 + j;
			etmp.id = dst;
			node[src]->link.push_back(etmp);
			etmp.id = src;
			node[dst]->link.push_back(etmp);
		}

	// Edge - Host links
	for(int i = 0; i < numberOfEdge; i++)
		for(int j = 0; j < pod/2; j++){
			src = numberOfCore + numberOfAggregate + i;
			dst = numberOfCore + numberOfAggregate + numberOfEdge + i*(pod/2) + j;
			etmp.id = dst;
			node[src]->link.push_back(etmp);
			etmp.id = src;
			node[dst]->link.push_back(etmp);
		}

	// Temp positions
	double x0, y0, x1, y1;

	// Edge - Edge wireless links
	etmp.cap = LINK_CAPACITY;
	for(int i = 0; i < numberOfEdge; i++)
		for(int j = i+1; j < numberOfEdge; j++){
			src = numberOfCore + numberOfAggregate + i;
			dst = numberOfCore + numberOfAggregate + j;
			x0 = sw[src]->posXY[0];
			y0 = sw[src]->posXY[1];
			x1 = sw[dst]->posXY[0];
			y1 = sw[dst]->posXY[1];
			if(myDis(x0, y0, x1, y1) <= WIRELESS_RANGE){
				etmp.id = dst;
				sw[src]->wlink.push_back(etmp);
				etmp.id = src;
				sw[dst]->wlink.push_back(etmp);
			}
		}
	
	// Create interference lists of each wireless edge
	vector<int>tlist;
	for(int i = 0; i < numberOfEdge; i++){
		src = numberOfCore + numberOfAggregate + i;
		for(int j = 0; j < sw[src]->wlink.size(); j++){
			dst = sw[src]->wlink[j].id;
			for(int z = 0; z < numberOfEdge; z++){
				now = numberOfCore + numberOfAggregate + z;
				if(now == src) continue;
				if(vecdot(sw[src]->posXY, sw[dst]->posXY, sw[src]->posXY, sw[now]->posXY) > 0 &&
					vecdot(sw[src]->posXY, sw[dst]->posXY, sw[now]->posXY, sw[dst]->posXY) >= 0 &&
					vecdis(sw[src]->posXY, sw[dst]->posXY, sw[src]->posXY, sw[now]->posXY) <= 11*inch)
					tlist.push_back(now);
			}
			sw[src]->iList.push_back(tlist);
			tlist.clear();
		}
	}

	// Random seeds
	srand((unsigned)time(NULL));

	// Controller interval timeout event
	Event evt;
	evt.setEventType(EVENT_INTERVAL);
	evt.setTimeStamp(CONTROL_BATCH);
	eventQueue.push(evt);

	// Wireless Shortest Path
	wirelessSP();

	// Initialize metric
	metric_flowSetupRequest = 0;
	metric_ruleInstallCount = 0;
	metric_avgFlowCompleteTime = 0.0;
	ruleReplacementCore = 0;
	ruleReplacementAggr = 0;
	ruleReplacementEdge = 0;
}
