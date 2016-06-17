
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

	// Clear alive flow count
	aliveFlow.clear();

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
		sw[now]->TCAMSize = MAX_TCAM_ENTRY;
		node[now] = sw[now];
		now++;
	}

	// Aggregate
	for(int i = 0; i < numberOfAggregate; i++){
		sw[now] = new Aggregate(now);
		ip.setIP(10, i/(pod/2), i%(pod/2)+(pod/2), 1);
		sw[now]->setIP(ip);
		sw[now]->TCAMSize = MAX_TCAM_ENTRY;
		node[now] = sw[now];
		now++;
	}

	// Edge
	for(int i = 0; i < numberOfEdge; i++){
		sw[now] = new Edge(now);
		ip.setIP(10, i/(pod/2), i%(pod/2), 1);
		sw[now]->setIP(ip);
		sw[now]->TCAMSize = MAX_TCAM_ENTRY;
		node[now] = sw[now];
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

	// Controller interval timeout event
	Event evt;
	evt.setEventType(EVENT_INTERVAL);
	evt.setTimeStamp(CONTROL_BATCH);
	eventQueue.push(evt);

	// Metric
	metric_flowSetupRequest = 0;
	metric_ruleInstallCount = 0;
	metric_avgFlowCompleteTime = 0.0;
	ruleReplacementCore = 0;
	ruleReplacementAggr = 0;
	ruleReplacementEdge = 0;
}
