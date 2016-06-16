
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

// Get functions
int Fattree::getNumberOfNode(void){
	return totalNode;
}
int Fattree::getNumberOfCore(void){
	return numberOfCore;
}
int Fattree::getNumberOfAggregate(void){
	return numberOfAggregate;
}
int Fattree::getNumberOfEdge(void){
	return numberOfEdge;
}
int Fattree::getNumberOfHost(void){
	return numberOfHost;
}
int Fattree::getNumberOfPod(void){
	return pod;
}
Node **Fattree::getNodePtr(void){
	return node;
}
