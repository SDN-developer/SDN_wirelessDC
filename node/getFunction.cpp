
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../node/node.h"
#include "../event/event.h"
#include "../IP/IP.h"

// Node class
int Node::getID(void){
	return id;
}
IP Node::getIP(void){
	return ip;
}
