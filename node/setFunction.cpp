
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../node/node.h"
#include "../event/event.h"
#include "../IP/IP.h"

// Node class
void Node::setID(int id){
	this->id = id;
}
void Node::setIP(IP ip){
	this->ip = ip;
}
