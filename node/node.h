
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../event/event.h"
#include "../IP/IP.h"

// Node class
#ifndef NODE_H
#define NODE_H
class Node{
	public:
		Node(int);
		void setID(int);
		void setIP(IP);;
		int getID(void);
		IP getIP(void);
		vector<EDGE>link;
	private:
		int id;
		IP ip;
};
#endif
