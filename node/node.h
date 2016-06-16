
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
		virtual Event forward(double,Packet);
		vector<EDGE>link;
		vector< pair<Event,Event> >blockEvt;	// Blocked event: due to lack of capacity
	private:
		int id;
		IP ip;
};
#endif
