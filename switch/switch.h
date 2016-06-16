
// Headers
#include "../basic_lib.h"
#include "../node/node.h"
#include "../entry/entry.h"
#include "../event/event.h"
#include "../packet/packet.h"
#include "../mylist/mylist.h"

// Switch class
#ifndef SWITCH_H
#define SWITCH_H
class Switch: public Node{
	public:
		Switch(int);
		vector<Packet>que;
		map<Packet,bool>isSetup;
		double posXY[2];
		vector<EDGE>wlink;
		vector< vector<int> >iList;
		double APrate;
		MyLL TCAMactive;
		MyLL TCAMinactive;
		map<Packet,LLNODE*>TCAMmapA;
		map<Packet,LLNODE*>TCAMmapI;
		map<int,int>flowLeaveTime;
};
#endif
