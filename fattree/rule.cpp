
#include "../basic_lib.h"
#include "../fattree/fattree.h"
#include "../entry/entry.h"
#include "../IP/IP.h"
#include "../packet/packet.h"

// Extract rule
bool Fattree::rule(int nid, vector<Entry>flowEnt, Entry &ent){

	// Return corresponding rule to switch nid
	for(int i = 0; i < flowEnt.size(); i++)
		if(nid == flowEnt[i].getSID()){
			ent = flowEnt[i];
			return true;
		}
	return false;
}
