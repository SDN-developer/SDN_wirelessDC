// Header
#include "fattree.h"
#include "../basic_lib.h"
#include "../prevhop.h"

// Modify capacity
void Fattree::modCap(int nid, int seq, double rate){

	// Check if previous exists
	PrevHop pp;
	map<int,PrevHop>::iterator it;
	it = this->prevHop.find(seq);
	if(it == this->prevHop.end()) return;
	else pp = it->second;

	// Wired
	int port, nxt;
	if(pp.tranType == PREV_WIRED){
		node[pp.id]->link[pp.port].cap += rate;
		nxt = node[pp.id]->link[pp.port].id;
		for(port = 0; port < node[nxt]->link.size(); port++)
			if(node[nxt]->link[port].id == pp.id) break;
		node[nxt]->link[port].cap += rate;
	}

	// WTF??
	else{
		fprintf(stderr, "(Error) Unknown previous hop link type\n");
	}
}
