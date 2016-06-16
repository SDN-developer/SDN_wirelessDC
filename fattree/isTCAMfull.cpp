// Headers
#include "fattree.h"
#include "../parameter.h"

// Test if TCAM is full
bool Fattree::isTCAMfull(const vector<Entry>& nodes, bool isWired){

	// Variables
	int numOfFull;
	int usage;
	int sid;

	// Wired, ignore "nodes", check all core & aggr
	if(isWired){

		/********** Modification Note **********
		 Actually, this can be done by maintaining 2 variables,
		 usageCore and usageAggr. Whenever new rules are inserted
		 /remove into/from these TCAMs, update the value. O(1).
		*********** Modification Note **********/

		// Count full nodes
		numOfFull = 0;
		for(int i = 0; i < numberOfCore + numberOfAggregate; i++){

			// Extract TCAM usage
			usage = (sw[i]->TCAMactive.size() + sw[i]->TCAMinactive.size());
			if(usage >= THR_TCAM_FULL*MAX_TCAM_ENTRY) numOfFull ++;
		}

		// Exceed ratio
		if((numOfFull+0.0)/(numberOfCore+numberOfAggregate) > THR_WIRED)
			return true;
		return false;
	}

	// Wireless, check "nodes" only
	else{

		// Count full nodes
		numOfFull = 0;
		for(int i = 0; i < nodes.size(); i++){

			// Extract TCAM usage
			sid = nodes[i].getSID();
			usage = (sw[sid]->TCAMactive.size() + sw[sid]->TCAMinactive.size());
			if(usage >= THR_TCAM_FULL*MAX_TCAM_ENTRY) numOfFull ++;
		}

		// Exceed ratio
		if((numOfFull+0.0)/nodes.size() > THR_WIRELESS)
			return true;
		return false;
	}
}
