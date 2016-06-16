#include "fattree.h"
#include "../basic_lib.h"

// Wireless path
void Fattree::wirelessSP(void){

	// IDs
	int srcID, dstID, nowID, nxtID;

	// SPFA
	int sft = numberOfCore + numberOfAggregate;
	int que[numberOfEdge], head, tail;
	int dis[numberOfEdge];
	int itf[numberOfEdge];
	int prev[numberOfEdge];
	bool inque[numberOfEdge];
	int tmp;

	// Paths
	vector<int>rev;
	vector<int>path;
	vector< vector<int> > rowPath;

	// Empty path for all starting node
	for(int i = 0; i < numberOfEdge; i++)
		this->wlPath.push_back(rowPath);

	// All pair shortest path
	for(int i = 0; i < numberOfEdge; i++){

		// Initialize
		for(int j = 0; j < numberOfEdge; j++)
			dis[j] = (int)1e9;
		memset(inque, false, sizeof(inque));
		memset(prev, -1, sizeof(prev));
		head = tail = 0;

		// SPFA for all destinations
		dis[i] = 0;
		itf[i] = 0;
		que[tail++] = i;
		inque[i] = true;
		while(head < tail){
			nowID = que[head%numberOfEdge];
			inque[nowID] = false;
			head ++;
			for(int z = 0; z < sw[nowID + sft]->wlink.size(); z++){
				nxtID = sw[nowID + sft]->wlink[z].id - sft;
				if(dis[nowID] + 1 < dis[nxtID]){
					dis[nxtID] = dis[nowID] + 1;
					itf[nxtID] = itf[nowID] + ((int)sw[nowID + sft]->iList[z].size());
					prev[nxtID] = nowID;
					if(!inque[nxtID]){
						que[tail%numberOfEdge] = nxtID;
						inque[nxtID] = true;
						tail ++;
					}
				}
				else if(dis[nowID] + 1 == dis[nxtID]){
					tmp = itf[nowID] + ((int)sw[nowID + sft]->iList[z].size());
					if(tmp < itf[nxtID]){
						itf[nxtID] = tmp;
						prev[nxtID] = nowID;
						if(!inque[nxtID]){
							que[tail%numberOfEdge] = nxtID;
							inque[nxtID] = true;
							tail ++;
						}
					}
					/* Multipath: elseif(tmp == itf[nxtID]) */
				}
			}
		}

		// For all destinations
		for(int j = 0; j < numberOfEdge; j++){

			// Self: empty path
			if(j == i){
				this->wlPath[i].push_back(path);
				continue;
			}

			// Recover path
			nowID = j;
			while(nowID != i){
				rev.push_back(nowID + sft);
				nowID = prev[nowID];
			}
			rev.push_back(i + sft);
			for(int z = rev.size()-1; z >= 0; z--)
				path.push_back(rev[z]);
			
			// Row path
			this->wlPath[i].push_back(path);
			
			// Clear
			path.clear();
			rev.clear();
		}
	}
}
