
// Headers
#include "../basic_lib.h"
#include "../node/node.h"
#include "../event/event.h"
#include "../switch/switch.h"
#include "../packet/packet.h"
#include "../prevhop.h"

// Fat Tree class
#ifndef FATTREE_H
#define FATTREE_H
class Fattree{
	public:
		// Public method
		Fattree(int);					// Constructor
		void readInput(void);			// Read the input packets 	
		void start(void);				// Start simulation
		int getNumberOfNode(void);		// Get number of nodes
		int getNumberOfCore(void);		// Get number of core switches
		int getNumberOfAggregate(void);	// Get number of aggregate switches
		int getNumberOfEdge(void);		// Get number of edge switchs
		int getNumberOfHost(void);		// Get number of hosts
		int getNumberOfPod(void);		// Get number of Pods
		Node **getNodePtr(void);		// Get node pointer

		// Public data
		vector< vector< vector<int> > > wlPath;	// Wireless paths

	private:
		// Private data
		int pod;						// Number of pods
		int totalNode;					// Number of nodes
		int numberOfCore;				// Number of core switches
		int numberOfAggregate;			// Number of aggregate switches
		int numberOfEdge;				// Number of edge switches
		int numberOfHost;				// Number of hosts
		int maxEntry;					// Maximum number of TCAM entries
		int flowIDCount;				// Current flow ID count
		double compAvail;				// Controller available time (not used)
		Node **node;					// All nodes
		Switch **sw;					// All switches
		priority_queue<Event>eventQueue;	// Event queue
		map<Packet,int>rcdFlowID;			// Flow ID of a packet
		vector< vector<Entry> > allEntry;	// Flow entries of some flows
		vector<Event>cumQue;			// Cumulated event queue
		map<int,PrevHop>prevHop;		// Previous hop
		map<Packet,int>aliveFlow;		// Number of alive flow currently in the network

		// New added variable
		int totFlow;					// Record total number of flows
		map<int, vector<int> >headerList;	// Record flowID for the same header
		map<int, int>flowCompTime;		// Record flow completion time of flowID

		// Private method
		void controller(Event);			// Handles a batch of flow setup requests
		void install(Event);			// Install rules into switch
		void cumulate(Event);			// Cumulate events until timeout
		bool legalAddr(IP);				// Check if address is legal
		void wirelessSP(void);			// Pre-process wireless shortest path
		bool rule(int,vector<Entry>,Entry&);	// Extract rule from flow path
		int pathInit(Packet,map<int,int>&);		// Initialize the prev array with -1
		bool wired(int,Packet,vector<Entry>&,int);		// Wired policy
		bool wireless(int,Packet,vector<Entry>&,int);	// Wireless policy
		double vecdot(double[],double[],double[],double[]);	// Calculate vector dot
		double vecdis(double[],double[],double[],double[]);	// Calculate vector distance
		void updateTCAM(int,int);		// Remove expired entries
		void recrdPrev(Event,Event);	// Record previous hop information
		void modCap(int,int,double);	// Release capacity of previous hop
		bool blockFlow(Event,Event);	// Check if capacity is enough or not
		void resumeFlow(int,int);		// Resume blocked flow
		int wiredHop(Packet);			// Calculate hops if using wired path
		int wirelessHop(Packet);		// Calculate hops if using wireless path
		void begTransmission(double,Packet);	// Called when transmission starts
		void endTransmission(double,Packet);	// Called when transmission finishes
		bool isTCAMfull(const vector<Entry>& nodes, bool isWired);	// Test if TCAM is "Full"

		// Metric
		int metric_flowSetupRequest;
		int metric_ruleInstallCount;
		double metric_avgFlowCompleteTime;
		map<int,double>metric_flowArrivalTime;
		int ruleReplacementCore;
		int ruleReplacementAggr;
		int ruleReplacementEdge;
};
#endif
