
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"
#include "../node/node.h"
#include "../switch/core.h"
#include "../switch/aggregate.h"
#include "../switch/edge.h"
#include "../host/host.h"
#include "../fattree/fattree.h"
#include "../event/event.h"
#include "../event/eventType.h"
#include "../entry/entry.h"

// Start simulation
void Fattree::start(void){

	// Until all event finished
	int arrive;
	int curTimeStamp;
	int compTimeStamp;
	Packet pkt;
	Event evt, next;
	vector<Entry>vent;

	// Statistic information
	int prevPerCent = -1, perCent;
	arrive = 0;

	// Event queue
	while(!eventQueue.empty()){

		// Get current event
		evt = eventQueue.top();
		eventQueue.pop();

		// Process event
		switch(evt.getEventType()){

			// No operation
			case EVENT_NOP:
//printf("[%6.1lf] No operation.\n", evt.getTimeStamp());
				break;

			// Check if flow transmission done
			case EVENT_CHECK:
				pkt = evt.getPacket();
				curTimeStamp = evt.getTimeStamp();
				compTimeStamp = flowCompTime[pkt.getSequence()];

				// Not known or not yet
				if(compTimeStamp == -1){
					fprintf(stderr, "[Error] Logically not possible!?\n");
					exit(1);
				}

				// Not yet finished: another check event (at destined finished time)
				if(curTimeStamp < compTimeStamp){
					next.setTimeStamp(compTimeStamp);
					next.setEventType(EVENT_CHECK);
					next.setPacket(pkt);
					eventQueue.push(next);
				}

				// Done
				else{
					next.setTimeStamp(curTimeStamp);
					next.setEventType(EVENT_DONE);
					next.setPacket(pkt);
					eventQueue.push(next);
				}
				break;

			// Cumulate until interval timeout
			case EVENT_FLOWSETUP:
//printf("[%6.1lf] Flow setup request: %d at %d.\n", evt.getTimeStamp(), evt.getPacket().getSequence(), evt.getID());
				cumulate(evt);
//				metric_flowSetupRequest ++;
				break;

			// Interval timeout: handle batch of flow setup requests
			case EVENT_INTERVAL:
				controller(evt);
				break;

			// Install & forward
			case EVENT_INSTALL:
//printf("[%6.1lf] Install: %d at %d\n", evt.getTimeStamp(), evt.getPacket().getSequence(), evt.getID());
				install(evt);
				metric_ruleInstallCount ++;
				break;

			// Flow transmission done
			case EVENT_DONE:
//printf("[%6.1lf] %d flows arrives\n", evt.getTimeStamp(), arrive);

				// Release capacity for this flow along the path
				pkt = evt.getPacket();
				vent = allEntry[ rcdFlowID[pkt] ];
				modifyCap(vent, pkt.getDataRate());
				
				// End transmission: 
				// Change active rule to inactive if all flows of current header is done
				endTransmission(evt.getTimeStamp(), evt.getPacket());

				// Percentage
				arrive ++;
				perCent = (arrive*100)/totFlow;
				if(perCent != prevPerCent){
					printf("%3d%% (%d/%d) done.\n", perCent, arrive, totFlow);
					prevPerCent = perCent;
				}
	
				// Flow arrival time
				metric_avgFlowCompleteTime += (evt.getTimeStamp() - metric_flowArrivalTime[evt.getPacket().getSequence()]);
				metric_flowArrivalTime.erase(evt.getPacket().getSequence());

				// Output metric
				if(perCent == 100){
					printf("# of flow setup request: %d\n", metric_flowSetupRequest);
					printf("# of installed rules: %d\n", metric_ruleInstallCount);
					printf("Avg. flow completion time: %.3lf\n", metric_avgFlowCompleteTime/totFlow);
					printf("Replacement: %d / %d / %d\n", ruleReplacementCore, ruleReplacementAggr, ruleReplacementEdge);
/*					printf("%d %d %.3lf %d %d %d\n", metric_flowSetupRequest, metric_ruleInstallCount, 
							metric_avgFlowCompleteTime/totFlow, ruleReplacementCore, ruleReplacementAggr, ruleReplacementEdge);*/
				}
				break;

			// Unknown
			case EVENT_UNKNOWN:
				printf("Error: unknown operation found.\n");
				break;
		}
	}
}
