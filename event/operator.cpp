
// Event class
#include "../event/event.h"
#include "../event/eventType.h"

// Event functions
bool Event::operator<(const Event& e)const{
	if(this->timeStamp == e.timeStamp){

		// Flow setup request event
		EVENT_TYPE te = this->eventType;
		EVENT_TYPE ee = e.eventType;
		if(te == EVENT_FLOWSETUP && te == ee){
			Packet tp = this->packet;
			Packet ep = e.packet;
			return tp.getSequence() > ep.getSequence();
		}
		else return false;
	}
	return this->timeStamp > e.timeStamp;
}
