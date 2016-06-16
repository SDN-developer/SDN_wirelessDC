
// Event class
#include "../event/event.h"

// Event functions
void Event::setTimeStamp(double tt){
	this->timeStamp = tt;
}
void Event::setID(int id){
	this->id = id;
}
void Event::setEventType(int et){
	this->eventType = et;
}
void Event::setPacket(Packet pp){
	this->packet = pp;
}
void Event::setEntry(Entry ent){
	this->ent = ent;
}
