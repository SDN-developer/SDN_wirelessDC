
// Event class
#include "../event/event.h"

// Event functions
double Event::getTimeStamp(void){
	return this->timeStamp;
}
int Event::getID(void){
	return this->id;
}
int Event::getEventType(void){
	return this->eventType;
}
Packet Event::getPacket(void){
	return this->packet;
}
Entry Event::getEntry(void){
	return this->ent;
}
