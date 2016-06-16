
// Headers
#include "../packet/packet.h"
#include "../IP/IP.h"
#include "../entry/entry.h"

// Event class
#ifndef EVENT_H
#define EVENT_H
class Event{
	public:
		double getTimeStamp(void);
		int getEventType(void);
		int getID(void);
		Packet getPacket(void);
		Entry getEntry(void);
		void setTimeStamp(double);
		void setEventType(int);
		void setID(int);
		void setPacket(Packet);
		void setEntry(Entry);
		bool operator<(const Event&)const;
	private:
		int eventType;
		double timeStamp;
		int id;
		Packet packet;
		Entry ent;
};
#endif
