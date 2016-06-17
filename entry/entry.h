
// Headers
#include "../basic_lib.h"
#include "../packet/packet.h"

// Entry class
#ifndef ENTRY_H
#define ENTRY_H
class Entry{
	public:
		Entry(int s0 = -1, int s1 = -1, int s2 = -1, int s3 = -1,
			int d0 = -1, int d1 = -1, int d2 = -1, int d3 = -1, 
			int out = 0, int pri = 0, unsigned int sp = -1,
			unsigned int dp = -1, unsigned int pro = -1);

		// Set function
		void setSrcMask(int,int,int,int);
		void setDstMask(int,int,int,int);
		void setSrcPort(unsigned int);
		void setDstPort(unsigned int);
		void setProtocol(unsigned int);
		void setOutputPort(int);
		void setPriority(int);
		void setSID(int);
		void setRate(double);
		void setExpire(int);

		// Get function
		int getOutputPort(void);
		int getPriority(void);
		int getSID(void);
		double getRate(void);
		Packet getSample(void);
		bool isMatch(Packet);
		void isWireless(bool);
		bool isWireless(void);
		bool isExpired(int);
	private:
		int sid;
		int srcMask[4];
		int dstMask[4];
		int srcPort;
		int dstPort;
		int protocol;
		int outputPort;
		int priority;
		double rate;
		bool wireless;
		int expireTime;
};
#endif
