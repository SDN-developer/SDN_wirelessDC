
// Basic Library
#include "../basic_lib.h"

// IP class
#ifndef IP_H
#define IP_H
class IP{
	public:
		IP(void);
		IP(string);
		IP(int,int,int,int);
		void setIP(string);
		void setIP(int,int,int,int);
		bool isEqual(IP);
		int byte[4];
		string fullIP;
};
#endif
