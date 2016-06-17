// Headers
#include "../basic_lib.h"
#include "../fattree/fattree.h"
#include "../event/event.h"

// Sort class
#ifndef MYSORT_H
#define MYSORT_H
class mySort{
	public:
		Fattree *fPtr;
		mySort(Fattree *pp):fPtr(pp){};
		bool operator()(Event,Event);
};
#endif
