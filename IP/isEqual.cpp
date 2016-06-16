
// Headers
#include "../IP/IP.h"

// Check if two IP is equal
bool IP::isEqual(IP secIP){

	// Check for bytes
	for(int i = 0; i < 4; i++)
		if(this->byte[i] != secIP.byte[i])
			return false;
	return true;
}
