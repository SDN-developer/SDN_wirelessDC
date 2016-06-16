// Header
#include "../basic_lib.h"
#include "fattree.h"

// Vector distance
double Fattree::vecdis(double pA[2], double pB[2], double pC[2], double pD[2]){
	double v1[2], v2[2], cross;

	// Vectors
	v1[0] = pB[0] - pA[0], v1[1] = pB[1] - pA[1];
	v2[0] = pD[0] - pC[0], v2[1] = pD[1] - pC[1];

	// Cross
	cross = v1[0]*v2[1] - v1[1]*v2[0];

	// Distance
	return myAbs(cross)/sqrt(v1[0]*v1[0]+v1[1]*v1[1]);
}
