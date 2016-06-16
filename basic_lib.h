#ifndef BASIC_LIB
#define BASIC_LIB
#include <cstdio>
#include <cstring>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <queue>
#include <vector>
#include <string>
#include <algorithm>
#include <map>
#include "parameter.h"
#include "mypair.h"
#define myAbs(x) ((x)>0?(x):(-(x)))
#define myMax(a,b) ((a)>(b)?(a):(b))
#define myMin(a,b) ((a)<(b)?(a):(b))
#define myDis(x1,y1,x2,y2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
using namespace std;
typedef struct{
	int id;
	double cap;
}EDGE;
// Linked list node Structure
#include "entry/entry.h"
struct gNode{
	Entry ent;
	struct gNode *prev;
	struct gNode *next;
};
typedef struct gNode LLNODE;
#endif
