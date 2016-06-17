
// Header file
#include "basic_lib.h"
#include "fattree/fattree.h"

// Main
int main(int argc, char *argv[])
{
	// Random seed
	srand((unsigned)atoi(argv[1]));

	// Fat tree
	Fattree myFatTree(SIZE_OF_K);

	// Info
	printf("Total: %d\n", myFatTree.getNumberOfNode());
	printf("Core: %d\n", myFatTree.getNumberOfCore());
	printf("Aggregate: %d\n", myFatTree.getNumberOfAggregate());
	printf("Edge: %d\n", myFatTree.getNumberOfEdge());
	printf("Host: %d\n", myFatTree.getNumberOfHost());

	// Read Input
	myFatTree.readInput();
	
	// Start simulation
	myFatTree.start();
	return 0;
}
