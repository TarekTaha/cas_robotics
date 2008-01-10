#include <iostream>
#include <stdlib.h>
#include "MatrixFunctions.h"
#include "GlobalMap.h"
#include "LocalMap.h"
#include "MapFuser.h"

using namespace std;



int main(int argc, char * argv[])
{
	MapFuser fuser;
	LocalMap locMap;
	locMap.P.read_from_file("SimulationData/localmap_1_P");
	fuser.fuse_map(locMap);
	return 1;
}
