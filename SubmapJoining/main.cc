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
	locMap.X.read_from_file("SimulationData/localmap_1_st");
	fuser.fuse_first_map(locMap);
	
	locMap.P.read_from_file("SimulationData/localmap_2_P");
	locMap.X.read_from_file("SimulationData/localmap_2_st");
	fuser.fuse_map(locMap);
	/*locMap.P.read_from_file("SimulationData/localmap_3_P");
	locMap.X.read_from_file("SimulationData/localmap_3_st");
	fuser.fuse_map(locMap);
	locMap.P.read_from_file("SimulationData/localmap_4_P");
	locMap.X.read_from_file("SimulationData/localmap_4_st");
	fuser.fuse_map(locMap);
	to_dence_matrix(fuser.glb_map.I).write_to_file("SavedMatrices/I.mat");*/


	return 1;
}
