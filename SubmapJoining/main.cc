#include <iostream>
#include <stdlib.h>
#include "MatrixFunctions.h"
#include "GlobalMap.h"
#include "LocalMap.h"
#include "MapFuser.h"
#include <sstream>

using namespace std;

void load_map(LocalMap& map, int index){
	stringstream out;
	out << index;
	string st_str = "SimulationData/localmap_"+ out.str() + "_st";
	string P_str = "SimulationData/localmap_"+ out.str() + "_P";
	map.P.read_from_file(P_str.c_str());
	map.X.read_from_file(st_str.c_str());
}

int main(int argc, char * argv[])
{
	MapFuser fuser;
	LocalMap locMap;
	load_map(locMap, 1);
	fuser.fuse_first_map(locMap);
	for(int i = 2; i <= 15; ++i){
		load_map(locMap, i);
		fuser.fuse_map(locMap);
	}
	fuser.glb_map.I.write_to_file("SavedMatrices/I");
	cout << "done fusing" << endl;


	return 1;
}
