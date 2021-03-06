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
	locMap.P.write_to_file("SavedMatrices/Ploc");
	fuser.fuse_first_map(locMap);
	for(int i = 2; i <= 100; ++i){
		cout << "localmap: " << i <<endl;
		load_map(locMap, i);
		to_sparse_symm_matrix(locMap.P).write_to_file("SavedMatrices/Ploc");
		fuser.fuse_map(locMap);
		//if(i == 27 || i == 34 || i == 43 || i == 57){
			//fuser.reorder_submaps();
		//}
		stringstream out;
		out << i;
		string tmp = "SavedMatrices/I" + out.str();
		fuser.glb_map.I.write_to_file(tmp.c_str());
		tmp = "SavedMatrices/i" + out.str();
		fuser.glb_map.i.write_to_file(tmp.c_str());
		tmp = "SavedMatrices/X" + out.str();
		fuser.glb_map.X.write_to_file(tmp.c_str());
	}
	fuser.timer.print();
	//fuser.glb_map.I.write_to_file("SavedMatrices/I");
	//fuser.glb_map.i.write_to_file("SavedMatrices/i");
	cout << "done fusing" << endl;

	typedef std::numeric_limits< long double > dl;
	typedef std::numeric_limits< float > fl;

	using namespace std;

	cout << "double:\n";
	cout << "\tdigits (bits):\t\t" << dl::digits << endl;
	cout << "\tdigits (decimal):\t" << dl::digits10 << endl;

	return 1;
}
