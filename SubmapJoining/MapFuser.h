#ifndef MAPFUSER
#define MAPFUSER

#include <iostream.h>
#include <cmath>
#include <exception>
#include <fstream>
#include "GlobalMap.h"
#include "LocalMap.h"
#include "MatrixFunctions.h"

using namespace std;

class MapFuser{
	public:
		MapFuser();
		GlobalMap glb_map;
		void fuse_map(LocalMap m);
		//void nearest_neightbour(SparseMatrix bec)
		int assosiation_matches[30];
		
};

#endif
