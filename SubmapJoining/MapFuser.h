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
		void fuse_first_map(LocalMap m);
		void set_potential_assosiations();
		Matrix get_part_of_X_for_assositation();
		Matrix restore_part_of_P_for_assositation();
		Matrix trans_cov_matrix_to_local_cordinate_system(const Matrix& P, const Matrix& X);
		Matrix trans_state_matrix_to_local_cordinate_system(const Matrix& X);
		void assosiate_beacons(const Matrix& beacX, const Matrix& beacP, const Matrix& obsX, const Matrix& obsP);

		//void nearest_neightbour(SparseMatrix bec)
		int potential_assosiation_beacons[100];
		int num_potential_assosiations;
		int assosiations[100];
		int assosiation_matches[30];
		int num_assositations;
		int submaps_first_beacon[1000];
		int place_of_beacon[10000];
		int num_submaps;
		int num_beacons;
		
};

#endif
