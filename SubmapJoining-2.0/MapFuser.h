#ifndef MAPFUSER
#define MAPFUSER

#include <iostream.h>
#include <cmath>
#include <exception>
#include <fstream>
#include "GlobalMap.h"
#include "LocalMap.h"
#include "MatrixFunctions.h"
#include "Timer.h"

static int MAX_L22_DIMENSION;
static double SIGMA_MULTIPLIER;
static double EST_BIAS;
static double CHI2_CONFIDENCE_NN;

using namespace std;

class MapFuser{
	public:
		MapFuser();
		GlobalMap glb_map;
		SparseMatrix order_for_comparision();
		void update_map(const SparseMatrix& obsX, const SparseMatrix& obsP);
		void fuse_map(LocalMap m);
		void fuse_first_map(LocalMap m);
		void set_potential_assosiations();
		void add_new_beacons_and_robot_location_to_state(const SparseMatrix& obsX);
		SparseMatrix get_part_of_X_for_assositation();
		SparseMatrix restore_part_of_P_for_assositation();
		SparseMatrix restore_part_of_P_for_assositation2();
		SparseMatrix trans_cov_matrix_to_local_cordinate_system(const SparseMatrix& P, const SparseMatrix& X);
		SparseMatrix trans_state_matrix_to_local_cordinate_system(const SparseMatrix& X);
		void assosiate_beacons(const SparseMatrix& beacX, const SparseMatrix& beacP, const SparseMatrix& obsX, const SparseMatrix& obsP);
		double submap_radius(const LocalMap& map);
		double distance_to_submap(int map);
		void reorder_submaps_AMD();
		void reorder_submaps_distance();
		void reorder_submaps(int*, int);
		void reorder_submaps(double*, int);
		void compute_cholesky_factorization();
		double wrap(double angle);
		void load_params();

		//void nearest_neightbour(SparseMatrix bec)
		int potential_assosiation_beacons[1000];
		int num_potential_assosiations;
		int assosiations[1000];
		int assosiation_matches[300];
		int num_matches;
		int num_assositations;
		int submaps_first_beacon[1000];
		int num_beacons_in_submap[1000];
		int index_of_robot[1000];
		double radius_of_submap[1000];
		int index_of_beacon[10000];
		int num_submaps;
		int num_beacons;
		double global_robot_uncertainty[1000];
		double local_robot_uncertainty[1000];
		int num_elements_updated_in_I;
		
		Timer timer;
};

#endif
