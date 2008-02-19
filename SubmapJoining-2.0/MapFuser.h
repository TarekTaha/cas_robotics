#ifndef MAPFUSER
#define MAPFUSER

#include <iostream.h>
#include <cmath>
#include <exception>
#include <fstream>
#include "GlobalMap.h"
#include "LocalMap.h"
#include "MatrixFunctions.h"
#include "association.h"

static int MAX_L22_DIMENSION;
static int REORDER_AMD;
static double SIGMA_MULTIPLIER;
static double EST_BIAS;
static int REORDER_AMD_FREQUENCY;
using namespace std;

class MapFuser{
	public:
		MapFuser();
		GlobalMap glb_map;
		void order_for_comparision();
		void update_map(SparseMatrix& obsX, SparseMatrix& obsP);
		void fuse_map(LocalMap m);
		void fuse_first_map(LocalMap m);
		void set_potential_associations();
		void add_new_beacons_and_robot_location_to_state(const SparseMatrix& obsX, const SparseMatrix& true_index = SparseMatrix());
		SparseMatrix get_part_of_X_for_assositation();
		SparseMatrix restore_part_of_P_for_assositation();
		SparseMatrix trans_cov_matrix_to_local_cordinate_system(const SparseMatrix& P, const SparseMatrix& X);
		SparseMatrix trans_state_matrix_to_local_cordinate_system(const SparseMatrix& X);
		void assosiate_beacons(const SparseMatrix& beacX, const SparseMatrix& beacP, const SparseMatrix& obsX, const SparseMatrix& obsP);
		void check_data_association(const SparseMatrix& true_index);
		double submap_radius(const LocalMap& map);
		double distance_to_submap(int map);
		void reorder_submaps_AMD_distance();
		void reorder_submaps_AMD();
		void reorder_submaps_distance();
		Permutation reorder_submaps(int*, int);
		void compute_cholesky_factorization();
		double wrap(double angle);
		void load_params();

		/*
		Used to list the selected beacons for association. The array 
		contains the global numbers of the interesting beacons 
		after set_potential_associations().
		*/
		int potential_association_beacons[1000];
		int num_potential_associations;
		
		/*
		used to list the associations, 
		-100 means new beacon 
		-1 poor observation
		>0 the global number of the beacon associated with
		*/
		int associations[1000];
		
		//the number of associations that's not -1 or -100
		int num_matches;

		//The global value of the submaps first beacon
		int submaps_first_beacon[1000];
		
		//The number of beacons in the submap
		int num_beacons_in_submap[1000];
		
		/*
		The index of the robot in the global state vector. It is the index of the
		x position. The y position is at index_of_robot + 1 and the bearing at
		index_of_robot + 2
		*/
		int index_of_robot[1000];
		
		//The radiuses of the submaps
		double radius_of_submap[1000];
		
		//THe index of the beacons in the global state vector
		int index_of_beacon[10000];
		
		/*
		for simulation checks only, contains the "true" index of the beacon given
		by the simulation.
		*/ 
		int true_index_of_beacon[10000];
		
		//Number of submaps fused so far
		int num_submaps;
		
		//Number of beacons in the state
		int num_beacons;
		
		double global_robot_uncertainty[1000];
		double local_robot_uncertainty[1000];
		
		/*
		The number of rows and columns updated in the information matrix. Used
		for incremental cholesky factorization.
		*/
		int num_rows_updated_in_I;
};

#endif
