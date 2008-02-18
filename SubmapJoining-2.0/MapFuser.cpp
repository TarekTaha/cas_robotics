#include "MapFuser.h"

MapFuser::MapFuser(){
	num_rows_updated_in_I = 0;
	load_params();
}

/*
fuse the current (not the first) local map into global map
*/
void MapFuser::fuse_map(LocalMap m){
	//add a bit to P to avoid singularity
	if(m.P.get(1,1) < 1e-8 || m.P.get(2,2) < 1e-8 || m.P.get(3,3) < 1e-8){
		m.P.set(1,1, m.P.get(1,1) + 1e-8);
		m.P.set(2,2, m.P.get(2,2) + 1e-8);
		m.P.set(3,3, m.P.get(3,3) + 1e-8);
	}
	
	radius_of_submap[num_submaps] = submap_radius(m);

	SparseMatrix obsP = m.P.get_submatrix(4,4, m.P.get_rows(), m.P.get_cols());
	SparseMatrix obsX = m.X.get_submatrix(4, 1, m.X.get_rows(), 1);
	set_potential_assosiations();
	SparseMatrix P = restore_part_of_P_for_assositation();
	global_robot_uncertainty[num_submaps] = SIGMA_MULTIPLIER * max_eig(sqrt(to_sparse_symm_matrix(P.get_submatrix(1, 1, 2, 2))));
	local_robot_uncertainty[num_submaps] = SIGMA_MULTIPLIER * max_eig(sqrt(to_sparse_symm_matrix(m.P.get_submatrix(1, 1, 2, 2))));

	SparseMatrix X = get_part_of_X_for_assositation();
	SparseMatrix beacP = trans_cov_matrix_to_local_cordinate_system(P,X);
	SparseMatrix beacX = trans_state_matrix_to_local_cordinate_system(X);
	
	assosiate_beacons(beacX, beacP, obsX, obsP);
	check_data_assosiation(m.true_index);
	
	add_new_beacons_and_robot_location_to_state(m.X, m.true_index);

	update_map(m.X, m.P);

	//reodrder and calculate choleskyfactorization depending on the method choosen
	if(REORDER_AMD == 0){
		if(num_rows_updated_in_I > MAX_L22_DIMENSION){
			cout << "Reorder distance " << num_rows_updated_in_I << endl;
			reorder_submaps_distance();
			glb_map.L = cholesky(glb_map.I);
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
		else{
			compute_cholesky_factorization();
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
	}
	else if(REORDER_AMD == 1){
		if(num_submaps%REORDER_AMD_FREQUENCY == 0){
			cout << "Reorder AMD" << endl;
			reorder_submaps_AMD();
		}
		glb_map.L = cholesky(glb_map.I);
		glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
	}
	else if(REORDER_AMD == 2){
		if(num_rows_updated_in_I > MAX_L22_DIMENSION){
			cout << "Reorder AMD distance" << endl;
			reorder_submaps_AMD_distance();
			glb_map.L = cholesky(glb_map.I);
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
		else{
			compute_cholesky_factorization();
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
	}
}


/*
checks if the data assisiation is correct. Can only be used in 
simulations where the true indexes of the beacons are provided.
*/
void MapFuser::check_data_assosiation(const SparseMatrix& true_index){
	int num_obs = (true_index.get_rows() - 3)/2;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			for(int j = 0; j < num_beacons; ++j){
				if(true_index_of_beacon[ j] == true_index.get(2*i + 4, 1)){
					cout << "Wrong data assosiation: the beacon with index " << true_index.get(2*i + 4, 1) << " has already been observed but was assosiated as a new beacon" << endl;
					break;
				}
			}
		}
		else if(assosiations[i] == -1){
			cout << "Wrong data assosiation: poor obeservation" << endl;
		}
		else{
			if(true_index_of_beacon[ assosiations[i]] != true_index.get(2*i + 4, 1)){
				cout << "Wrong data assosiatio: wrong match, beacon with true index " << true_index.get(2*i + 4, 1) << " was matched with beacon with true index "<< true_index_of_beacon[ assosiations[i]] << endl;
			}
		}
	}
}

/*
Only used to reorder the information matrix, state 
vector and information vector for comparision with matlab.
*/
void MapFuser::order_for_comparision(){
	int order[10000];
	int index = 0;
	for(int i = 0; i < num_submaps; ++i){
		order[index] = i - num_submaps;
		++index;
	}
	for(int i = 0; i < num_beacons; ++i){
		order[index] = i;
		++index;
	}
	Permutation p = reorder_submaps(order, index);
	glb_map.X = glb_map.X.get_submatrix(p, no_reorder(1));
}

/*
Derives a pratial cholesky factorization based on that only the num_rows_updated_in_I
last rows in the information matrix was changed.
*/
void MapFuser::compute_cholesky_factorization(){
	/*derives the diffrent parts of the cholesky factorization and puts them together
	 	[spaL11,      0]
	 	[spaL21, spaL22]
	*/
	
	if(glb_map.I.get_rows() != num_rows_updated_in_I){
		SparseMatrix spaL = to_sparse_matrix(glb_map.L);
		SparseMatrix spaL21 = spaL.get_submatrix(glb_map.I.get_rows() - num_rows_updated_in_I + 1, 1, glb_map.L.get_rows(), glb_map.I.get_cols() - num_rows_updated_in_I);
		spaL21 = vertcat(spaL21, zeros(num_rows_updated_in_I - spaL21.get_rows(), spaL21.get_cols()));
		SparseMatrix spaL11 = spaL.get_submatrix(1, 1, glb_map.I.get_rows() - num_rows_updated_in_I, glb_map.I.get_rows() - num_rows_updated_in_I);
		SparseMatrix I22 = to_sparse_matrix_fast(glb_map.I).get_submatrix(glb_map.I.get_rows() -  num_rows_updated_in_I + 1, glb_map.I.get_cols() -  num_rows_updated_in_I + 1, glb_map.I.get_rows(), glb_map.I.get_cols());
		SparseMatrix spaL22 = to_sparse_matrix(cholesky(to_sparse_symm_matrix(I22 - aat(spaL21))));
		glb_map.L = to_factor(vertcat(horzcat(spaL11, zeros(spaL11.get_rows(), glb_map.I.get_cols() - spaL11.get_cols())), horzcat(spaL21, spaL22)));
	}
	else{
		glb_map.L = cholesky(glb_map.I);
	}
}

/*
Input: an angle
Output: an angle in the interval -pi to pi
*/
double MapFuser::wrap(double angle){
	double PI = 3.14159265358979;
	while(angle < PI){
		angle += 2*PI;
	}
	while(angle > PI){
		angle -= 2*PI;
	}
	return angle;
}


/* 
reorder the submaps in the global state vector and the global information
vector --- to make the iterative Cholesky factorization more efficient
and (possibly) reduce the fill-in

This function reorders the right lower MAX_L22_DIMENSION x MAX_L22_DIMENSION matrix 
by distance and the rest of the matrix with AMD reordering 
*/
void MapFuser::reorder_submaps_AMD_distance(){
	double dist[10000];
	int order_submaps[10000];
	int order_detail[10000];
	int index_detail;
	int set[10000];
	int temp_index;
	double temp_dist;
	int temp_order;
	Permutation perm(glb_map.I_small.get_rows());
	
	// --- reorder by distance ---

	//calculate distances to the submaps
	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order_submaps[i] = i;
	}
	
	//sort the submaps by distance
	inc_quicksort_dd1_ci1(dist, order_submaps, 0, num_submaps - 1);

	/*
	Create an array describing the new order of the state vector. The submaps are 
	represented by a negative number from -1 to -"number of submaps". The beacons 
	are represented with positive numbers from 0 to "number of beacons" - 1. 
	*/
	index_detail = 0;
	for(int i = 0; i < num_submaps; ++i){
		for(int j = 0; j < num_beacons_in_submap[order_submaps[i]]; ++j){
			order_detail[index_detail] = submaps_first_beacon[order_submaps[i]] + j;
			++index_detail;
		}
		order_detail[index_detail] = -order_submaps[i] - 1;
		++index_detail;
	}
	
	//derive the permutation that changes current the order into the new order
	perm = sorting_permutation(glb_map.order, -num_submaps, num_beacons - 1) + inv(sorting_permutation(order_detail, -num_submaps, num_beacons - 1));
	Permutation p2 = no_reorder(1); 

	//permute the small informationmatrix to that order
	glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(perm, perm));
	permute(glb_map.order,perm);

	//--- end reorder by distance ---
	
	
	//--- reorder by AMD ---
	
	//reorder the upper left part of the small information matrix by AMD
	if(glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2 > 1){
		SparseSymmMatrix part_I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(1,1, glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2, glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2));
		Permutation p = reorder_AMD(part_I_small);
		for(int i = 0; i < part_I_small.get_rows(); ++i){
			perm.p[i] = p.p[i];
		}
		for(int i = part_I_small.get_rows(); i < glb_map.I_small.get_rows(); ++i){
			perm.p[i] = i;
		}
		glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(perm, perm));
		permute(glb_map.order, perm);
	}
	
	//--- end reorder by AMD ---
	
	//Do the change in order on the actual (big) state and information matrix
	reorder_submaps(glb_map.order, glb_map.I_small.get_rows());
}

/* 
reorder the submaps in the global state vector and the global information
vector --- to make the iterative Cholesky factorization more efficient
and (possibly) reduce the fill-in

This function does the reodering using AMD only
*/
void MapFuser::reorder_submaps_AMD(){
	Permutation p = reorder_AMD(glb_map.I_small);
	Permutation p2 = no_reorder(1);
	glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(p, p));
	permute(glb_map.order, p);
	reorder_submaps(glb_map.order, glb_map.I_small.get_rows());
}

/* 
reorder the submaps in the global state vector and the global information
vector --- to make the iterative Cholesky factorization more efficient
and (possibly) reduce the fill-in

This function reorders by distance only
*/
void MapFuser::reorder_submaps_distance(){
	double dist[10000];
	int order_submaps[10000];
	int order_detail[10000];
	int index_detail;
	int set[10000];
	int temp_index;
	double temp_dist;
	int temp_order;

	
	//calculate distances to the submaps
	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order_submaps[i] = i;
	}
	
	//sort the submaps by distance
	inc_quicksort_dd1_ci1(dist, order_submaps, 0, num_submaps - 1);

	/*
	Create an array describing the new order of the state vector. The submaps are 
	represented by a negative number from -1 to -"number of submaps". The beacons 
	are represented with positive numbers from 0 to "number of beacons" - 1. 
	*/
	index_detail = 0;
	for(int i = 0; i < num_submaps; ++i){
		for(int j = 0; j < num_beacons_in_submap[order_submaps[i]]; ++j){
			order_detail[index_detail] = submaps_first_beacon[order_submaps[i]] + j;
			++index_detail;
		}
		order_detail[index_detail] = -order_submaps[i] - 1;
		++index_detail;
	}
	
	//Do the change in order on the state and information matrix
	reorder_submaps(order_detail, num_beacons + num_submaps);
}

/* 
reorder the submaps in the global state vector and the global information
vector --- to make the iterative Cholesky factorization more efficient
and (possibly) reduce the fill-in

Input: the order of submaps and beacons
Reorders the information matrix and information vector according to the order
int the int* array. The submaps are represented by a negative number 
from -1 to -"number of submaps". The beacons are represented with positive numbers
from 0 to "number of beacons" - 1.
*/

Permutation MapFuser::reorder_submaps(int* order, int size){
	int set[10000];
	int index = 0;
	int set2[1] = {0};
	for(int i = 0; i < size; ++i){
		if(order[i] >= 0){
			set[index] = index_of_beacon[order[i]] - 1;
			++index;
			set[index] = index_of_beacon[order[i]];
			++index;
			index_of_beacon[order[i]] = index - 1;
		}
		else{
			set[index] = index_of_robot[-order[i] - 1] - 1;
			++index;
			set[index] = index_of_robot[-order[i] - 1];
			++index;
			set[index] = index_of_robot[-order[i] - 1] + 1;
			++index;
			index_of_robot[-order[i] - 1] = index - 2;
		}
	}
	
	Permutation p(index, set);
	Permutation p2 = no_reorder(1);
	glb_map.i = glb_map.i.get_submatrix(p, p2);
	SparseMatrix temp_I = to_sparse_matrix(glb_map.I);
	glb_map.I = to_sparse_symm_matrix( temp_I.get_submatrix(p, p));
	return p;
}

/*
Computes the distance from current submap to the submaps specifyed by map
*/
double MapFuser::distance_to_submap(int map){
	if(map == num_submaps - 1){
		return 0;
	}
	double rx = glb_map.X.get( index_of_robot[num_submaps - 1], 1);
	double ry = glb_map.X.get( index_of_robot[num_submaps - 1] + 1, 1);
	double mx, my;
	if(map == 0){
		mx = 0;
		my = 0;
	}
	else{
		mx = glb_map.X.get(index_of_robot[map - 1], 1);
		my = glb_map.X.get(index_of_robot[map - 1] + 1, 1);
	}
	return sqrt((rx - mx) * (rx - mx) + (ry - my) * (ry - my)); 
}

/*
Selects the subset of the beacons from the global map 
to perform data association. It's first using the radius
and distances to submaps and then checking the distance
to individual beacons.
*/
void MapFuser::set_potential_assosiations(){
	num_potential_assosiations = 0;
	double x_beac, y_beac, dist_beac;
	double rx = glb_map.X.get( index_of_robot[num_submaps - 1], 1);
	double ry = glb_map.X.get( index_of_robot[num_submaps - 1] + 1, 1);

	for(int i = 0; i < num_submaps; ++i){
		if(distance_to_submap(i) < radius_of_submap[i] + radius_of_submap[num_submaps] + global_robot_uncertainty[num_submaps - 1]+ global_robot_uncertainty[i] + EST_BIAS){
			for(int j = 0; j < num_beacons_in_submap[i]; ++j){
		        x_beac= glb_map.X.get(index_of_beacon[ submaps_first_beacon[i] + j], 1);
		        y_beac= glb_map.X.get(index_of_beacon[ submaps_first_beacon[i] + j] + 1, 1);     
		        dist_beac = sqrt((x_beac - rx) * (x_beac - rx) + (y_beac - ry) * (y_beac - ry));
		        if(dist_beac < radius_of_submap[num_submaps] + EST_BIAS){
					potential_assosiation_beacons[num_potential_assosiations] = submaps_first_beacon[i] + j;
					++num_potential_assosiations;
		        }
			}
		}
	}
}

/*
run this after select beacon for data association
 
output: the part of global map state matrix needed for the data assosiation
*/
SparseMatrix MapFuser::get_part_of_X_for_assositation(){
	int size = num_potential_assosiations;
	SparseMatrix result(2*size + 3, 1, 2*size + 3);
	result.set(1, 1, glb_map.X.get(index_of_robot[num_submaps - 1]	  , 1));
	result.set(2, 1, glb_map.X.get(index_of_robot[num_submaps - 1] + 1, 1));
	result.set(3, 1, glb_map.X.get(index_of_robot[num_submaps - 1] + 2, 1));
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			result.set(2 * i + k + 4, 1, glb_map.X.get(index_of_beacon[potential_assosiation_beacons[i]] + k, 1));
		}
	}
	return result;
}


/*
covariance sub-matrix recovery for map joining

recover this after select beacon for data association

output: the part of global map covariance matrix needed for the data assosiation
*/
SparseMatrix MapFuser::restore_part_of_P_for_assositation(){
	int size = num_potential_assosiations;
	SparseMatrix result;
	SparseMatrix rhs(glb_map.I.get_rows(), 2 * num_potential_assosiations + 3, 2 * num_potential_assosiations + 3);
	SparseMatrix x;
	int rset[2 * num_potential_assosiations + 3];
	int cset[2 * num_potential_assosiations + 3];
	int index = 0;

	for(int i = 0; i < 3; ++i){
		rhs.set(index_of_robot[num_submaps - 1] + i, index + 1, 1);
		rset[index] = index_of_robot[num_submaps - 1] + i - 1;
		cset[index] = index;
		++index;
	}
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			rhs.set(index_of_beacon[potential_assosiation_beacons[i]] + k, index + 1, 1);
			rset[index] = index_of_beacon[potential_assosiation_beacons[i]] + k - 1;
			cset[index] = index;
			++index;
		}
	}

	x = solve_cholesky(glb_map.L, rhs);
	
	result = x.get_submatrix(rset, index, cset, index);

	return result;
}
/*
Fuses the first local map into global map

input: first localmap
*/

void MapFuser::fuse_first_map(LocalMap m){
	//add a bit to P to avoid singularity
	if(m.P.get(1,1) < 1e-8 || m.P.get(2,2) < 1e-8 || m.P.get(3,3) < 1e-8){
		m.P.set(1,1, m.P.get(1,1) + 1e-8);
		m.P.set(2,2, m.P.get(2,2) + 1e-8);
		m.P.set(3,3, m.P.get(3,3) + 1e-8);
	}
	
	radius_of_submap[0] = submap_radius(m);
	int set[m.P.get_rows()];
	int set2[1] = {0};
	for(int i = 0; i < m.P.get_rows() - 3; ++i){
		set[i] = i + 3;
	}
	set[m.P.get_rows() - 3] = 0;
	set[m.P.get_rows() - 2] = 1;
	set[m.P.get_rows() - 1] = 2;
	m.P = m.P.get_submatrix(set, m.P.get_rows(), set, m.P.get_rows());
	m.X = m.X.get_submatrix(set, m.P.get_rows(), set2, 1);
	
	global_robot_uncertainty[0] = 3 * max_eig(sqrt(to_sparse_symm_matrix(m.P.get_submatrix(1, 1, 2, 2))));
	local_robot_uncertainty[0] = 3 * max_eig(sqrt(to_sparse_symm_matrix(m.P.get_submatrix(1, 1, 2, 2))));
	
    glb_map.I = inv(to_sparse_symm_matrix(m.P));
    glb_map.i = inv(to_sparse_symm_matrix(m.P))* m.X;
    glb_map.L = cholesky(glb_map.I);
    

    glb_map.X = m.X;
    
    //if(AMD)
    glb_map.I_small = to_sparse_symm_matrix(ones((glb_map.I.get_rows() - 1)/2, (glb_map.I.get_rows() - 1)/2));
    for(int i = 1; i <= (glb_map.I.get_rows() - 1)/2; ++i){
    	glb_map.I_small.set(i, i, 100);
    }
    for(int i = 0; i < glb_map.I.get_rows() - 1; ++i){
    	glb_map.order[i] = i;
    }
    glb_map.order[glb_map.I_small.get_rows() - 1] = -1;
    
    
    num_submaps = 1;
    num_beacons = (m.P.get_rows() - 3)/2;
    num_beacons_in_submap[0] = num_beacons;
    for(int i = 0; i <= num_beacons; ++i){
    	index_of_beacon[i] = 2 * i + 1;
    	true_index_of_beacon[i] = (int)m.true_index.get(2 * i + 4, 1);
    }
    index_of_robot[0] = m.P.get_rows() - 2;
    submaps_first_beacon[0] = 0;
    global_robot_uncertainty[0] = 0;
}

/*
Transfers the covariance matrix into the local maps cordinate system.

Input: Covariance matrix and state matrix. The local maps robot posistion 
needs to be first in the matrices.

Output: The transfered covariance matrix
*/
SparseMatrix MapFuser::trans_cov_matrix_to_local_cordinate_system(const SparseMatrix& P, const SparseMatrix& X){
	//robot pos is first in X and will be (0,0,0) in the new system 
	double xr = X.get(1, 1);
	double yr = X.get(2, 1);
	double fir = X.get(3, 1);
	SparseMatrix jh(X.get_rows() - 3, X.get_rows(), 10 * num_beacons);
	int num_beacons = (X.get_rows() -3)/2;
	double dx, dy;
	for(int i = 1; i <= num_beacons; ++i){
		dx = X.get(3 + 2*i - 1, 1) - xr;
		dy = X.get(3 + 2*i, 1) - yr;
		jh.set(2*i - 1, 1, 	-cos(fir));
		jh.set(2*i - 1, 2, 	-sin(fir));
		jh.set(2*i - 1, 3, 	-dx * sin(fir) + dy*cos(fir));
		jh.set(2*i,		1, 	sin(fir));
		jh.set(2*i,		2, 	-cos(fir));
		jh.set(2*i,		3, 	-dx * cos(fir) - dy * sin(fir));
		
		jh.set(2 * i - 1, 2 * i + 2, cos(fir));
		jh.set(2 * i - 1, 2 * i + 3, sin(fir));
		jh.set(2 * i	, 2 * i + 2, -sin(fir));
		jh.set(2 * i	, 2 * i + 3, cos(fir));
	}

	return jh*P*trn(jh);
}

/*
Transfers the state matrix into the local maps cordinate system.

Input: State matrix. The local maps robot posistion 
needs to be first in the matrix.

Output: The transfered state matrix
*/
SparseMatrix MapFuser::trans_state_matrix_to_local_cordinate_system(const SparseMatrix& X){
	
	double xr = X.get(1, 1);
	double yr = X.get(2, 1);
	double fir = X.get(3, 1);
	int num_beacons = (X.get_rows() -3)/2;
	double dx, dy;
	SparseMatrix result(X.get_rows() - 3, 1, X.get_rows() - 3);
	for(int i = 1; i <= num_beacons; ++i){
		dx=X.get(2*i + 2, 1) - xr;
		dy=X.get(2*i + 3, 1) - yr;    

		result.set(2*i-1, 1, cos(fir)*dx+sin(fir)*dy);
		result.set(2*i, 1, -sin(fir)*dx+cos(fir)*dy);
	}
	return result;
}

void MapFuser::assosiate_beacons(const SparseMatrix& beacX, const SparseMatrix& beacP, const SparseMatrix& obsX, const SparseMatrix& obsP){
	int num_obs = obsP.get_rows()/2;
	int num_beac = beacP.get_rows()/2;
	double r_obs, b_obs, r_beac, b_beac;
	SparseSymmMatrix cov_obs, cov_beac, totalcov;
	SparseMatrix innov(2,1, 2);
	SparseMatrix trninnov(1,2, 2);
	double mahadist_min, mahadist_new;
	int mahadist_min_index, dist_min_index;
	double dist_min, dist_new;
	num_matches = 0;
	

	for(int j = 1; j <= num_obs; ++j){
		mahadist_min = 1e8;
		dist_min = 1e8;

	    r_obs = obsX.get(j*2-1, 1);
	    b_obs = obsX.get(j*2, 1);

	    cov_obs = to_sparse_symm_matrix(obsP.get_submatrix(j*2-1,j*2-1,j*2,j*2));  // covariance matrix of the j-th obs
	    for(int i = 1; i <= num_beac; ++i){
	        r_beac = beacX.get(i*2-1, 1);
	        b_beac = beacX.get(i*2, 1);
	        cov_beac = to_sparse_symm_matrix(beacP.get_submatrix(i*2-1,i*2-1,i*2,i*2));  // covariance matrix of the i-th beac

	        // compute Mahalanobis distance from j-th obs to i-th beac
	        innov.set(1,1, r_beac-r_obs);
	        innov.set(2,1, b_beac-b_obs);
	        trninnov.set(1,1, r_beac-r_obs);
	        trninnov.set(1,2, b_beac-b_obs);

	        totalcov = cov_beac + cov_obs;
	        dist_new = aat(trninnov).get(1,1);
	        mahadist_new  = (trninnov*inv(totalcov)*innov).get(1,1);


	        if(mahadist_new < mahadist_min){
	        	mahadist_min = mahadist_new;
	        	mahadist_min_index = i;
	        }
	        if(dist_new < dist_min){
	        	dist_min_index = i;
	        	dist_min = dist_new;
	        }
	    }
    	if(mahadist_min < CHI2_CONFIDENCE_NN && dist_min < 3 && mahadist_min_index == dist_min_index){
    		++num_matches;
    		assosiations[j - 1] = mahadist_min_index - 1;
    	}
    	else if(dist_min < 0.5 && mahadist_min_index == dist_min_index){
    		++num_matches;
    		assosiations[j - 1] = mahadist_min_index - 1;
    	}
    	else if(dist_min < 1.5){ //true dist
    		assosiations[j - 1] = -1;
    	}
    	else{
    		assosiations[j - 1] = -100;
    	}
	}
	
	//transfer assosiations to global index
	for(int j = 0; j < num_obs; ++j){
		if(assosiations[j] >= 0)
			assosiations[j] = potential_assosiation_beacons[assosiations[j]];
	}
}

/*
Adds the new beacons (beacons that got assosiation -100) to the state. 
If it is a simulation the "true" index of the beacons can be provided 
to make it possible to check the data assosiation. 

Input: State of the observed beacons. The robot posistion must be last. Optionaly the "true" index of the beacons.
*/
void MapFuser::add_new_beacons_and_robot_location_to_state(const SparseMatrix& obsX, const SparseMatrix& true_index){
	int num_obs = (obsX.get_rows() - 3)/2;

	double xr1 = glb_map.X.get(index_of_robot[num_submaps - 1]		, 1);
	double yr1 = glb_map.X.get(index_of_robot[num_submaps - 1] +  1	, 1);
	double fir1 = glb_map.X.get(index_of_robot[num_submaps - 1]	+ 2	, 1);
	double xj;
	double yj;
	int num_new = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			++num_new;
		}
	}
	SparseMatrix newX(2 * num_new + 3, 1, 2 * num_new + 3);

	submaps_first_beacon[num_submaps] = num_beacons;
	num_beacons_in_submap[num_submaps] = 0;
	int j = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			xj = obsX.get(2*i + 4, 1);
			yj =  obsX.get(2*i + 5, 1);
			newX.set(2 * j + 1, 1, xr1 + xj * cos(fir1) - yj * sin(fir1));
			newX.set(2 * j + 2, 1, yr1 + yj * cos(fir1) + xj * sin(fir1));
			index_of_beacon[num_beacons] = glb_map.X.get_rows() + 1 + 2 * j;
			if(true_index.get_rows() > 0){
				true_index_of_beacon[num_beacons] = (int)true_index.get(2*i + 4, 1);
			}
			assosiations[i] = num_beacons;
			
			glb_map.order[glb_map.I_small.get_rows() + j] = num_beacons;
			++num_matches;
			++num_beacons;
			++num_beacons_in_submap[num_submaps];
			++j;
		}
	}
	glb_map.order[glb_map.I_small.get_rows() + j] = -(num_submaps + 1);
	double xr2 = obsX.get(1	, 1);
	double yr2 = obsX.get(2	, 1);
	double fir2 = obsX.get(3, 1);
	newX.set(2 * num_new + 1, 1, xr1 + xr2 * cos(fir1) - yr2 * sin(fir1));
	newX.set(2 * num_new + 2, 1, yr1 + yr2 * cos(fir1) + xr2 * sin(fir1));
	newX.set(2 * num_new + 3, 1, wrap(fir1 + fir2));

	glb_map.X = vertcat(glb_map.X, newX);
	glb_map.i = vertcat(glb_map.i, zeros(2 * num_new + 3, 1));
	
	SparseMatrix temp = vertcat(to_sparse_matrix_fast(glb_map.I), zeros(2 * num_new + 3, glb_map.I.get_cols()));
	glb_map.I = to_sparse_symm_matrix(horzcat(temp, zeros(temp.get_rows(), 2 * num_new + 3)));
	
	//if(AMD)
	SparseMatrix temp_small = vertcat(to_sparse_matrix_fast(glb_map.I_small), zeros(num_new + 1, glb_map.I_small.get_cols()));
	glb_map.I_small = to_sparse_symm_matrix(horzcat(temp_small, zeros(temp_small.get_rows(), num_new + 1)));
	reallocate(glb_map.I_small, glb_map.I_small.num_nonzero() + (num_obs + 1) * (num_obs + 1));
	
	index_of_robot[num_submaps] = glb_map.X.get_rows() - 2;
	++num_submaps;
}

/*
The update step in map joining

Input: The observed state and covariance matrix. The robot location should be first.
*/
void MapFuser::update_map(SparseMatrix& obsX, SparseMatrix& obsP){
	int num_obs = (obsX.get_rows() - 3)/2;
	int index_robot1 = index_of_robot[num_submaps - 2];
	int index_robot2 = index_of_robot[num_submaps - 1];
	double xi, yi;
	double xr1 = glb_map.X.get(index_robot1		, 1);
	double yr1 = glb_map.X.get(index_robot1 + 1	, 1);
	double fir1 = glb_map.X.get(index_robot1 + 2, 1);
	double xr2 = glb_map.X.get(index_robot2		, 1);
	double yr2 = glb_map.X.get(index_robot2 + 1	, 1);
	double fir2 = glb_map.X.get(index_robot2 + 2, 1);
	
	int ind = 0;
	if(num_obs != num_matches){
		Permutation p(2 * num_matches + 3);
		for(int i = 0; i < 3; ++i){
			p.p[ind] = i;
			++ind;
		}
		for(int i = 0; i < num_obs; ++i){
			if(assosiations[i] >= 0){
				p.p[ind] = 3 + 2 * i;
				++ind;
				p.p[ind] = 4 + 2 * i;
				++ind;
			}
		}
		obsX = obsX.get_submatrix(p, no_reorder(1));
		obsP = obsP.get_submatrix(p, p);
	}

	SparseMatrix jh(2 * num_matches + 3, glb_map.I.get_rows(), (2 + 3) * (2 * num_matches + 3) + 9);
	SparseMatrix H(2 * num_matches + 3, 1, 2 * num_matches + 3);
	
	//the first three rows (robot) of jacobian
	jh.set(1, index_robot1		, -cos(fir1));
	jh.set(1, index_robot1 + 1	, -sin(fir1));
	jh.set(1, index_robot1 + 2	, -(xr2 - xr1) * sin(fir1) + (yr2 - yr1) * cos(fir1));
	jh.set(2, index_robot1		, sin(fir1));
	jh.set(2, index_robot1 + 1	, -cos(fir1));
	jh.set(2, index_robot1 + 2	, -(xr2 - xr1) * cos(fir1) - (yr2 - yr1) * sin(fir1));
	jh.set(3, index_robot1 + 2	, -1);

	jh.set(1, index_robot2		, cos(fir1));
	jh.set(1, index_robot2 + 1	, sin(fir1));
	jh.set(2, index_robot2		, -sin(fir1));
	jh.set(2, index_robot2 + 1	, cos(fir1));
	jh.set(3, index_robot2 + 2	, 1);
	
	// the first three rows (robot) of predicted measurement
	H.set(1, 1, (xr2 - xr1) * cos(fir1) + (yr2 - yr1) * sin(fir1));
	H.set(2, 1, -(xr2 - xr1) * sin(fir1) + (yr2 - yr1) * cos(fir1));
	H.set(3, 1, fir2 - fir1);
	
	// the other rows (beacons) of jacobian and predicted measurement
	ind = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] >= 0){
			xi = glb_map.X.get(index_of_beacon[assosiations[i]]		, 1);
			yi = glb_map.X.get(index_of_beacon[assosiations[i]] + 1	, 1);
			jh.set(2 * ind + 4, index_of_beacon[assosiations[i]], cos(fir1));
			jh.set(2 * ind + 4, index_of_beacon[assosiations[i]] + 1, sin(fir1));
			jh.set(2 * ind + 5, index_of_beacon[assosiations[i]], -sin(fir1));
			jh.set(2 * ind + 5, index_of_beacon[assosiations[i]] + 1, cos(fir1));
		
			jh.set(2 * ind + 4, index_robot1		, -cos(fir1));
			jh.set(2 * ind + 4, index_robot1 + 1	, -sin(fir1));
			jh.set(2 * ind + 4, index_robot1 + 2	, -(xi - xr1) * sin(fir1) + (yi - yr1) * cos(fir1));
			jh.set(2 * ind + 5, index_robot1		, sin(fir1));
			jh.set(2 * ind + 5, index_robot1 + 1	, -cos(fir1));
			jh.set(2 * ind + 5, index_robot1 + 2	, -(xi - xr1) * cos(fir1) - (yi - yr1) * sin(fir1));
		
			H.set(2 * ind + 4, 1, (xi - xr1) * cos(fir1) + (yi - yr1) * sin(fir1));
			H.set(2 * ind + 5, 1, -(xi - xr1) * sin(fir1) + (yi - yr1) * cos(fir1));
			++ind;
		}
	}
	
	SparseSymmMatrix I_new = to_sparse_symm_matrix(trn(jh) * inv(to_sparse_symm_matrix(obsP)) * jh);

	glb_map.I = glb_map.I + I_new;
	int i = 0;
	while(((int*)I_new.A->p)[i] == 0){
		++i;
	}
	num_rows_updated_in_I = glb_map.I.get_rows() - i + 1;
	SparseMatrix z = obsX - H;
	z.set(3, 1, wrap(z.get(3,1)));
	glb_map.i = glb_map.i + trn(jh) * inv(to_sparse_symm_matrix(obsP)) * (z + jh * glb_map.X);
	

	int index_local[50];
	int index, index_loc;
	index_loc = 0;

	Permutation p_order = sorting_permutation(glb_map.order, -num_submaps, num_beacons - 1);
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] != -1){
			index_local[index_loc] = p_order.p[ num_submaps + assosiations[i]] + 1;
			++index_loc;
		}
	}
	index_local[index_loc] = glb_map.I_small.get_rows();
	++index_loc;
	index = 1;
	index_local[index_loc] = p_order.p[ 1] + 1;
	++index_loc;

	for(int i = 0; i < index_loc; ++i){
		for(int j = i; j < index_loc; ++j){
			if(j != i){
				glb_map.I_small.set(index_local[i], index_local[j], 1);
			}
			else{
				glb_map.I_small.set(index_local[i], index_local[j], 100);
			}
		}
	}

}

/*
Input: a local map

Output: The maximum distance from the robot position to one of the beacons.
*/

double MapFuser::submap_radius(const LocalMap& map){
	int num_beac = (map.X.get_rows() - 3)/2;

	double radius = 0;
	double distance, uncertainty;
	SparseSymmMatrix cov;

	for(int i = 1; i <= num_beac; ++i){
	    distance = sqrt((map.X.get(2 * i + 2, 1)- map.X.get(1,1)) * (map.X.get(2 * i + 2, 1)- map.X.get(1,1)) + (map.X.get(2 * i + 3, 1)- map.X.get(2,1)) * (map.X.get(2 * i + 3, 1)- map.X.get(2,1)) );
	    // uncertainty of the beacon
	    cov = to_sparse_symm_matrix(map.P.get_submatrix(2 * i + 2, 2 * i + 2, 2 * i + 3, 2 * i + 3));
	    // times the uncertainty with multiplier (e.g. 3 sigma)
	    uncertainty = SIGMA_MULTIPLIER * max_eig(sqrt(cov));

	    if(uncertainty>1){ // check whether the uncertainty is too large
	        cout << "uncertainty to large " << uncertainty << endl;
	    }
	    distance = distance + uncertainty;
	    if(distance > radius){ // find the largest distance
	        radius = distance;
	    }
	}
	return radius;
}

/*
Loads the paramaeters from the file params. This function 
is called when a MapFuser object is created. 
*/
void MapFuser::load_params(){
	string temp;
	ifstream file("params");

	while(!file.eof()){
		file >> temp;
		if(temp == "MaxL22dimension"){
			file >> MAX_L22_DIMENSION;
		}
		else if(temp == "SigmaMultiplier"){
			file >> SIGMA_MULTIPLIER;
		}
		else if(temp == "EstBias"){
			file >> EST_BIAS;
		}
		else if(temp == "Chi2ConfidenceNN"){
			file >> CHI2_CONFIDENCE_NN;
		}
		else if(temp == "ReorderAMD"){
			file >> REORDER_AMD;
		}
		else if(temp == "ReorderAMDFrequency"){
			file >> REORDER_AMD_FREQUENCY;
		}
	}
}
