#include "MapFuser.h"

MapFuser::MapFuser(){
	num_elements_updated_in_I = 0;
	load_params();
}

void MapFuser::fuse_map(LocalMap m){
	if(m.P.get(1,1) < 1e-8 || m.P.get(2,2) < 1e-8 || m.P.get(3,3) < 1e-8){
		m.P.set(1,1, m.P.get(1,1) + 1e-8);
		m.P.set(2,2, m.P.get(2,2) + 1e-8);
		m.P.set(3,3, m.P.get(3,3) + 1e-8);
	}
	

	radius_of_submap[num_submaps] = submap_radius(m);

	SparseMatrix obsP = m.P.get_submatrix(4,4, m.P.get_rows(), m.P.get_cols());
	SparseMatrix obsX = m.X.get_submatrix(4, 1, m.X.get_rows(), 1);
	set_potential_assosiations();
	SparseMatrix P = restore_part_of_P_for_assositation2();
	global_robot_uncertainty[num_submaps] = SIGMA_MULTIPLIER * max_eig(sqrt(to_sparse_symm_matrix(P.get_submatrix(1, 1, 2, 2))));
	local_robot_uncertainty[num_submaps] = SIGMA_MULTIPLIER * max_eig(sqrt(to_sparse_symm_matrix(m.P.get_submatrix(1, 1, 2, 2))));

	SparseMatrix X = get_part_of_X_for_assositation();
	SparseMatrix beacP = trans_cov_matrix_to_local_cordinate_system(P,X);
	SparseMatrix beacX = trans_state_matrix_to_local_cordinate_system(X);
	
	assosiate_beacons(beacX, beacP, obsX, obsP);

	add_new_beacons_and_robot_location_to_state(m.X);

	update_map(m.X, m.P);

	
	if(REORDER_AMD == 0){
		if(num_elements_updated_in_I > MAX_L22_DIMENSION){
			cout << "Reorder distance " << num_elements_updated_in_I << endl;
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
		if(num_elements_updated_in_I > MAX_L22_DIMENSION){
			cout << "Reorder AMD distance" << endl;
			timer.start(1);
			reorder_submaps_AMD_distance();
			timer.stop(1);
			glb_map.L = cholesky(glb_map.I);
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
		else{
			compute_cholesky_factorization();
			glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
		}
	}
}

SparseMatrix MapFuser::order_for_comparision(){
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
	reorder_submaps(order, index);
	glb_map.L = cholesky(glb_map.I);
	glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
	return solve_cholesky(glb_map.L, eye(glb_map.I.get_rows()));
}

void MapFuser::compute_cholesky_factorization(){
	if(glb_map.I.get_rows() != num_elements_updated_in_I){
		SparseMatrix spaL = to_sparse_matrix(glb_map.L);
		SparseMatrix spaL21 = spaL.get_submatrix(glb_map.I.get_rows() - num_elements_updated_in_I + 1, 1, glb_map.L.get_rows(), glb_map.I.get_cols() - num_elements_updated_in_I);
		spaL21 = vertcat(spaL21, zeros(num_elements_updated_in_I - spaL21.get_rows(), spaL21.get_cols()));
		SparseMatrix spaL11 = spaL.get_submatrix(1, 1, glb_map.I.get_rows() - num_elements_updated_in_I, glb_map.I.get_rows() - num_elements_updated_in_I);
		SparseMatrix I22 = to_sparse_matrix_fast(glb_map.I).get_submatrix(glb_map.I.get_rows() -  num_elements_updated_in_I + 1, glb_map.I.get_cols() -  num_elements_updated_in_I + 1, glb_map.I.get_rows(), glb_map.I.get_cols());
		SparseMatrix spaL22 = to_sparse_matrix(cholesky(to_sparse_symm_matrix(I22 - spaL21 * trn(spaL21))));
		glb_map.L = to_factor(vertcat(horzcat(spaL11, zeros(spaL11.get_rows(), glb_map.I.get_cols() - spaL11.get_cols())), horzcat(spaL21, spaL22)));
	}
	else{
		glb_map.L = cholesky(glb_map.I);
	}
}

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

	timer.start(4);
	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order_submaps[i] = i;
	}
	inc_quicksort_dd1_ci1(dist, order_submaps, 0, num_submaps - 1);
	
	/*for(int i = 0; i < num_submaps; ++i){
		temp_dist = dist[i];
		temp_index = i;
		for(int j = i + 1; j < num_submaps; ++j){
			if(dist[j] > temp_dist){
				temp_dist = dist[j];
				temp_index = j;
			}
		}
		temp_order = order_submaps[temp_index];
		temp_dist = dist[temp_index];
		for(int j = temp_index; j > i; --j){
			dist[j] = dist[j - 1];
			order_submaps[j] = order_submaps[j - 1];
		}
		order_submaps[i] = temp_order;
		dist[i] = temp_dist;
	}*/

	index_detail = 0;
	for(int i = 0; i < num_submaps; ++i){
		for(int j = 0; j < num_beacons_in_submap[order_submaps[i]]; ++j){
			order_detail[index_detail] = submaps_first_beacon[order_submaps[i]] + j;
			++index_detail;
		}
		order_detail[index_detail] = -order_submaps[i] - 1;
		++index_detail;
	}
	timer.stop(4);

	timer.start(3);
	perm = sorting_permutation((double*)glb_map.order.A->x, num_submaps + num_beacons) + inv(sorting_permutation(order_detail, num_submaps + num_beacons));
	Permutation p2 = no_reorder(1); 
	timer.stop(3);

	timer.start(2);
	glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(perm, perm));
	timer.stop(2);
	glb_map.order = glb_map.order.get_submatrix(perm, p2);

	if(glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2 > 1){
		timer.start(5);
		SparseSymmMatrix part_I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(1,1, glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2, glb_map.I_small.get_rows() - MAX_L22_DIMENSION/2));
		Permutation p = reorder_AMD(part_I_small);
		for(int i = 0; i < part_I_small.get_rows(); ++i){
			perm.p[i] = p.p[i];
		}
		for(int i = part_I_small.get_rows(); i < glb_map.I_small.get_rows(); ++i){
			perm.p[i] = i;
		}
		timer.stop(5);
		timer.start(2);
		glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(perm, perm));
		timer.stop(2);
		glb_map.order = glb_map.order.get_submatrix(perm, p2);
	}
	timer.start(6);
	reorder_submaps((double*)glb_map.order.A->x, glb_map.I_small.get_rows());
	timer.stop(6);
}

void MapFuser::reorder_submaps_AMD(){
	Permutation p = reorder_AMD(glb_map.I_small);
	Permutation p2 = no_reorder(1);
	glb_map.I_small = to_sparse_symm_matrix(to_sparse_matrix(glb_map.I_small).get_submatrix(p, p));
	glb_map.order = glb_map.order.get_submatrix(p, p2);
	reorder_submaps((double*)glb_map.order.A->x, glb_map.I_small.get_rows());
}


void MapFuser::reorder_submaps_distance(){
	double dist[10000];
	int order_submaps[10000];
	int order_detail[10000];
	int index_detail;
	int set[10000];
	int temp_index;
	double temp_dist;
	int temp_order;


	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order_submaps[i] = i;
	}
	
	for(int i = 0; i < num_submaps; ++i){
		temp_dist = dist[i];
		temp_index = i;
		for(int j = i + 1; j < num_submaps; ++j){
			if(dist[j] > temp_dist){
				temp_dist = dist[j];
				temp_index = j;
			}
		}
		temp_order = order_submaps[temp_index];
		temp_dist = dist[temp_index];
		for(int j = temp_index; j > i; --j){
			dist[j] = dist[j - 1];
			order_submaps[j] = order_submaps[j - 1];
		}
		order_submaps[i] = temp_order;
		dist[i] = temp_dist;
	}
	//cout << "Submaps order" << endl;
	//for(int i = 0; i < num_submaps; ++i){
		//cout << dist[i] << " " ;
	//}
	//cout <<  endl;

	index_detail = 0;
	//cout << "Num beac: " << num_beacons_in_submap[order_submaps[1]] << " " << order_submaps[1] << endl;
	for(int i = 0; i < num_submaps; ++i){
		for(int j = 0; j < num_beacons_in_submap[order_submaps[i]]; ++j){
			order_detail[index_detail] = submaps_first_beacon[order_submaps[i]] + j;
			++index_detail;
		}
		order_detail[index_detail] = -order_submaps[i] - 1;
		++index_detail;
	}
	reorder_submaps(order_detail, num_beacons + num_submaps);

}

void MapFuser::reorder_submaps(double* order, int size){
	int set[10000];
	int index = 0;
	int set2[1] = {0};
	//cout << index_of_beacon[order[1]] << " " << order[1] << endl;
	for(int i = 0; i < size; ++i){
		if(order[i] >= 0){
			set[index] = index_of_beacon[(int)order[i]] - 1;
			++index;
			set[index] = index_of_beacon[(int)order[i]];
			++index;
			index_of_beacon[(int)order[i]] = index - 1;
		}
		else{
			set[index] = index_of_robot[-(int)order[i] - 1] - 1;
			++index;
			set[index] = index_of_robot[-(int)order[i] - 1];
			++index;
			set[index] = index_of_robot[-(int)order[i] - 1] + 1;
			++index;
			index_of_robot[-(int)order[i] - 1] = index - 2;
		}
	}

	glb_map.i = glb_map.i.get_submatrix(set, index, set2, 1);
	SparseMatrix temp_I = to_sparse_matrix(glb_map.I);
	glb_map.I = to_sparse_symm_matrix( temp_I.get_submatrix(set, index, set, index));
}

void MapFuser::reorder_submaps(int* order, int size){
	int set[10000];
	int index = 0;
	int set2[1] = {0};
	//cout << index_of_beacon[order[1]] << " " << order[1] << endl;
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

	glb_map.i = glb_map.i.get_submatrix(set, index, set2, 1);
	SparseMatrix temp_I = to_sparse_matrix(glb_map.I);
	glb_map.I = to_sparse_symm_matrix( temp_I.get_submatrix(set, index, set, index));
}


double MapFuser::distance_to_submap(int map){
	if(map == num_submaps - 1){
		return 0;
	}
	//double rx = glb_map.X.get( glb_map.X.get_rows() - 2, 1);
	//double ry = glb_map.X.get( glb_map.X.get_rows() - 1, 1);
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

void MapFuser::set_potential_assosiations(){ //const Matrix& P_glb_robot, const Matrix& P_loc_robot){
	num_potential_assosiations = 0;
	double x_beac, y_beac, dist_beac;
	double rx = glb_map.X.get( index_of_robot[num_submaps - 1], 1);
	double ry = glb_map.X.get( index_of_robot[num_submaps - 1] + 1, 1);
	//cout << "rx: " << rx << " ry: " << ry << " " << radius_of_submap[num_submaps] << endl;
	double glb_rob_uncertanty = 0;//max_eig(sqrt(P_glb_robot));
	double loc_rob_uncertanty = 0;//max_eig(sqrt(P_loc_robot));
	/*for(int i = 0; i < num_submaps; ++i){
		//cout << "here pot1" << endl;
		//cout << "i: " << i << " " << " start beac: " << submaps_first_beacon[i] << " num beac: " << num_beacons_in_submap[i] << " " << distance_to_submap(i)<< "  "<< radius_of_submap[i] << " " <<  radius_of_submap[num_submaps] <<   global_robot_uncertainty[num_submaps - 1] << " " <<  global_robot_uncertainty[i]<< " " << local_robot_uncertainty[num_submaps - 1] << endl;
		if(distance_to_submap(i) < radius_of_submap[i] + radius_of_submap[num_submaps] + global_robot_uncertainty[num_submaps - 1]+ global_robot_uncertainty[i] + local_robot_uncertainty[num_submaps - 1] + EST_BIAS){
			//cout << "i: " << i << " index: " << index_of_submap[i] << " " << index_of_submap[i] + 2 * num_beacons_in_submap[i] - 1 << endl;
			//cout << "Submap " << i << " is in " << num_beacons_in_submap[i] << endl;
			for(int j = 0; j < num_beacons_in_submap[i]; ++j){
		        x_beac= glb_map.X.get(index_of_beacon[ submaps_first_beacon[i] + j], 1);
		        y_beac= glb_map.X.get(index_of_beacon[ submaps_first_beacon[i] + j] + 1, 1);     
		        dist_beac = sqrt((x_beac - rx) * (x_beac - rx) + (y_beac - ry) * (y_beac - ry));
		        if(dist_beac < radius_of_submap[num_submaps] + EST_BIAS){
					potential_assosiation_beacons[num_potential_assosiations] = submaps_first_beacon[i] + j;
					++num_potential_assosiations;
		        }
				//cout << "here pot2" << endl;
				//cout << "setting potential: " << submaps_first_beacon[i] + j << endl;

			}
		}
	}*/

				for(int j = 0; j < num_beacons; ++j){
			        x_beac= glb_map.X.get(index_of_beacon[j], 1);
			        y_beac= glb_map.X.get(index_of_beacon[j] + 1, 1);  
			        //cout << x_beac << " " << y_beac << endl;
			        dist_beac = sqrt((x_beac - rx) * (x_beac - rx) + (y_beac - ry) * (y_beac - ry));
			        if(dist_beac < radius_of_submap[num_submaps] + EST_BIAS){
						potential_assosiation_beacons[num_potential_assosiations] = j;
						++num_potential_assosiations;
			        }
					//cout << "here pot2" << endl;
					//cout << "setting potential: " << submaps_first_beacon[i] + j << endl;

				}

	//cout << "potentiella" << endl;
	//for(int i = 0; i < num_potential_assosiations; ++i){
		//cout << potential_assosiation_beacons[i] << endl;
	//}

}

SparseMatrix MapFuser::get_part_of_X_for_assositation(){
	int size = num_potential_assosiations;
	SparseMatrix result(2*size + 3, 1, 2*size + 3);
	//result.set(1, 1, glb_map.X.get(glb_map.X.get_rows() - 2	, 1));
	//result.set(2, 1, glb_map.X.get(glb_map.X.get_rows() - 1	, 1));
	//result.set(3, 1, glb_map.X.get(glb_map.X.get_rows()		, 1));
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

SparseMatrix MapFuser::restore_part_of_P_for_assositation(){
	int size = num_potential_assosiations;
	SparseMatrix result(2*size + 3, 2*size + 3, (2*size + 3)*(2*size + 3));
	SparseMatrix rhs(glb_map.I.get_rows(), 1, glb_map.I.get_rows());
	SparseMatrix x;
	//solve_cholesky(glb_map.L, eye(glb_map.I.get_rows())).print();
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			rhs.clear();
			rhs.set(index_of_beacon[potential_assosiation_beacons[i]] + k, 1, 1);

			x = solve_cholesky(glb_map.L, rhs);
			for(int j = 0; j < 3; ++j){
				result.set(j + 1, 2 * i + k + 4, x.get(rhs.get_rows() - 2 + j, 1));
			}
			for(int j = 0; j < size; ++j){
				for(int m = 0; m < 2; ++m){
					//result.values[2 * j + m + 3][2 * i + k + 3] = x.first_in_row[index_of_beacon[potential_assosiation_beacons[j]] + m]->value;
					result.set(2 * j + m + 4,2 * i + k + 4, x.get(index_of_beacon[potential_assosiation_beacons[j]] + m, 1));
				}
			}	
		}
	}
	for(int i = 0; i < 3; ++i){
		rhs.clear();
		rhs.set(rhs.get_rows() - 2 + i, 1, 1);
		x = solve_cholesky(glb_map.L, rhs);
		for(int j = 0; j < 3; ++j){
			//result.values[j][i] = x.first_in_row[rhs.rows - 2 + j]->value;
			result.set(j + 1, i + 1, x.get(rhs.get_rows() - 2 + j, 1));
		}
		for(int j = 0; j < size; ++j){
			for(int m = 0; m < 2; ++m){
				//result.values[2 * j + m + 3][i] = x.first_in_row[index_of_beacon[potential_assosiation_beacons[j]] + m]->value;
				result.set(2 * j + m + 4, i + 1, x.get(index_of_beacon[potential_assosiation_beacons[j]] + m, 1));
			}
		}	
	}

	return result;
}

SparseMatrix MapFuser::restore_part_of_P_for_assositation2(){

	int size = num_potential_assosiations;
	SparseMatrix result;
	SparseMatrix rhs(glb_map.I.get_rows(), 2 * num_potential_assosiations + 3, 2 * num_potential_assosiations + 3);
	SparseMatrix x;
	int rset[2 * num_potential_assosiations + 3];
	int cset[2 * num_potential_assosiations + 3];
	int index = 0;
	//solve_cholesky(glb_map.L, eye(glb_map.I.get_rows())).print();
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

	//cout << "rset: ";
	//for(int i = 0; i < index; ++i){
	//	cout << rset[i] << " ";
	//}
	//cout << endl;

	x = solve_cholesky(glb_map.L, rhs);

	result = x.get_submatrix(rset, index, cset, index);

	return result;
}



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
    glb_map.order = SparseMatrix((glb_map.I.get_rows() - 1)/2, 1, (glb_map.I.get_rows() - 1)/2);
    for(int i = 1; i <= (glb_map.I.get_rows() - 1)/2; ++i){
    	glb_map.I_small.set(i, i, 100);
    }
    for(int i = 1; i <= (glb_map.I.get_rows() - 1)/2; ++i){
    	glb_map.order.set(i, 1, i - 1);
    }
    glb_map.order.set(glb_map.order.get_rows(), 1, -1);
    
    
    num_submaps = 1;
    num_beacons = (m.P.get_rows() - 3)/2;
    num_beacons_in_submap[0] = num_beacons;
    for(int i = 0; i <= num_beacons; ++i){
    	index_of_beacon[i] = 2 * i + 1;
    }
    index_of_robot[0] = m.P.get_rows() - 2;
    submaps_first_beacon[0] = 0;
    global_robot_uncertainty[0] = 0;
}

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
	//beacX.write_to_file("SavedMatrices/beacXAMD1");
	//beacP.write_to_file("SavedMatrices/beacPAMD1");
	int num_obs = obsP.get_rows()/2;
	int num_beac = beacP.get_rows()/2;
	double r_obs, b_obs, r_beac, b_beac;
	SparseSymmMatrix cov_obs, cov_beac, totalcov;
	SparseMatrix innov(2,1, 2);
	double mahadist, mahadist_new;
	num_matches = 0;
	//cout << "num_obs: " << num_obs << endl;
	for(int j = 1; j <= num_obs; ++j){
		mahadist = 1e8;

	    r_obs = obsX.get(j*2-1, 1);
	    b_obs = obsX.get(j*2, 1);
	    if(j == 1){
	    	//cout << "obs pos: " << r_obs << " " << b_obs << endl;
	    }
		if(j == 10){
			//cout << "pos obs: " << r_obs << " " << b_obs << endl;
		}
	    cov_obs = to_sparse_symm_matrix(obsP.get_submatrix(j*2-1,j*2-1,j*2,j*2));  // covariance matrix of the j-th obs
	    for(int i = 1; i <= num_beac; ++i){
	        r_beac = beacX.get(i*2-1, 1);
	        b_beac = beacX.get(i*2, 1);
	        if(j == 1){
	        	//cout << "beac pos: " << r_beac << " " << b_beac << endl;
	        }
	        if(j == 10){
	        	//cout << "pos beac: " << r_beac << " " << b_beac << endl;
	        }
	        cov_beac = to_sparse_symm_matrix(beacP.get_submatrix(i*2-1,i*2-1,i*2,i*2));  // covariance matrix of the i-th beac
	        // compute Mahalanobis distance from j-th obs to i-th beac
	        innov.set(1,1, r_beac-r_obs);
	        innov.set(2,1, b_beac-b_obs);
	        totalcov = cov_beac + cov_obs;

	        mahadist_new  = (trn(innov)*inv(totalcov)*innov).get(1,1);
	        //cout << "j: " << j << " " << mahadist_new << endl;
        	//if(j == 10){
        		//cout << "Maha dist: " << mahadist_new << endl;
        	//}
	       // if(j == 1){
	        	//cout << "dist: " << mahadist_new << endl;
	        	//cout << "cov_beac" << endl;
	        	//cov_beac.print();
	        	//cout << "cov_obs" << endl;
	        	//cov_obs.print();
	        	//totalcov.print();
	        //}
	        if(mahadist_new < mahadist){
	        	//cout << "obs: " << j << " old: " << i  << " new dist " << mahadist_new << " old dist: " << mahadist << endl;
	        	mahadist = mahadist_new;
	        	//9.2103 is the chi-square invers for 99% with 2 degrees of freedom

	        	if(mahadist < CHI2_CONFIDENCE_NN){
	        		assosiations[j - 1] = i - 1;
	        	}
	        	else{
	        		assosiations[j - 1] = -100;
	        	}
	        }
	    }

	}
   // for(int j = 0; j < num_obs; ++j){
    	//cout << "ass: " << assosiations[j] << endl;
    //}
	//if(num_submaps == 42){
		//assosiations[9] = -100;
	//}
	//if(num_submaps  +  1 == 211){
		//assosiations[0] = 5;
	//cout << "index 25 " << index_of_beacon[25] << endl;
	//}
	//cout << "Matches: "<<endl;
	for(int j = 0; j < num_obs; ++j){
		if(assosiations[j] >= 0)
			assosiations[j] = potential_assosiation_beacons[assosiations[j]];
		//cout << assosiations[j] << " x: " << glb_map.X.get(index_of_beacon[assosiations[j]], 1) << " y: " << glb_map.X.get(index_of_beacon[assosiations[j]] + 1, 1) << endl;
	}
}

void MapFuser::add_new_beacons_and_robot_location_to_state(const SparseMatrix& obsX){
	int num_obs = (obsX.get_rows() - 3)/2;
	//double xr1 = glb_map.X.get(glb_map.X.get_rows() - 2	, 1);
	//double yr1 = glb_map.X.get(glb_map.X.get_rows() - 1	, 1);
	//double fir1 = glb_map.X.get(glb_map.X.get_rows()		, 1);
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
	//if(AMD)
	SparseMatrix new_order(num_new + 1, 1, num_new + 1);
	//SparseMatrix newI(glb_map.I.get_rows() + 2 * num_obs + 3, glb_map.I.get_cols() + 2 * num_obs + 3, glb_map.I.max_num_nonzero());
	//SparseMatrix newi(glb_map.i.get_rows() + 2 * num_obs + 3, 1, glb_map.i.max_num_nonzero());
	submaps_first_beacon[num_submaps] = num_beacons;
	num_beacons_in_submap[num_submaps] = 0;
	int j = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			//cout << "i: " << i << endl;
			xj = obsX.get(2*i + 4, 1);
			yj =  obsX.get(2*i + 5, 1);
			newX.set(2 * j + 1, 1, xr1 + xj * cos(fir1) - yj * sin(fir1));
			newX.set(2 * j + 2, 1, yr1 + yj * cos(fir1) + xj * sin(fir1));
			index_of_beacon[num_beacons] = glb_map.X.get_rows() + 1 + 2 * j;
			assosiations[i] = num_beacons;
			
			//if(AMD)
			new_order.set(j + 1, 1, num_beacons);
			++num_matches;
			++num_beacons;
			++num_beacons_in_submap[num_submaps];
			++j;
		}
	}
	new_order.set(new_order.get_rows(), 1, -(num_submaps + 1));
	double xr2 = obsX.get(1	, 1);
	double yr2 = obsX.get(2	, 1);
	double fir2 = obsX.get(3, 1);
	newX.set(2 * num_new + 1, 1, xr1 + xr2 * cos(fir1) - yr2 * sin(fir1));
	newX.set(2 * num_new + 2, 1, yr1 + yr2 * cos(fir1) + xr2 * sin(fir1));
	newX.set(2 * num_new + 3, 1, wrap(fir1 + fir2));


	glb_map.X = vertcat(glb_map.X, newX);
	glb_map.i = vertcat(glb_map.i, zeros(2 * num_new + 3, 1));
	
	//if(AMD)
	glb_map.order = vertcat(glb_map.order, new_order);
	
	//cout << "Innan: " << glb_map.I_small.max_num_nonzero() << " " << glb_map.I_small.num_nonzero() << endl;
	
	SparseMatrix temp = vertcat(to_sparse_matrix_fast(glb_map.I), zeros(2 * num_new + 3, glb_map.I.get_cols()));
	glb_map.I = to_sparse_symm_matrix(horzcat(temp, zeros(temp.get_rows(), 2 * num_new + 3)));
	SparseMatrix temp_small = vertcat(to_sparse_matrix_fast(glb_map.I_small), zeros(num_new + 1, glb_map.I_small.get_cols()));
	glb_map.I_small = to_sparse_symm_matrix(horzcat(temp_small, zeros(temp_small.get_rows(), num_new + 1)));
	reallocate(glb_map.I_small, glb_map.I_small.num_nonzero() + (num_obs + 1) * (num_obs + 1));
	index_of_robot[num_submaps] = glb_map.X.get_rows() - 2;
	++num_submaps;
	//cout << "Efter: " << glb_map.I_small.max_num_nonzero() << " " << glb_map.I_small.num_nonzero() << endl;
	//cout << "Matches: "<<endl;
	//for(int j = 0; j < num_obs; ++j){
		//cout << assosiations[j] << endl;
	//}
}

void MapFuser::update_map(const SparseMatrix& obsX, const SparseMatrix& obsP){
	int num_obs = (obsX.get_rows() - 3)/2;
	//int index_robot1 = glb_map.X.get_rows() - 2 * num_beacons_in_submap[num_submaps - 1] - 5;
	//int index_robot2 = glb_map.X.get_rows() - 2;
	int index_robot1 = index_of_robot[num_submaps - 2];
	int index_robot2 = index_of_robot[num_submaps - 1];
	double xi, yi;
	double xr1 = glb_map.X.get(index_robot1		, 1);
	double yr1 = glb_map.X.get(index_robot1 + 1	, 1);
	double fir1 = glb_map.X.get(index_robot1 + 2, 1);
	double xr2 = glb_map.X.get(index_robot2		, 1);
	double yr2 = glb_map.X.get(index_robot2 + 1	, 1);
	double fir2 = glb_map.X.get(index_robot2 + 2, 1);
	//cout << xr1 << " "<< yr1 << " "<< fir1 << " "<< xr2 << " "<< yr2 << " "<< fir2 << " " << endl;
	
	
	SparseMatrix jh(2 * num_obs + 3, glb_map.I.get_rows(), (2 + 3) * (2 * num_obs + 3) + 9);
	SparseMatrix H(2 * num_obs + 3, 1, 2 * num_obs + 3);
	
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
	for(int i = 0; i < num_obs; ++i){
		//if(assosiations[i] >= 0){
			xi = glb_map.X.get(index_of_beacon[assosiations[i]]		, 1);
			yi = glb_map.X.get(index_of_beacon[assosiations[i]] + 1	, 1);
			//cout << xi << " " << yi <<  " " << index_of_beacon[assosiations[i]] <<" " << assosiations[i] <<endl;
			jh.set(2 * i + 4, index_of_beacon[assosiations[i]], cos(fir1));
			jh.set(2 * i + 4, index_of_beacon[assosiations[i]] + 1, sin(fir1));
			jh.set(2 * i + 5, index_of_beacon[assosiations[i]], -sin(fir1));
			jh.set(2 * i + 5, index_of_beacon[assosiations[i]] + 1, cos(fir1));
		
			jh.set(2 * i + 4, index_robot1		, -cos(fir1));
			jh.set(2 * i + 4, index_robot1 + 1	, -sin(fir1));
			jh.set(2 * i + 4, index_robot1 + 2	, -(xi - xr1) * sin(fir1) + (yi - yr1) * cos(fir1));
			jh.set(2 * i + 5, index_robot1		, sin(fir1));
			jh.set(2 * i + 5, index_robot1 + 1	, -cos(fir1));
			jh.set(2 * i + 5, index_robot1 + 2	, -(xi - xr1) * cos(fir1) - (yi - yr1) * sin(fir1));
		
			H.set(2 * i + 4, 1, (xi - xr1) * cos(fir1) + (yi - yr1) * sin(fir1));
			H.set(2 * i + 5, 1, -(xi - xr1) * sin(fir1) + (yi - yr1) * cos(fir1));
		//}
	}


	SparseSymmMatrix I_new = to_sparse_symm_matrix(trn(jh) * inv(to_sparse_symm_matrix(obsP)) * jh);

	glb_map.I = glb_map.I + I_new;
	int i = 0;
	while(((int*)I_new.A->p)[i] == 0){
		++i;
	}
	num_elements_updated_in_I = glb_map.I.get_rows() - i + 1;

	SparseMatrix z = obsX - H;
	z.set(3, 1, wrap(z.get(3,1)));
	glb_map.i = glb_map.i + trn(jh) * inv(to_sparse_symm_matrix(obsP)) * (z + jh * glb_map.X);
	
	int index_local[50];
	int index, index_loc;
	index_loc = 0;
	for(int i = 0; i < num_obs; ++i){
		index = 1;
		while(assosiations[i] != glb_map.order.get(index, 1)){
			++index;
		}
		index_local[index_loc] = index;
		++index_loc;
	}
	index_local[index_loc] = glb_map.I_small.get_rows();
	++index_loc;
	index = 1;
	while(-num_submaps + 1 != glb_map.order.get(index, 1)){
		++index;
	}
	index_local[index_loc] = index;
	++index_loc;
	
	//cout << "Index: ";
	//for(int i = 0; i < index_loc; ++i){
		//cout << index_local[i] << " ";
	//}
	//cout << endl;
	for(int i = 0; i < index_loc; ++i){
		for(int j = i; j < index_loc; ++j){
			//cout << "Loop: " << glb_map.I_small.max_num_nonzero() << " " << glb_map.I_small.num_nonzero() << endl;
			if(j != i){
				glb_map.I_small.set(index_local[i], index_local[j], 1);
			}
			else{
				glb_map.I_small.set(index_local[i], index_local[j], 100);
			}
		}
	}
}

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
	    //sqrt(cov).print();
	    //cout << endl;
	    //cout << uncertainty << endl;
	    if(uncertainty>1){ // check whether the uncertainty is too large
	        cout << "uncertainty to large " << uncertainty << endl;
	    }
	    distance = distance + uncertainty;
	    if(distance > radius){ // find the largest distance
	        radius = distance;
	    }
	}
	//cout << radius << endl;
	//cout << endl;
	return radius;
}

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
