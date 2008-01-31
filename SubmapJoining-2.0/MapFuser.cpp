#include "MapFuser.h"

MapFuser::MapFuser(){
	num_elements_updated_in_I = 0;
}

void MapFuser::fuse_map(LocalMap m){
	if(m.P.get(1,1) < 1e-8 || m.P.get(2,2) < 1e-8 || m.P.get(3,3) < 1e-8){
		m.P.set(1,1, m.P.get(1,1) + 1e-8);
		m.P.set(2,2, m.P.get(2,2) + 1e-8);
		m.P.set(3,3, m.P.get(3,3) + 1e-8);
	}
	

	radius_of_submap[num_submaps] = submap_radius(m);
	//cout << "here" << endl;
	m.P.get_submatrix(4,4, m.P.get_rows(), m.P.get_cols());
	//cout << "here" << endl;
	SparseMatrix obsP = m.P.get_submatrix(4,4, m.P.get_rows(), m.P.get_cols());
	SparseMatrix obsX = m.X.get_submatrix(4, 1, m.X.get_rows(), 1);
	set_potential_assosiations();
	SparseMatrix P = restore_part_of_P_for_assositation();
	//cout << "before " << endl;
	global_robot_uncertainty[num_submaps] = 3 * max_eig(sqrt(to_sparse_symm_matrix(P.get_submatrix(1, 1, 2, 2))));
	//cout << "here" << endl;
	local_robot_uncertainty[num_submaps] = 3 * max_eig(sqrt(to_sparse_symm_matrix(m.P.get_submatrix(1, 1, 2, 2))));
	//cout << "here1.1" << endl;
	SparseMatrix X = get_part_of_X_for_assositation();
	//cout << "here2" << endl;
	SparseMatrix beacP = trans_cov_matrix_to_local_cordinate_system(P,X);
	//cout << "here3" << endl;
	SparseMatrix beacX = trans_state_matrix_to_local_cordinate_system(X);

	beacX.write_to_file("SavedMatrices/beacX1");
	beacP.write_to_file("SavedMatrices/beacP1");
	obsX.write_to_file("SavedMatrices/obsX1");
	obsP.write_to_file("SavedMatrices/obsP1");
	assosiate_beacons(beacX, beacP, obsX, obsP);
	//cout << "here4" << endl;
	add_new_beacons_and_robot_location_to_state(m.X);
	
	update_map(m.X, m.P);
	cout << "here5" << endl;
	glb_map.L = cholesky(glb_map.I);
	cout << "here6" << endl;
	glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
	//cout << "here7" << endl;
	/*if(num_elements_updated_in_I > 250){
		glb_map.L = cholesky(glb_map.I, timer);
		glb_map.X = solve_cholesky2(glb_map.L, glb_map.i, timer);
		reorder_submaps();
		glb_map.L = cholesky(glb_map.I, timer);
		
	}
	else{
		compute_cholesky_factorization();
		glb_map.X = solve_cholesky2(glb_map.L, glb_map.i, timer);
	}*/
}

//void MapFuser::compute_cholesky_factorization(){
	//part_cholesky(glb_map.L, glb_map.I, num_elements_updated_in_I, timer);
	//cout << glb_map.I.rows - num_elements_updated_in_I << endl;
	//SparseMatrix chol_new;
	/*for(int i = 1; i <=num_elements_updated_in_I; ++i){
		new_I.first_in_row[i] = glb_map.I.first_in_row[glb_map.I.rows - num_elements_updated_in_I + i];
	}
	cout << "glb_map" << endl;
	glb_map.I.print();
	cout << "new_I" << endl;
	new_I.print();*/
	/*chol_new = cholesky(glb_map.I, glb_map.I.rows - num_elements_updated_in_I);
	for(int i = glb_map.I.rows - num_elements_updated_in_I; i <=glb_map.I.rows; ++i){
		//memmory leak!!
		glb_map.L.first_in_row[i] = chol_new.first_in_row[i];
		chol_new.first_in_row[i] = 0;
	}
	glb_map.L.rows = glb_map.I.rows;
	glb_map.L.cols = glb_map.I.cols;
	
	cout << "Cholesky" << endl;
	glb_map.L.print();
	cout << "should be" << endl;
	(cholesky(glb_map.I, 1)).print();*/
//

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

void MapFuser::reorder_submaps(){
	double dist[1000];
	int order[1000];
	int set[1000];
	int temp_index;
	double temp_dist;
	int temp_order;

	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order[i] = i;
	}
	for(int i = 0; i < num_submaps; ++i){
		cout << dist[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << order[i] << " ";
	}
	cout << endl;
	
	for(int i = 0; i < num_submaps; ++i){
		temp_dist = dist[i];
		temp_index = i;
		for(int j = i + 1; j < num_submaps; ++j){
			if(dist[j] > temp_dist){
				temp_dist = dist[j];
				temp_index = j;
			}
		}
		temp_order = order[temp_index];
		temp_dist = dist[temp_index];
		for(int j = temp_index; j > i; --j){
			dist[j] = dist[j - 1];
			order[j] = order[j - 1];
		}
		order[i] = temp_order;
		dist[i] = temp_dist;
	}

	for(int i = 0; i < num_submaps; ++i){
		cout << dist[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << order[i] << " ";
	}
	cout << endl;

	
	//make set
	int col_num = 0;
	int submap1_index, from_col, to_col;
	for(int i = 0; i < num_submaps; ++i){
		submap1_index = index_of_submap[order[i]];
		from_col = submap1_index;
		to_col = submap1_index + 2 * num_beacons_in_submap[order[i]] + 2;
		for(int j = from_col; j <= to_col; ++j){
			set[col_num] = j - 1;
			++col_num;
		}
	}
	
	cout << "set" << endl;
	for(int i = 0; i < col_num; ++i){
		cout << set[i] << " ";
	}
	cout << endl;
	int set2[1] = {0};
	glb_map.X = glb_map.X.get_submatrix(set, col_num, set2, 1);
	glb_map.i = glb_map.i.get_submatrix(set, col_num, set2, 1);
	glb_map.I = to_sparse_symm_matrix( to_sparse_matrix(glb_map.I).get_submatrix(set, col_num, set, col_num));
	
	
	cout << "index before" << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << index_of_submap[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_beacons; ++i){
		cout << place_of_beacon[i] << " ";
	}
	cout << endl;
	int index = 1;
	int index_diff;
	//int index_of_submap[1000];
	//int place_of_beacon[10000];
	for(int i = 0; i < num_submaps; ++i){
		index_diff = index_of_submap[order[i]] - index;
		index_of_submap[order[i]] -= index_diff; 
		for(int j = submaps_first_beacon[order[i]]; j < submaps_first_beacon[order[i]] + num_beacons_in_submap[order[i]]; ++j){
			place_of_beacon[j] -= index_diff;
		}
		index += 2 * num_beacons_in_submap[order[i]] + 3;
	}
	
	cout << "index after" << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << index_of_submap[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_beacons; ++i){
		cout << place_of_beacon[i] << " ";
	}
	cout << endl;
}

/*void MapFuser::reorder_submaps(){
	double dist[1000];
	int order[1000];
	int temp_index;
	double temp_dist;
	int temp_order;

	for(int i = 0; i < num_submaps; ++i){
		dist[i] = distance_to_submap(i);
		order[i] = i;
	}
	for(int i = 0; i < num_submaps; ++i){
		cout << dist[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << order[i] << " ";
	}
	cout << endl;
	
	for(int i = 0; i < num_submaps; ++i){
		temp_dist = dist[i];
		temp_index = i;
		for(int j = i + 1; j < num_submaps; ++j){
			if(dist[j] > temp_dist){
				temp_dist = dist[j];
				temp_index = j;
			}
		}
		temp_order = order[temp_index];
		temp_dist = dist[temp_index];
		for(int j = temp_index; j > i; --j){
			dist[j] = dist[j - 1];
			order[j] = order[j - 1];
		}
		order[i] = temp_order;
		dist[i] = temp_dist;
	}

	for(int i = 0; i < num_submaps; ++i){
		cout << dist[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << order[i] << " ";
	}
	cout << endl;
	
	// reorder I
	SparseSymmMatrix temp(glb_map.I.rows, glb_map.I.cols);
	SparseMatrixElement *row_ptr;
	int set_row = 0;
	int to_row, from_row, to_col, from_col;
	int submap1_index, submap2_index;
	int col_num;
	int row_num = 1;
	SparseMatrix temp_sparse;
	for(int i = 0; i < num_submaps; ++i){
		col_num = glb_map.I.cols + 1;
		for(int j = num_submaps - 1; j >= i; --j){
			col_num -= 2 * num_beacons_in_submap[order[j]] + 3;
			submap1_index = index_of_submap[order[i]];
			submap2_index = index_of_submap[order[j]];
			if(submap1_index > submap2_index){
				from_row = submap2_index;
				to_row = submap2_index + 2 * num_beacons_in_submap[order[j]] + 2;
				from_col = submap1_index;
				to_col = submap1_index + 2 * num_beacons_in_submap[order[i]] + 2;
				temp_sparse = extract_sub_matrix(glb_map.I, from_row, from_col, to_row, to_col);
				set(temp, row_num, col_num, trn(temp_sparse));
			}
			else{
				from_row = submap1_index;
				to_row = submap1_index + 2 * num_beacons_in_submap[order[i]] + 2;
				from_col = submap2_index;
				to_col = submap2_index + 2 * num_beacons_in_submap[order[j]] + 2;
				temp_sparse = extract_sub_matrix(glb_map.I, from_row, from_col, to_row, to_col);
				set(temp, row_num, col_num, temp_sparse);
			}
		}
		row_num += 2 * num_beacons_in_submap[order[i]] + 3;
	}
	glb_map.I = temp;
	
	//glb_map.i.write_to_file("SavedMatrices/ibr");

	SparseMatrix temp_i(glb_map.i.rows, 1);
	
	//reorder i
	col_num = 1;
	for(int i = 0; i < num_submaps; ++i){
		submap1_index = index_of_submap[order[i]];
		from_col = submap1_index;
		to_col = submap1_index + 2 * num_beacons_in_submap[order[i]] + 2;
		for(int j = from_col; j <= to_col; ++j){
			cout << "Set: " << col_num << " " <<glb_map.i.get(j, 1) << endl;
			temp_i.set(col_num, 1, glb_map.i.get(j, 1));
			cout << "after set" << endl;
			++col_num;
		}
	}
	glb_map.i = temp_i;
	//glb_map.i.write_to_file("SavedMatrices/iar");
	
	//reorder X
	SparseMatrix temp_X(glb_map.X.rows, 1);

	col_num = 1;
	for(int i = 0; i < num_submaps; ++i){
		submap1_index = index_of_submap[order[i]];
		from_col = submap1_index;
		to_col = submap1_index + 2 * num_beacons_in_submap[order[i]] + 2;
		for(int j = from_col; j <= to_col; ++j){
			temp_X.set(col_num, 1, glb_map.X.get(j, 1));
			++col_num;
		}
	}
	glb_map.X = temp_X;
	//glb_map.X.write_to_file("SavedMatrices/Xar");
	
	cout << "index before" << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << index_of_submap[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_beacons; ++i){
		cout << place_of_beacon[i] << " ";
	}
	cout << endl;
	int index = 1;
	int index_diff;
	//int index_of_submap[1000];
	//int place_of_beacon[10000];
	for(int i = 0; i < num_submaps; ++i){
		index_diff = index_of_submap[order[i]] - index;
		index_of_submap[order[i]] -= index_diff; 
		for(int j = submaps_first_beacon[order[i]]; j < submaps_first_beacon[order[i]] + num_beacons_in_submap[order[i]]; ++j){
			place_of_beacon[j] -= index_diff;
		}
		index += 2 * num_beacons_in_submap[order[i]] + 3;
	}
	
	cout << "index after" << endl;
	for(int i = 0; i < num_submaps; ++i){
		cout << index_of_submap[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < num_beacons; ++i){
		cout << place_of_beacon[i] << " ";
	}
	cout << endl;
}*/

double MapFuser::distance_to_submap(int map){
	//cout << "dist start" << endl;
	if(map == num_submaps - 1){
		return 0;
	}
	double rx = glb_map.X.get( glb_map.X.get_rows() - 2, 1);
	double ry = glb_map.X.get( glb_map.X.get_rows() - 1, 1);
	//cout << "dist1" << endl;
	double mx, my;
	if(map == 0){
		mx = 0;
		my = 0;
	}
	else{
		//cout << "dist2" << endl;
		//cout << "index" <<  index_of_submap[map] << endl;
		mx = glb_map.X.get( index_of_submap[map - 1] + 2 * num_beacons_in_submap[map - 1] , 1);
		my = glb_map.X.get(index_of_submap[map - 1] + 2 * num_beacons_in_submap[map - 1] + 1, 1);
		//cout << "dist3" << endl;
	}
	return sqrt((rx - mx) * (rx - mx) + (ry - my) * (ry - my)); 
}

void MapFuser::set_potential_assosiations(){ //const Matrix& P_glb_robot, const Matrix& P_loc_robot){
	num_potential_assosiations = 0;
	double x_beac, y_beac, dist_beac;
	double rx = glb_map.X.get( glb_map.X.get_rows() - 2, 1);
	double ry = glb_map.X.get( glb_map.X.get_rows() - 1, 1);
	double glb_rob_uncertanty = 0;//max_eig(sqrt(P_glb_robot));
	double loc_rob_uncertanty = 0;//max_eig(sqrt(P_loc_robot));
	for(int i = 0; i < num_submaps; ++i){
		//cout << "here pot1" << endl;
		//cout << "i: " << i << " " << " start beac: " << submaps_first_beacon[i] << " num beac: " << num_beacons_in_submap[i] << " " << distance_to_submap(i)<< "  "<< radius_of_submap[i] << " " <<  radius_of_submap[num_submaps] <<   global_robot_uncertainty[num_submaps - 1] << " " <<  global_robot_uncertainty[i]<< " " << local_robot_uncertainty[num_submaps - 1] << endl;
		if(distance_to_submap(i) < radius_of_submap[i] + radius_of_submap[num_submaps] + global_robot_uncertainty[num_submaps - 1]+ global_robot_uncertainty[i] + local_robot_uncertainty[num_submaps - 1] + 3){
			//cout << "i: " << i << " index: " << index_of_submap[i] << " " << index_of_submap[i] + 2 * num_beacons_in_submap[i] - 1 << endl;
			//cout << "Submap " << i << " is in " << num_beacons_in_submap[i] << endl;
			for(int j = 0; j < num_beacons_in_submap[i]; ++j){
		        x_beac= glb_map.X.get(place_of_beacon[ submaps_first_beacon[i] + j], 1);
		        y_beac= glb_map.X.get(place_of_beacon[ submaps_first_beacon[i] + j] + 1, 1);     
		        dist_beac = sqrt((x_beac - rx) * (x_beac - rx) + (y_beac - ry) * (y_beac - ry));
		        
		        if(dist_beac < radius_of_submap[num_submaps] + 3){
					potential_assosiation_beacons[num_potential_assosiations] = submaps_first_beacon[i] + j;
					++num_potential_assosiations;
		        }
				//cout << "here pot2" << endl;
				//cout << "setting potential: " << submaps_first_beacon[i] + j << endl;

			}
		}
	}
	//cout << "potentiella" << endl;
	for(int i = 0; i < num_potential_assosiations; ++i){
		//cout << potential_assosiation_beacons[i] << endl;
	}

}

SparseMatrix MapFuser::get_part_of_X_for_assositation(){
	int size = num_potential_assosiations;
	SparseMatrix result(2*size + 3, 1, 2*size + 3);
	result.set(1, 1, glb_map.X.get(glb_map.X.get_rows() - 2	, 1));
	result.set(2, 1, glb_map.X.get(glb_map.X.get_rows() - 1	, 1));
	result.set(3, 1, glb_map.X.get(glb_map.X.get_rows()		, 1));
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			result.set(2 * i + k + 4, 1, glb_map.X.get(place_of_beacon[potential_assosiation_beacons[i]] + k, 1));
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
			rhs.set(place_of_beacon[potential_assosiation_beacons[i]] + k, 1, 1);

			x = solve_cholesky(glb_map.L, rhs);
			for(int j = 0; j < 3; ++j){
				result.set(j + 1, 2 * i + k + 4, x.get(rhs.get_rows() - 2 + j, 1));
			}
			for(int j = 0; j < size; ++j){
				for(int m = 0; m < 2; ++m){
					//result.values[2 * j + m + 3][2 * i + k + 3] = x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]->value;
					result.set(2 * j + m + 4,2 * i + k + 4, x.get(place_of_beacon[potential_assosiation_beacons[j]] + m, 1));
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
				//result.values[2 * j + m + 3][i] = x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]->value;
				result.set(2 * j + m + 4, i + 1, x.get(place_of_beacon[potential_assosiation_beacons[j]] + m, 1));
			}
		}	
	}

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
	
    glb_map.I = inv(to_sparse_symm_matrix(m.P));
    glb_map.i = inv(to_sparse_symm_matrix(m.P))* m.X;
    glb_map.L = cholesky(glb_map.I);

    glb_map.X = m.X;
    
    num_submaps = 1;
    num_beacons = (m.P.get_rows() - 3)/2;
    num_beacons_in_submap[0] = num_beacons;
    for(int i = 0; i <= num_beacons; ++i){
    	place_of_beacon[i] = 2 * i + 1;
    }
    index_of_submap[0] = 1;
    submaps_first_beacon[0] = 0;
    global_robot_uncertainty[0] = 0;
}

SparseMatrix MapFuser::trans_cov_matrix_to_local_cordinate_system(const SparseMatrix& P, const SparseMatrix& X){
	//robot pos is first in X and will be (0,0,0) in the new system 
	double xr = X.get(1, 1);
	double yr = X.get(2, 1);
	double fir = X.get(3, 1);
	SparseMatrix jh(X.get_rows() - 3, X.get_rows(), (X.get_rows() - 3) * X.get_rows());
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

	int num_obs = obsP.get_rows()/2;
	int num_beac = beacP.get_rows()/2;
	double r_obs, b_obs, r_beac, b_beac;
	SparseSymmMatrix cov_obs, cov_beac, totalcov;
	SparseMatrix innov(2,1, 2);
	double mahadist, mahadist_new;
	num_matches = 0;
	for(int j = 1; j <= num_obs; ++j){
		mahadist = 1e8;

	    r_obs = obsX.get(j*2-1, 1);
	    b_obs = obsX.get(j*2, 1);
	    //cout << "obs pos: " << r_obs << " " << b_obs << endl;
		if(j == 10){
			//cout << "pos obs: " << r_obs << " " << b_obs << endl;
		}
	    cov_obs = to_sparse_symm_matrix(obsP.get_submatrix(j*2-1,j*2-1,j*2,j*2));  // covariance matrix of the j-th obs
	    for(int i = 1; i <= num_beac; ++i){
	        r_beac = beacX.get(i*2-1, 1);
	        b_beac = beacX.get(i*2, 1);
	        //cout << "beac pos: " << r_beac << " " << b_beac << endl;
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
	        if(mahadist_new < mahadist){
	        	//cout << "obs: " << j << " old: " << i  << " new dist " << mahadist_new << " old dist: " << mahadist << endl;
	        	mahadist = mahadist_new;
	        	//9.2103 is the chi-square invers for 99% with 2 degrees of freedom

	        	if(mahadist < 9.21034037197618){
	        		assosiations[j - 1] = i - 1;
	        	}
	        	else{
	        		assosiations[j - 1] = -100;
	        	}
	        }
	    }
	}
	//if(num_submaps == 42){
		//assosiations[9] = -100;
	//}
	if(num_submaps == 42){
	//cout << "index 25 " << place_of_beacon[25] << endl;
	}
	//cout << "Matches: "<<endl;
	for(int j = 0; j < num_obs; ++j){
		if(assosiations[j] >= 0)
			assosiations[j] = potential_assosiation_beacons[assosiations[j]];
		//cout << assosiations[j] << " x: " << glb_map.X.get(place_of_beacon[assosiations[j]], 1) << " y: " << glb_map.X.get(place_of_beacon[assosiations[j]] + 1, 1) << endl;
	}
}

void MapFuser::add_new_beacons_and_robot_location_to_state(const SparseMatrix& obsX){
	int num_obs = (obsX.get_rows() - 3)/2;
	double xr1 = glb_map.X.get(glb_map.X.get_rows() - 2	, 1);
	double yr1 = glb_map.X.get(glb_map.X.get_rows() - 1	, 1);
	double fir1 = glb_map.X.get(glb_map.X.get_rows()		, 1);
	double xj;
	double yj;
	int num_new = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			++num_new;
		}
	}
	SparseMatrix newX(2 * num_new + 3, 1, 2 * num_new + 3);
	//SparseMatrix newI(glb_map.I.get_rows() + 2 * num_obs + 3, glb_map.I.get_cols() + 2 * num_obs + 3, glb_map.I.max_num_nonzero());
	//SparseMatrix newi(glb_map.i.get_rows() + 2 * num_obs + 3, 1, glb_map.i.max_num_nonzero());
	submaps_first_beacon[num_submaps] = num_beacons;
	index_of_submap[num_submaps] = glb_map.X.get_rows() + 1;
	num_beacons_in_submap[num_submaps] = 0;
	int j = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			//cout << "i: " << i << endl;
			xj = obsX.get(2*i + 4, 1);
			yj =  obsX.get(2*i + 5, 1);
			newX.set(2 * j + 1, 1, xr1 + xj * cos(fir1) - yj * sin(fir1));
			newX.set(2 * j + 2, 1, yr1 + yj * cos(fir1) + xj * sin(fir1));
			place_of_beacon[num_beacons] = glb_map.X.get_rows() + 1 + 2 * j;
			assosiations[i] = num_beacons;
			++num_matches;
			++num_beacons;
			++num_beacons_in_submap[num_submaps];
			++j;
		}
	}
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
	++num_submaps;
}

void MapFuser::update_map(const SparseMatrix& obsX, const SparseMatrix& obsP){
	int num_obs = (obsX.get_rows() - 3)/2;
	int index_robot1 = glb_map.X.get_rows() - 2 * num_beacons_in_submap[num_submaps - 1] - 5;
	int index_robot2 = glb_map.X.get_rows() - 2;
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
			xi = glb_map.X.get(place_of_beacon[assosiations[i]]		, 1);
			yi = glb_map.X.get(place_of_beacon[assosiations[i]] + 1	, 1);
			//cout << xi << " " << yi <<  " " << place_of_beacon[assosiations[i]] <<" " << assosiations[i] <<endl;
			jh.set(2 * i + 4, place_of_beacon[assosiations[i]], cos(fir1));
			jh.set(2 * i + 4, place_of_beacon[assosiations[i]] + 1, sin(fir1));
			jh.set(2 * i + 5, place_of_beacon[assosiations[i]], -sin(fir1));
			jh.set(2 * i + 5, place_of_beacon[assosiations[i]] + 1, cos(fir1));
		
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
	//cout << "here1" << endl;
	//jh.write_to_file("SavedMatrices/jh1");
	//cout << "here2" << endl;
	//H.write_to_file("SavedMatrices/H1");
	//glb_map.X.write_to_file("SavedMatrices/X");
	//obsX.write_to_file("SavedMatrices/obsX");
	//glb_map.i.write_to_file("SavedMatrices/iInnan");
	//cout << "rows/cols " << glb_map.I.rows << "/" << glb_map.I.cols << endl;

	SparseSymmMatrix I_new = to_sparse_symm_matrix(trn(jh) * inv(to_sparse_symm_matrix(obsP)) * jh);

	glb_map.I = glb_map.I + I_new;
	/*int i = 1;
	while(!I_new.first_in_row[i]){
		++i;
	}
	num_elements_updated_in_I = glb_map.I.rows - i;*/

	//cout << "here2" << endl;
	SparseMatrix z = obsX - H;
	z.set(3, 1, wrap(z.get(3,1)));
	glb_map.i = glb_map.i + trn(jh) * inv(to_sparse_symm_matrix(obsP)) * (z + jh * glb_map.X);
	//glb_map.I.write_to_file("SavedMatrices/I");
	//glb_map.i.write_to_file("SavedMatrices/i");
	//glb_map.L = cholesky(glb_map.I);
	//glb_map.X.print();
	//glb_map.X = solve_cholesky(glb_map.L, glb_map.i);
	//cout << endl << endl;
	//glb_map.X.print();
	//glb_map.X.write_to_file("SavedMatrices/Xaft");
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
	    uncertainty = 3 * max_eig(sqrt(cov));
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

