#include "MapFuser.h"

MapFuser::MapFuser(){}

void MapFuser::set_potential_assosiations(){
	for(int i = 0; i < num_beacons; ++i){
		potential_assosiation_beacons[i] = i;
	}
	num_potential_assosiations = num_beacons;
}

Matrix MapFuser::get_part_of_X_for_assositation(){
	int size = num_potential_assosiations;
	Matrix result(2*size + 3, 1);
	result.set(1, 1, glb_map.X.get(glb_map.X.rows - 2	, 1));
	result.set(2, 1, glb_map.X.get(glb_map.X.rows - 1	, 1));
	result.set(3, 1, glb_map.X.get(glb_map.X.rows		, 1));
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			cout << " i: " << i << " "<< place_of_beacon[potential_assosiation_beacons[i]] << endl;
			result.set(2 * i + k + 4, 1, glb_map.X.get(place_of_beacon[potential_assosiation_beacons[i]] + k, 1));
		}
	}
	return result;
}

Matrix MapFuser::restore_part_of_P_for_assositation(){
	int size = num_potential_assosiations;
	Matrix result(2*size + 3, 2*size + 3);
	SparseMatrix rhs(glb_map.I.rows, 1);
	SparseMatrix x;
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
			rhs.remove_all_elements();
			rhs.set(place_of_beacon[potential_assosiation_beacons[i]] + k, 1, 1);
			x = solve_cholesky(glb_map.L, rhs);
			for(int j = 0; j < 3; ++j){
				if(x.first_in_row[rhs.rows - 2 + j])
					result.values[j][2 * i + k + 3] = x.first_in_row[rhs.rows - 2 + j]->value;
			}
			for(int j = 0; j < size; ++j){
				for(int m = 0; m < 2; ++m){
					if(x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]){
						//cout << "m : " << x.first_in_row[2 * potential_assosiation_beacons[j] + 1 + m]->value << endl;
						result.values[2 * j + m + 3][2 * i + k + 3] = x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]->value;
					}
				}
			}	
		}
	}
	for(int i = 0; i < 2; ++i){
		rhs.remove_all_elements();
		rhs.set(rhs.rows - 2 + i, 1, 1);
		x = solve_cholesky(glb_map.L, rhs);
		for(int j = 0; j < 3; ++j){
			if(x.first_in_row[rhs.rows - 2 + j])
				result.values[j][i] = x.first_in_row[rhs.rows - 2 + j]->value;
		}
		for(int j = 0; j < size; ++j){
			for(int m = 0; m < 2; ++m){
				if(x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]){
					//cout << "m : " << x.first_in_row[2 * potential_assosiation_beacons[j] + 1 + m]->value << endl;
					result.values[2 * j + m + 3][i] = x.first_in_row[place_of_beacon[potential_assosiation_beacons[j]] + m]->value;
				}
			}
		}	
	}

	return result;
}

void MapFuser::fuse_map(LocalMap m){
	Matrix obsP = m.P.get_sub_matrix(4,4, m.P.rows, m.P.columns);
	Matrix obsX = m.X.get_sub_matrix(4, 1, m.X.rows, 1);
	set_potential_assosiations();
	
	Matrix P = restore_part_of_P_for_assositation();
	Matrix X = get_part_of_X_for_assositation();
	Matrix beacP = trans_cov_matrix_to_local_cordinate_system(P,X);
	Matrix beacX = trans_state_matrix_to_local_cordinate_system(X);
}

void MapFuser::fuse_first_map(LocalMap m){
	cout << "here1" << endl;
	//add a bit to P to avoid singularity
	m.P.values[0][0] += 1e-8;
	m.P.values[1][1] += 1e-8;
	m.P.values[2][2] += 1e-8;
	cout << "here1" << endl;
	m.X.print();
	cout << "here3" << endl;
    trans_state_matrix_to_local_cordinate_system(m.X).print();
	cout << "here2" << endl;
	double temp1 = m.X.values[0][0];
	double temp2 = m.X.values[1][0];
	double temp3 = m.X.values[2][0];
	for(int j = 0; j < m.X.rows - 3; ++j){
		m.X.values[j][0] = m.X.values[j + 3][0];
	}
	m.X.values[m.X.rows - 3][0] = temp1;
	m.X.values[m.X.rows - 2][0] = temp2;
	m.X.values[m.X.rows - 1][0] = temp3;
	for(int i = 0; i < m.P.columns; ++i){
		temp1 = m.P.values[0][i];
		temp2 = m.P.values[1][i];
		temp3 = m.P.values[2][i];
		for(int j = 0; j < m.P.rows - 3; ++j){
    		m.P.values[j][i] = m.P.values[j + 3][i];
    	}
		m.P.values[m.P.rows - 3][i] = temp1;
		m.P.values[m.P.rows - 2][i] = temp2;
		m.P.values[m.P.rows - 1][i] = temp3;
	}
	for(int i = 0; i < m.P.rows; ++i){
		temp1 = m.P.values[i][0];
		temp2 = m.P.values[i][1];
		temp3 = m.P.values[i][2];
		for(int j = 0; j < m.P.columns - 3; ++j){
    		m.P.values[i][j] = m.P.values[i][j + 3];
    	}
		m.P.values[i][m.P.columns - 3] = temp1;
		m.P.values[i][m.P.columns - 2] = temp2;
		m.P.values[i][m.P.columns - 1] = temp3;
	}

    glb_map.I = to_sparse_symm_matrix(inv(m.P));
    glb_map.i = to_sparse_matrix(inv(m.P)* m.X);
    glb_map.L = cholesky(glb_map.I);
    glb_map.X = to_sparse_matrix(m.X);
    
    num_submaps = 1;
    num_beacons = (m.P.rows - 3)/2;
    for(int i = 0; i < num_beacons; ++i){
    	place_of_beacon[i] = 2 * i + 1;
    }
    submaps_first_beacon[0] = 0;
}

Matrix MapFuser::trans_cov_matrix_to_local_cordinate_system(const Matrix& P, const Matrix& X){
	//robot pos is first in X and will be (0,0,0) in the new system 
	double xr = X.get(1, 1);
	double yr = X.get(2, 1);
	double fir = X.get(3, 1);
	Matrix jh(X.rows - 3, X.rows);
	int num_beacons = (X.rows -3)/2;
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

Matrix MapFuser::trans_state_matrix_to_local_cordinate_system(const Matrix& X){
	
	double xr = X.get(1, 1);
	double yr = X.get(2, 1);
	double fir = X.get(3, 1);
	int num_beacons = (X.rows -3)/2;
	double dx, dy;
	Matrix result(X.rows - 3, 1);
	for(int i = 1; i <= num_beacons; ++i){
		dx=X.get(2*i + 2, 1) - xr;
		dy=X.get(2*i + 3, 1) - yr;    

		result.set(2*i-1, 1, cos(fir)*dx+sin(fir)*dy);
		result.set(2*i, 1, -sin(fir)*dx+cos(fir)*dy);
	}
	return result;
}

void MapFuser::assosiate_beacons(const Matrix& beacX, const Matrix& beacP, const Matrix& obsX, const Matrix& obsP){

	int num_obs = obsP.rows;
	int num_beac = beacP.rows;
	double r_obs, b_obs, r_beac, b_beac;
	Matrix cov_obs, cov_beac, totalcov;
	Matrix innov(2,1);
	double mahadist, mahadist_new;
	for(int j = 1; j <= num_obs; ++j){
		mahadist = 1e8;
	    r_obs = obsX.get(j*2-1, 1);
	    b_obs = obsX.get(j*2, 1);
	    cov_obs = obsP.get_sub_matrix(j*2-1,j*2-1,j*2,j*2);  // covariance matrix of the j-th obs
	    for(int i = 1; i <= num_beac; ++i){
	        r_beac = beacX.get(i*2-1, 1);
	        b_beac = beacX.get(i*2, 1);
	        cov_beac = beacP.get_sub_matrix(i*2-1,i*2-1,i*2,i*2);  // covariance matrix of the i-th beac
	        // compute Mahalanobis distance from j-th obs to i-th beac
	        innov.set(1,1, r_beac-r_obs);
	        innov.set(2,1, b_beac-b_obs);
	        totalcov = cov_beac + cov_obs;
	        mahadist_new  = (trn(innov)*inv(totalcov)*innov).get(1,1);
	        if(mahadist_new < mahadist){
	        	mahadist = mahadist_new;
	        	assosiations[j] = i;
	        }
	    }
	}
	for(int j = 1; j <= num_obs; ++j){
		cout << assosiations[j] << " " << j << endl;
	}
}

