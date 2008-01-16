#include "MapFuser.h"

MapFuser::MapFuser(){}

double MapFuser::distance_to_submap(int map){
	double rx = glb_map.X.get( glb_map.X.rows - 2, 1);
	double ry = glb_map.X.get( glb_map.X.rows - 1, 1);
	double mx, my;
	if(map == 0){
		mx = 0;
		my = 0;
	}
	else{
		mx = glb_map.X.get( place_of_beacon[ submaps_first_beacon[map]] - 3 , 1);
		my = glb_map.X.get( place_of_beacon[ submaps_first_beacon[map]] - 2, 1);
	}
	return sqrt((rx - mx) * (rx - mx) + (ry - my) * (ry - my)); 
}

void MapFuser::set_potential_assosiations(){ //const Matrix& P_glb_robot, const Matrix& P_loc_robot){
	num_potential_assosiations = 0;
	double glb_rob_uncertanty = 0;//max_eig(sqrt(P_glb_robot));
	double loc_rob_uncertanty = 0;//max_eig(sqrt(P_loc_robot));
	for(int i = 0; i < num_submaps; ++i){
		//cout << "i: " << i << " " << distance_to_submap(i)<< "  "<< radius_of_submap[i] << " " <<  radius_of_submap[num_submaps] <<   global_robot_uncertainty[num_submaps - 1] << " " <<  global_robot_uncertainty[i]<< " " << local_robot_uncertainty[num_submaps - 1] << endl;
		if(distance_to_submap(i) < radius_of_submap[i] + radius_of_submap[num_submaps] + global_robot_uncertainty[num_submaps - 1]+ global_robot_uncertainty[i] + local_robot_uncertainty[num_submaps - 1] + 3){
			//cout << "Submap " << i << " is in " << num_beacons_in_submap[i] << endl;
			for(int j = 0; j < num_beacons_in_submap[i]; ++j){
				cout << "setting potential: " << submaps_first_beacon[i] + j << endl;
				potential_assosiation_beacons[num_potential_assosiations] = submaps_first_beacon[i] + j;
				++num_potential_assosiations;
			}
		}
	}
	cout << "potentiella" << endl;
	for(int i = 0; i < num_potential_assosiations; ++i){
		cout << potential_assosiation_beacons[i] << endl;
	}
	/*for(int i = 0; i <= num_beacons; ++i){
		potential_assosiation_beacons[i] = i;
	}
	num_potential_assosiations = num_beacons;*/
}

Matrix MapFuser::get_part_of_X_for_assositation(){
	int size = num_potential_assosiations;
	Matrix result(2*size + 3, 1);
	result.set(1, 1, glb_map.X.get(glb_map.X.rows - 2	, 1));
	result.set(2, 1, glb_map.X.get(glb_map.X.rows - 1	, 1));
	result.set(3, 1, glb_map.X.get(glb_map.X.rows		, 1));
	for(int i = 0; i < size; ++i){
		for(int k = 0; k < 2; ++k){
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
				if(x.first_in_row[rhs.rows - 2 + j]){
					result.values[j][2 * i + k + 3] = x.first_in_row[rhs.rows - 2 + j]->value;
				}
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
	for(int i = 0; i < 3; ++i){
		rhs.remove_all_elements();
		rhs.set(rhs.rows - 2 + i, 1, 1);
		x = solve_cholesky(glb_map.L, rhs);
		for(int j = 0; j < 3; ++j){
			if(x.first_in_row[rhs.rows - 2 + j]){
				result.values[j][i] = x.first_in_row[rhs.rows - 2 + j]->value;
			}
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
	if(m.P.values[0][0] < 1e-8 || m.P.values[1][1] < 1e-8 || m.P.values[2][2] < 1e-8){
		m.P.values[0][0] += 1e-8;
		m.P.values[1][1] += 1e-8;
		m.P.values[2][2] += 1e-8;
	}
	radius_of_submap[num_submaps] = submap_radius(m);
	
	Matrix obsP = m.P.get_sub_matrix(4,4, m.P.rows, m.P.columns);
	Matrix obsX = m.X.get_sub_matrix(4, 1, m.X.rows, 1);
	set_potential_assosiations();
	
	Matrix P = restore_part_of_P_for_assositation();
	global_robot_uncertainty[num_submaps] = 3 * max_eig(sqrt(P.get_sub_matrix(1, 1, 2, 2)));
	local_robot_uncertainty[num_submaps] = 3 * max_eig(sqrt(m.P.get_sub_matrix(1, 1, 2, 2)));
	P.get_sub_matrix( 1, 1,2, 2).print();
	Matrix X = get_part_of_X_for_assositation();

	Matrix beacP = trans_cov_matrix_to_local_cordinate_system(P,X);
	Matrix beacX = trans_state_matrix_to_local_cordinate_system(X);
	//beacP.write_to_file("SavedMatrices/beacP");
	//beacX.write_to_file("SavedMatrices/beacX");
	//obsP.write_to_file("SavedMatrices/obsP");
	//obsX.write_to_file("SavedMatrices/obsX");
	assosiate_beacons(beacX, beacP, obsX, obsP);
	add_new_beacons_and_robot_location_to_state(m.X);
	//to_dence_matrix(glb_map.X).write_to_file("SavedMatrices/glbX");
	
	update_map(m.X, m.P);
}

void MapFuser::fuse_first_map(LocalMap m){
	//add a bit to P to avoid singularity
	if(m.P.values[0][0] < 1e-8 || m.P.values[1][1] < 1e-8 || m.P.values[2][2] < 1e-8){
		m.P.values[0][0] += 1e-8;
		m.P.values[1][1] += 1e-8;
		m.P.values[2][2] += 1e-8;
	}

	radius_of_submap[0] = submap_radius(m);
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
    num_beacons_in_submap[0] = num_beacons;
    for(int i = 0; i <= num_beacons; ++i){
    	place_of_beacon[i] = 2 * i + 1;
    }
    submaps_first_beacon[0] = 0;
    global_robot_uncertainty[0] = 0;
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

	int num_obs = obsP.rows/2;
	int num_beac = beacP.rows/2;
	double r_obs, b_obs, r_beac, b_beac;
	Matrix cov_obs, cov_beac, totalcov;
	Matrix innov(2,1);
	double mahadist, mahadist_new;
	num_matches = 0;
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
	        //cout << "j: " << j << " " << mahadist_new << endl;
	        if(mahadist_new < mahadist){
	        	mahadist = mahadist_new;
	        	//9.2103 is the chi-square invers for 99% with 2 degrees of freedom
	        	if(mahadist < 9.2103){
	        		assosiations[j - 1] = i - 1;
	        	}
	        	else{
	        		assosiations[j] = -100;
	        	}
	        }
	    }
	}
	cout << "Matches: "<<endl;
	for(int j = 0; j < num_obs; ++j){
		if(assosiations[j] >= 0)
			assosiations[j] = potential_assosiation_beacons[assosiations[j]];
		cout << assosiations[j] << endl;
	}
}

void MapFuser::add_new_beacons_and_robot_location_to_state(const Matrix& obsX){
	int num_obs = (obsX.rows - 3)/2;
	double xr1 = glb_map.X.get(glb_map.X.rows - 2	, 1);
	double yr1 = glb_map.X.get(glb_map.X.rows - 1	, 1);
	double fir1 = glb_map.X.get(glb_map.X.rows		, 1);
	double xj;
	double yj;
	
	submaps_first_beacon[num_submaps] = num_beacons;

	num_beacons_in_submap[num_submaps] = 0;
	for(int i = 0; i < num_obs; ++i){
		if(assosiations[i] == -100){
			//cout << "i: " << i << endl;
			xj = obsX.get(2*i + 4, 1);
			yj =  obsX.get(2*i + 5, 1);
			glb_map.X.rows += 2;
			glb_map.I.rows += 2;
			glb_map.I.cols += 2;
			glb_map.i.rows += 2;
			glb_map.X.set(glb_map.X.rows - 1, 1, xr1 + xj * cos(fir1) - yj * sin(fir1));
			glb_map.X.set(glb_map.X.rows	, 1, yr1 + yj * cos(fir1) + xj * sin(fir1));
			place_of_beacon[num_beacons] = glb_map.X.rows - 1;
			assosiations[i] = num_beacons;
			++num_matches;
			++num_beacons;
			++num_beacons_in_submap[num_submaps];
		}
	}
	glb_map.X.rows += 3;
	glb_map.I.rows += 3;
	glb_map.I.cols += 3;
	glb_map.i.rows += 3;
	double xr2 = obsX.get(1	, 1);
	double yr2 = obsX.get(2	, 1);
	double fir2 = obsX.get(3, 1);
	glb_map.X.set(glb_map.X.rows - 2, 1, xr1 + xr2 * cos(fir1) - yr2 * sin(fir1));
	glb_map.X.set(glb_map.X.rows - 1, 1, yr1 + yr2 * cos(fir1) + xr2 * sin(fir1));
	glb_map.X.set(glb_map.X.rows	, 1, fir1 + fir2);
	
	++num_submaps;
}

void MapFuser::update_map(const Matrix& obsX, const Matrix& obsP){
	int num_obs = (obsX.rows - 3)/2;
	int index_robot1 = glb_map.X.rows - 2 * num_beacons_in_submap[num_submaps - 1] - 5;
	int index_robot2 = glb_map.X.rows - 2;
	double xi, yi;
	double xr1 = glb_map.X.get(index_robot1		, 1);
	double yr1 = glb_map.X.get(index_robot1 + 1	, 1);
	double fir1 = glb_map.X.get(index_robot1 + 2, 1);
	double xr2 = glb_map.X.get(index_robot2		, 1);
	double yr2 = glb_map.X.get(index_robot2 + 1	, 1);
	double fir2 = glb_map.X.get(index_robot2 + 2, 1);
	//cout << xr1 << " "<< yr1 << " "<< fir1 << " "<< xr2 << " "<< yr2 << " "<< fir2 << " " << endl;
	
	
	SparseMatrix jh(2 * num_obs + 3, glb_map.I.rows);
	Matrix H(2 * num_obs + 3, 1);
	
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
	//to_dence_matrix(jh).write_to_file("SavedMatrices/jh");
	//H.write_to_file("SavedMatrices/H");
	
	//cout << "rows/cols " << glb_map.I.rows << "/" << glb_map.I.cols << endl;
	cout << "here1" << endl;
	glb_map.I = glb_map.I + to_sparse_symm_matrix(trn(jh) * inv(obsP) * jh);
	cout << "here2" << endl;
	glb_map.i = glb_map.i + trn(jh) * inv(obsP) * (obsX - H + jh * glb_map.X);
	//to_dence_matrix(glb_map.I).write_to_file("SavedMatrices/I");
	//to_dence_matrix(glb_map.i).write_to_file("SavedMatrices/i");
	glb_map.L = cholesky(glb_map.I);
	glb_map.X = solve_cholesky(glb_map.L, glb_map.i);

}

double MapFuser::submap_radius(const LocalMap& map){
	int num_beac = (map.X.rows - 3)/2;

	double radius = 0;
	double distance, uncertainty;
	Matrix cov;

	for(int i = 1; i <= num_beac; ++i){
	    distance = sqrt((map.X.get(2 * i + 2, 1)- map.X.get(1,1)) * (map.X.get(2 * i + 2, 1)- map.X.get(1,1)) + (map.X.get(2 * i + 3, 1)- map.X.get(2,1)) * (map.X.get(2 * i + 3, 1)- map.X.get(2,1)) );
	    // uncertainty of the beacon
	    cov = map.P.get_sub_matrix(2 * i + 2, 2 * i + 2, 2 * i + 3, 2 * i + 3);
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

