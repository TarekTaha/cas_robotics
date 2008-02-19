#include "association.h"
/*
The association function can be changed to another function. Look at input and 
output below for a description.

Associates the observed beacons with the old beacons by a nearest neighbour test.

Input: 
beacX: the state vector of the selected beacons for data association (old beacons)
beacP: the covariance corresponding to beacX
obsX: the state vector of the newly observed beacons
obsP: the covariance corresponding to obsX
There is no robot position in the matrices

Output:
returns the number of matches which is the number of new beacons matched 
to old beacons or in other words the number of non -100 or -1 values in
associations.

associations must be set as the following example tells:
associations[0] = 1 means that the beacon with x and y values 
on row 1 and 2 in obsX should be matched with the beacon with x and y values 
on row 3 and 4 in beacX.
associations[3] = 2 means that the beacon with x and y values 
on row 7 and 8 in obsX should be matched with the beacon with x and y values 
on row 5 and 6 in beacX.
associations[0] = -100 means that the beacon with x and y values 
on row 1 and 2 in obsX is a new beacon.
associations[0] = -1 means that the beacon with x and y values 
on row 1 and 2 in obsX is a poor observation that should be skiped
in the update step.

*/
int associate_beacons(int *associations, const SparseMatrix& beacX, const SparseMatrix& beacP, const SparseMatrix& obsX, const SparseMatrix& obsP){
	int num_obs = obsP.get_rows()/2;
	int num_beac = beacP.get_rows()/2;
	double r_obs, b_obs, r_beac, b_beac;
	SparseSymmMatrix cov_obs, cov_beac, totalcov;
	SparseMatrix innov(2,1, 2);
	SparseMatrix trninnov(1,2, 2);
	double mahadist_min, mahadist_new;
	int mahadist_min_index, dist_min_index;
	double dist_min, dist_new;
	int num_matches = 0;
	

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
    		associations[j - 1] = mahadist_min_index - 1;
    	}
    	else if(dist_min < 0.5 && mahadist_min_index == dist_min_index){
    		++num_matches;
    		associations[j - 1] = mahadist_min_index - 1;
    	}
    	else if(dist_min < 1.5){ //true dist
    		associations[j - 1] = -1;
    	}
    	else{
    		associations[j - 1] = -100;
    	}
	}
	return num_matches;
}
