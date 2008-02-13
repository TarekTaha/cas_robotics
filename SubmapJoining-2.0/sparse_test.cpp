#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"
#include <sstream>
#include "Permutation.h"

using namespace std;

SparseMatrix random_matrix(int rows, int cols, int num_nonzero){
	SparseMatrix result(rows, cols, num_nonzero);
	int tmp_row, tmp_col;
	double tmp_value;
	for(int i = 0; i < num_nonzero; ++i){
		tmp_row = (int)(rows * (double)rand()/(double)RAND_MAX) + 1;
		tmp_col = (int)(cols * (double)rand()/(double)RAND_MAX) + 1;
		tmp_value = (int)(5 * (double)rand()/(double)RAND_MAX) + 1;
		result.set(tmp_row, tmp_col, tmp_value);
	}
	return result;
}

SparseSymmMatrix random_symm_matrix(int rows, int cols, int num_nonzero){
	SparseSymmMatrix result(rows, cols, num_nonzero);
	int tmp_row, tmp_col;
	double tmp_value;
	for(int i = 0; i < num_nonzero; ++i){
		tmp_row = (int)(rows * (double)rand()/(double)RAND_MAX) + 1;
		tmp_col = (int)(cols * (double)rand()/(double)RAND_MAX) + 1;
		//cout << tmp_row << " "<< tmp_col << endl; 
		tmp_value = (int)(5 * (double)rand()/(double)RAND_MAX) + 1;
		result.set(tmp_row, tmp_col, tmp_value);
		//cout << "Check: " << cholmod_check_sparse(result.A, &result.c) << endl;
	}
	return result;
}

int main(int argc, char *argv[]){

	int size = 200;
	//Matrix dA, dB, dC, db, dx;
	
	clock_t clock_start;
	clock_start = clock();
	
  
	SparseSymmMatrix symmA, symmB, symmC;
	SparseMatrix spaA, spaB, spaC, spaL;
	SparseMatrix sb, x;
	CholeskyFactor L_large, L_small;
	//Timer timer;
  
	for(int i = 0; i < 0; ++i){
		if(i%1 == 0){
			cout << i<< endl;
		}
		symmA = random_symm_matrix(size,size, size*size);


		for(int j = 1; j<= size; ++j){
			symmA.set(j,j, 50 + (int)(30 * (double)rand()/(double)RAND_MAX));
		}


		
		try{
			sb = ones(size, 1);
			//symmA.print();
			
			L_large = cholesky(symmA);
			//solve_cholesky(L_large, sb);
			//spaB = to_sparse_matrix(L_large);

			//spaB.print();
			//cholmod_check_factor(L_large.A, &L_large.c);

			L_small = cholesky(to_sparse_symm_matrix(to_sparse_matrix_fast(symmA).get_submatrix(1, 1, size/2, size/2)));
			//cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, L_large.A, &L_large.c);
			//cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, L_small.A, &L_small.c);
			//compare(L_large, L_small);
			//cout << "large" << endl;
			//L_large.print();
			//cout << "small" << endl;
			//L_small.print();
		
			spaA = SparseMatrix(size - (int)(size/2), size, (size - (int)(size/2)) * size);
			//cout << "h" << endl;
			for(int i = 1; i <= size - (int)(size/2); ++i){
				for(int j = 1; j <= size; ++j){
					if(i + (int)(size/2) >= j){
						//L_large.get(1, 1);
						//cout << "i: " <<i + (int)(size/2)<< " j: " << j<< endl;
						spaA.set(i, j, L_large.get(i + (int)(size/2), j));
					}
				}
			}
			//cout << "h" << endl;
			//cout << "spaA" << endl;
			//spaA.print();
			//cout << "append" << endl;
			L_small = append(L_small, spaA, L_large);
			//cout << "after append" << endl;
			//cout << "small after" << endl;
			//L_small.print();
			
			//if(cholmod_check_factor(L_small.A, &L_small.c)){
			if(solve_cholesky(L_large, sb) == solve_cholesky(L_small, sb)){
				//cout << "works" << endl;
	
			}
			else{
				cout << "dosen't work" << endl;

			}
		}
		catch(...){
			cout << "Unexpected error" << endl;
		}
	}

	cout << "finish! The test took (clock):" <<endl;
	cout << (double)(1000 * (clock() - clock_start))<< endl;

	SparseMatrix m = random_matrix(5, 5, 10);
	
	int ip1[5] = {1, 2, 3, 4, 0};
	int ip2[5] = {3, 4, 1, 0, 2};
	Permutation p(5, ip1);
	Permutation p2(5, ip2);
	m.print();
	cout << endl;
	m.get_submatrix(p, p).get_submatrix(p2, p2).print();
	cout << endl;
	m.get_submatrix(p2 + p, p2 + p).print();
	
	
	return 1;
}
