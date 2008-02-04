#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"
#include <sstream>

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
  
	for(int i = 0; i < 1000; ++i){
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

/*	symmA = random_symm_matrix(4,4,16);
	symmA.set(1, 1, 4);
	symmA.set(2, 1, 1);
	symmA.set(3, 1, 1);
	symmA.set(1, 2, 1);
	symmA.set(2, 2, 4);
	symmA.set(3, 2, 1);
	symmA.set(1, 3, 1);
	symmA.set(2, 3, 1);
	symmA.set(3, 3, 4);

	symmB = random_symm_matrix(2,2,4);
	symmB.set(1, 1, 4);
	symmB.set(2, 1, 1);
	symmB.set(1, 2, 1);
	symmB.set(2, 2, 4);
	SparseMatrix hej(1, 3, 3);
	spaA = hej;

	CholeskyFactor L = cholesky(symmB);
	CholeskyFactor Lb = cholesky(symmA);*/
	/*for(int i = 1; i <= 2; ++i){
		for(int j = 1; j <= 4; ++j){
			if(j != 4 || i != 1){
				spaA.set(i, j, Lb.get(i + 2, j));
			}
		}
	}*/
	/*for(int i = 1; i <= 3; ++i){
		spaA.set(1, i, Lb.get(3, i));
	}*/
	/*cout << "Lb" << endl;
	Lb.print();
	
	cout << "L stuff" << endl;
	int *colcou = (int*)L.A->ColCount;
	for(int i = 0; i < L.A->n; ++i){
		cout << colcou[i] << endl;
	}
	cout <<"nz" << endl;
	int *nz = (int*)L.A->nz;
	for(int i = 0; i < L.A->n; ++i){
		cout << nz[i] << endl;
	}
	cout << "ordering" << endl;
	cout << L.A->ordering << endl;
	cout << "is_ll" << endl;
	cout << L.A->is_ll << endl;
	cout << "is_super" << endl;
	cout << L.A->is_super << endl;
	cout << "is_monotonic" << endl;
	cout << L.A->is_monotonic << endl;
	cout << "itype" << endl;
	cout << L.A->itype << endl;
	cout << "xtype" << endl;
	cout << L.A->xtype << endl;
	cout << "dtype" << endl;
	cout << L.A->dtype << endl;
	cout << "Valid Lb: " << cholmod_check_factor(Lb.A, &Lb.c)  << endl;*/
/*	CholeskyFactor Lc = append(L, spaA, Lb);
	
	SparseMatrix x_temp = solve_cholesky(Lc, ones(3,1));
	x_temp.print();*/
	

	/*int *row = (int*)L.A->i;
	int *col = (int*)L.A->p;
	double *x2 = (double*)L.A->x;
	int *next = (int*)L.A->next;
	int *prev = (int*)L.A->prev;
	cout << "info" << endl;
	for(int i = 0; i < L.A->n + 2; ++i){
		cout << col[i] << " ";
	}
	cout << endl;
	
	for(int i = 0; i < L.A->n + 2; ++i){
		cout << next[i] << " ";
	}
	cout << endl;
	for(int i = 0; i < L.A->n + 2; ++i){
		cout << prev[i] << " ";
	}
	cout << endl;*/
	//cout <<"end" << endl;
	
	
	
	
	
	return 1;
}
