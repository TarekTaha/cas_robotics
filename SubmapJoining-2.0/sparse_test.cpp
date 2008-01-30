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

	int size = 4;
	//Matrix dA, dB, dC, db, dx;
	
	clock_t clock_start;
	clock_start = clock();
	
  
	SparseSymmMatrix symmA, symmB, symmC;
	SparseMatrix spaA, spaB, spaC, spaL;
	SparseMatrix sb(size, 1, size), x;
	//Timer timer;
  
	/*for(int i = 0; i < 0; ++i){
		if(i%100 == 0){
			cout << i<< endl;
		}
		
		symmA = random_symm_matrix(size,size, 1000);
		symmB = random_symm_matrix(size,size, 1000);
		

		for(int j = 1; j<= size; ++j){
			symmA.set(j,j, 50 + (int)(30 * (double)rand()/(double)RAND_MAX));
		}

		
		try{
			//dA = to_dence_matrix(symmA);
			//dB = to_dence_matrix(symmB);
			//symmA.plus_equals(symmB);
			//dC = to_dence_matrix(symmA);
			//cout << "A after add" << endl;
			//symmA.print();
			//symmB.print();
			if(cholmod_check_sparse(symmA.A, &symmA.c)){
				//cout << "works" << endl;
	
			}
			else{
				cout << "dosen't work" << endl;

			}
		}
		catch(...){
			cout << "Unexpected error" << endl;
		}
	}*/

	cout << "finish! The test took (clock):" <<endl;
	cout << (double)(1000 * (clock() - clock_start))<< endl;
	spaA = random_matrix(2,2,4);
	symmA = random_symm_matrix(2,2,4);
	spaA.set(1,1, 4);
	spaA.set(1, 2, 1);
	spaA.set(2, 1, 1);
	spaA.set(2, 2, 4);
	symmA.set(1,1, 4);
	symmA.set(1, 2, 1);
	symmA.set(2, 1, 1);
	symmA.set(2, 2, 4);
	
	cout << "SymmA" << endl;
	symmA.print();
	cout << "spaA" << endl;
	spaA.print();
	cout << endl;
	
	cholesky(symmA).print();
	cout << endl;
	//cholesky(spaA).print();
	


	return 1;
}
