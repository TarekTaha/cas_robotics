#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"
#include <sstream>

using namespace std;

SparseMatrix random_matrix(int rows, int cols, int num_nonzero){
	SparseMatrix result(rows, cols);
	int tmp_row, tmp_col, tmp_value;
	for(int i = 0; i < num_nonzero; ++i){
		tmp_row = (int)(rows * (double)rand()/(double)RAND_MAX) + 1;
		tmp_col = (int)(cols * (double)rand()/(double)RAND_MAX) + 1;
		tmp_value = (int)(9 * (double)rand()/(double)RAND_MAX) + 1;
		result.set(tmp_row, tmp_col, tmp_value);
	}
	return result;
}

SparseSymmMatrix random_symm_matrix(int rows, int cols, int num_nonzero){
	SparseSymmMatrix result(rows, cols);
	int tmp_row, tmp_col;
	double tmp_value;
	for(int i = 0; i < num_nonzero; ++i){
		tmp_row = (int)(rows * (double)rand()/(double)RAND_MAX) + 1;
		tmp_col = (int)(cols * (double)rand()/(double)RAND_MAX) + 1;
		//cout << tmp_row << " "<< tmp_col << endl; 
		tmp_value = (int)(9 * (double)rand()/(double)RAND_MAX) + 1;
		result.set(tmp_row, tmp_col, tmp_value);
	}
	return result;
}

int main(int argc, char *argv[]){

	SparseSymmMatrix sA(6,6);
	sA.set(1,1, 1);
	sA.set(2,2, 1);
	sA.set(2,4, 1);

	sA.set(2,5, 2);
	sA.set(2,6, 2);
	sA.set(2,6, 1);
	sA.set(6,4, 2);
	sA.add(6,4, 1);


	Matrix dA, dB, dC;
  
	SparseSymmMatrix sC(2,2);
	SparseMatrix sB;
  
	for(int i = 0; i < 10; ++i){
		sA = random_symm_matrix(10,10, 20);
		sC.set(1,1,2);
		sC.set(2,2,2);
		sC.set(1,2,1);
		//sA = sC;
		sB = cholesky(sA);
  
		dA = to_dence_matrix(sA);
		dB = to_dence_matrix(sB);
		dC = trn(dB) * dB;

		if(dC == dA){}
		else{
			stringstream out;
			out << i;
			string tmp = "SavedMatrices/A" + out.str() + ".mat";
			dA.write_to_file("SavedMatrices/A.mat");
		}
	}
	cout << "finish" <<endl;


  return 1;
}
