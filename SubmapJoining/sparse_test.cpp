#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"

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
	SparseSymmMatrix sB(sA);
	sB.set(6,6,6);

	Matrix dA, dB, dC;
  
	SparseSymmMatrix sC(2,2);
  
	for(int i = 0; i < 0; ++i){
		sA = random_symm_matrix(50,50, 1000);
		sB = random_symm_matrix(50,50, 1000);
		sC = sA + sB;

  
		dA = to_dence_matrix(sA);
		dB = to_dence_matrix(sB);
		dC = to_dence_matrix(sC);

		if(dC == dA + dB){}
		else{
			cout << "dA" << endl;
			dA.print();
			cout << "dB" << endl;
			dB.print();
			cout << "dC" << endl;
			dC.print();
			cout << "sA" << endl;
			sA.print();
			cout << "sB" << endl;
			sB.print();
			cout << "sC" << endl;
			sC.print();
		}
	}
	sC.set(1,1,2);
	sC.set(2,2,2);
	sC.set(1,2,1);
	cholesky(sC).print();
  cout << "finish" <<endl;


  return 1;
}
