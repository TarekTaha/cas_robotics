#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"
#include <sstream>

using namespace std;

SparseMatrix random_matrix(int rows, int cols, int num_nonzero){
	SparseMatrix result(rows, cols);
	int tmp_row, tmp_col;
	double tmp_value;
	for(int i = 0; i < num_nonzero; ++i){
		tmp_row = (int)(rows * (double)rand()/(double)RAND_MAX) + 1;
		tmp_col = (int)(cols * (double)rand()/(double)RAND_MAX) + 1;
		tmp_value = (9 * (double)rand()/(double)RAND_MAX) + 1;
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
		tmp_value = (int)(5 * (double)rand()/(double)RAND_MAX) + 1;
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

	int size = 50;

	Matrix dA, dB, dC, db, dx;
  
	SparseSymmMatrix sC(2,2);
	SparseMatrix sB, L;
	SparseMatrix sb(size, 1), x;
  
	for(int i = 0; i < 1000; ++i){
		sA = random_symm_matrix(size,size, 1000);
		for(int j = 1; j<= size; ++j){
			sA.set(j,j, 50 + (int)(30 * (double)rand()/(double)RAND_MAX));
			sb.set(j, 1, 1);
		}
		try{
			dA = to_dence_matrix(sA);
			L = cholesky(sA);
			x = solve_cholesky(L, sb);
			

			//dB = to_dence_matrix(sB);
			dC = to_dence_matrix(trn(sB));
			//dC = trn(dB) * dB;
			db = to_dence_matrix(sb);
			dx = to_dence_matrix(x);
			//x.print();
			//(inv(trn(dB))*db).print();
			//(inv(dA)*db).print();
			//dx.print();
			if(inv(dA)*db == dx){
				//cout << "works" << endl;
				stringstream out;
				out << i;
				string tmp = "SavedMatrices/B" + out.str() + ".mat";
				dA.write_to_file(tmp.c_str());
				tmp = "SavedMatrices/C" + out.str() + ".mat";
				dB.write_to_file(tmp.c_str());
			}
			else{
				cout << "dosen't work" << endl;
				stringstream out;
				out << i;
				string tmp = "SavedMatrices/A" + out.str() + ".mat";
				dA.write_to_file(tmp.c_str());
			}
		}
		catch(MatrixException me){
			cout << me.what() << endl;
			cout << "Not excisting: " << i << endl;
			stringstream out;
			out << i;
			string tmp = "SavedMatrices/D" + out.str() + ".mat";
			dA.write_to_file(tmp.c_str());
		}
	}
	cout << "finish" <<endl;


  return 1;
}
