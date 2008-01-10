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

	int size = 30;
	Matrix dA, dB, dC, db, dx;
  
	SparseSymmMatrix symmA, symmB, symmC;
	SparseMatrix spaA, spaB, spaC;
	SparseMatrix sb(size, 1), x;
  
	for(int i = 0; i < 0; ++i){
		spaA = random_matrix(size,5, 100);
		spaB = random_matrix(5,size, 2000);
		//cout << "mult start" << endl;
		//spaC = spaA*spaB;
		//cout << "mult end" << endl;
		/*spaA.print();
		cout << endl;
		spaB.print();
		cout << endl;
		spaC.print();*/
		/*for(int j = 1; j<= size; ++j){
			sA.set(j,j, 50 + (int)(30 * (double)rand()/(double)RAND_MAX));
			sb.set(j, 1, 1);
		}*/
		try{
			dA = to_dence_matrix(spaA);
			dB = to_dence_matrix(spaB);
			spaC = spaA*dB;
			dC = to_dence_matrix(spaC);
			/*spaA.print();
			cout << endl;
			dB.print();
			cout << endl;
			spaC.print();
			cout << endl;*/
			
			
			if(dA*dB == dC){
				//cout << "works" << endl;
				/*stringstream out;
				out << i;
				string tmp = "SavedMatrices/B" + out.str() + ".mat";
				dA.write_to_file(tmp.c_str());
				tmp = "SavedMatrices/C" + out.str() + ".mat";
				dB.write_to_file(tmp.c_str());*/
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
	dA.read_from_file("SimulationData/localmap_1_st");
	dA.print();

  return 1;
}
