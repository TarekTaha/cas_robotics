#include <stdlib.h>
#include <iostream.h>
#include "SparseSymmMatrix.h"
#include "DenseMatrix.h"
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
	//SparseMatrix sb(size, 1), x;
	//Timer timer;
  
	for(int i = 0; i < 0; ++i){
		if(i%100 == 0){
			cout << i<< endl;
		}
		
		//cout << "Check: " << cholmod_check_sparse(A.A, &A.c) << endl;
		symmA = random_symm_matrix(size,size, 1000);
		symmB = random_symm_matrix(size,size, 1000);
		

		for(int j = 1; j<= size; ++j){
			symmA.set(j,j, 50 + (int)(30 * (double)rand()/(double)RAND_MAX));
			//sb.set(1, 1, (30 * (double)rand()/(double)RAND_MAX));
		}
		//cout << "NEW MATRICES" << endl;
		//cout << "A" << endl;
		///symmA.print();
		//cout << "B" << endl;
		//symmB.print();
		//spaL = cholesky(symmA, timer);
		//spaL.print();
		//x = solve_cholesky2(spaL, sb, timer);
		
		//cout << "L" << endl;
		//spaL.print();
		//cout << "rhs" << endl;
		//sb.print();
		//cout << "x" << endl;
		//x.print();
		
		//spaA.print();
		//cout << endl;
		//spaB.print();
		
		//spaA = random_matrix(size, size, 1000);
		//spaB = random_matrix(size, size, 1000);
		symmC = symmA * symmB;
		
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
				/*stringstream out;
				out << i;
				string tmp = "SavedMatrices/B" + out.str() + ".mat";
				dA.write_to_file(tmp.c_str());
				tmp = "SavedMatrices/C" + out.str() + ".mat";
				dB.write_to_file(tmp.c_str());*/
			}
			else{
				cout << "dosen't work" << endl;
				/*stringstream out;
				out << i;
				string tmp = "SavedMatrices/A" + out.str() + ".mat";
				dA.write_to_file(tmp.c_str());*/
			}
		}
		//catch(MatrixException me){
			//cout << me.what() << endl;
			//cout << "Not excisting: " << i << endl;
			/*stringstream out;
			out << i;
			string tmp = "SavedMatrices/D" + out.str() + ".mat";
			dA.write_to_file(tmp.c_str());*/
		//}
		catch(...){
			cout << "Unexpected error" << endl;
		}
	}

	cout << "finish! The test took (clock):" <<endl;
	cout << (double)(1000 * (clock() - clock_start))<< endl;
	
	cout << "dense" << endl;
	DenseMatrix dA(3,3);
	dA.set(2,2,3);
	dA.set(1,2,1);
	dA.print();


	return 1;
}
