#include <iostream>
#include <stdlib.h>
#include "MatrixLib.h"

using namespace std;



int main(int argc, char * argv[])
{
  int row_ptr[6] = {0, 3, 6, 9, 10 , 12};
  double values[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
  int col_ind[12] = {0, 1, 4, 0, 1, 2, 1, 2, 4, 3, 0, 4};

  SparseMatrix A;
  SparseMatrix B;
  SparseMatrix C;
  SparseMatrix D;
  
  Matrix F(3,2);
  F.set(1,1, 0);
  F.set(1,2, 1);
  F.set(2,1, 3);
  F.set(2,2, 1);
  F.set(3,1, 4);
  F.set(3,2, 0);

  A.set(1,1, 16);
  A.set(1,2, 4);
  A.set(1,3, 1);
  A.set(2,1, 4);
  A.set(2,2, 16);
  A.set(2,3, 4);
  A.set(3,1, 1);
  A.set(3,2, 4);
  A.set(3,3, 16);
  
  
  /*B.set(1,1, 4);
  B.set(1,3, 1);
  B.set(1,6, 3);
  B.set(2,2, 4);
  B.set(2,2, 4);
  B.set(3,1, 1);
  B.set(3,3, 5);
  B.set(3,5, 3);
  B.set(4,4, 3);
  B.set(4,6, 1);
  B.set(5,3, 3);
  B.set(5,5, 5);
  B.set(6,1, 3);
  B.set(6,4, 1);
  B.set(6,6, 9);*/
  int rand1, rand2;
  double rand3;
  Matrix dB,db,dc, dD;
  SparseMatrix b, c;
  cout << "start" << endl;
  for(int j = 1; j <= 100; ++j){
	  cout << "loop1" << endl;
	  B.remove_all_elements();
	  for(int i = 1; i <= 30; ++i){
		  B.set(i,i,10);
		  rand1 = (int)(30 * (double)rand()/(double)RAND_MAX) + 1;
		  rand2 = (int)(30 * (double)rand()/(double)RAND_MAX) + 1;
		  rand3 = 5 * (double)rand()/(double)RAND_MAX;
		  B.set(rand1, rand2, rand3);
		  B.set(rand2, rand1, rand3);
	  }
	  B.write_to_file("B.mat");

	  D = cholesky(B);
	  D.print();
	  for(int i = 1; i <= 30; ++i)
		  b.set(i,1,1);
	  cout << "loop1.1" << endl;
	  c = solve_cholesky(D, b);
	  cout << "loop2" << endl;
	  cout << "c" << endl;
	  c.print();
	  c.trn();
	  cout << "c2" << endl;
	  c.print();


	  dB = sparce_matrix_to_dence_matrix(B);
	  db = sparce_matrix_to_dence_matrix(b);
	  dc = sparce_matrix_to_dence_matrix(c);
  

	  cout << "dc" << endl;
	  trn((inv(dB)*db)).print();
	  cout << "equal?" << (dc == trn(inv(dB)*db)) << endl;
  
	  
	  if(!(dc == trn(inv(dB)*db))){
		  B.write_to_file(j + ".mat");
	  }
  }
  return 1;
}
