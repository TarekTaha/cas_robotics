#include <stdlib.h>
#include <iostream.h>
#include "MatrixFunctions.h"

using namespace std;

int  main(int argc, char *argv[]){

  SparseMatrix sA(6,6);
  sA.set(1,1, 1);
  sA.set(2,2, 1);
  sA.set(2,4, 1);
  SparseMatrix sB(sA);
  sA.set(2,5, 2);
  sA.set(6,4, 2);
  sB.set(6,6,6);

  

  
  Matrix dA = sparse_matrix_to_dence_matrix(sA);

  
  cout << "sA" << endl;
  sA.print();
  cout << "sB" << endl;
  sB.print();

  return 1;
}
