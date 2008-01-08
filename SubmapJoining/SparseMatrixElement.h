

#ifndef SPARSEMATRIXELEMENT
#define SPARSEMATRIXELEMENT

#include <iostream.h>

using namespace std;

class SparseMatrixElement{	
  public:
    SparseMatrixElement(int row = 0, int col = 0, double value = 0, SparseMatrixElement *next_int_row = 0);
    int row;
    int col;
    double value;
    SparseMatrixElement *next_in_row;

};


#endif
