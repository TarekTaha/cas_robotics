#ifndef SPARSESYMMMATRIX
#define SPARSESYMMMATRIX

#include <iostream.h>
#include "SparseMatrixElement.h"
#include "MatrixException.h"

class SparseSymmMatrix{
  public:
    SparseSymmMatrix(int rows = 0, int cols = 0);
    SparseSymmMatrix(const SparseSymmMatrix&);
    ~SparseSymmMatrix();
    SparseSymmMatrix& operator=(const SparseSymmMatrix& m);
    SparseMatrixElement *first_in_row[1000];
    int rows;
    int cols;

    void set(int, int, double);
    void add(int, int, double);
    void print() const;
    void remove_all_elements();
    //void print();
    //void print_plain();
};

SparseSymmMatrix operator+(SparseSymmMatrix m1, const SparseSymmMatrix& m2);

#endif
