#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <iostream.h>
#include "SparseMatrixElement.h"

class SparseMatrix{
  public:
    SparseMatrix(int rows = 0, int cols = 0);
    SparseMatrix(const SparseMatrix&);
    ~SparseMatrix();
    SparseMatrix& operator=(const SparseMatrix& m);
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

SparseMatrix operator+(SparseMatrix m1, const SparseMatrix& m2);

#endif
