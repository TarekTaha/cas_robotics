#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <iostream.h>
#include "SparseMatrixElement.h"
#include "MatrixException.h"
#include <fstream>

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
    double get(int row, int column);
    void add(int, int, double);
    void print() const;
    void print_coord_format() const;
    void remove_all_elements();
    void remove_row(int row);
    void write_to_file(const char* filename);
    //void print();
    //void print_plain();
};

SparseMatrix operator+(const SparseMatrix& m1, const SparseMatrix& m2);
SparseMatrix trn(const SparseMatrix&);
SparseMatrix operator*(const SparseMatrix& m1, const SparseMatrix& m2);
#endif
