#ifndef SPARSESYMMMATRIX
#define SPARSESYMMMATRIX

#include <iostream.h>
#include "SparseMatrixElement.h"
#include "MatrixException.h"
#include <fstream>

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
    double get(int row, int column);
    void add(int, int, double);
    void print() const;
    void remove_all_elements();
    void write_to_file(const char* filename);
    void write_to_file_coord(const char* filename) const;
    void extract_sub_segment(int row, int from_col, int to_col, SparseMatrixElement **start, SparseMatrixElement *end,  int row_num_change, int col_num_change);
    void remove_row(int row);
    void plus_equals(SparseSymmMatrix& m);
    //void print();
    //void print_plain();
};

SparseSymmMatrix operator+(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2);
SparseSymmMatrix operator-(SparseSymmMatrix m1, const SparseSymmMatrix& m2);

#endif
