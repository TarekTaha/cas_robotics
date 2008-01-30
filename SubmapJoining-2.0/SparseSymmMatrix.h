#ifndef SPARSESYMMMATRIX
#define SPARSESYMMMATRIX

#include <iostream.h>
#include "cholmod.h"
#include <fstream>
#include "SparseMatrix.h"

using namespace std;

class SparseSymmMatrix : public SparseMatrix{
  public:
    SparseSymmMatrix(int rows = 0, int cols = 0, int zmax = 0);
    SparseSymmMatrix(const SparseSymmMatrix&);
    ~SparseSymmMatrix();
    SparseSymmMatrix& operator=(const SparseSymmMatrix& m);

    virtual void set(int, int, double);
    virtual double get(int, int) const;

};

SparseSymmMatrix to_symm(const SparseMatrix&);
SparseSymmMatrix operator+(const SparseSymmMatrix&, const SparseSymmMatrix&);
SparseSymmMatrix operator-(const SparseSymmMatrix&, const SparseSymmMatrix&);
SparseSymmMatrix operator*(const SparseSymmMatrix&, const SparseSymmMatrix&);
SparseSymmMatrix trn(const SparseSymmMatrix&);
SparseSymmMatrix eye(int size);
SparseSymmMatrix operator*(double, const SparseSymmMatrix&);

#endif
