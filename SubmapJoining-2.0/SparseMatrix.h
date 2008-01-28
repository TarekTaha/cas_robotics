#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <iostream.h>
#include "cholmod.h"
#include <fstream>
#include "Matrix.h"

using namespace std;

class SparseMatrix : public Matrix{
  public:
    SparseMatrix(int rows = 0, int cols = 0, int zmax = 0);
    SparseMatrix(const SparseMatrix&);
    ~SparseMatrix();
    SparseMatrix& operator=(const SparseMatrix& m);
	cholmod_sparse *A;
	mutable cholmod_common c;

	virtual int get_cols() const;
	virtual int get_rows() const;
    virtual void set(int, int, double);
    virtual double get(int, int) const;
    virtual void print_coord() const;
};

SparseMatrix operator+(const SparseMatrix&, const SparseMatrix&);
SparseMatrix operator-(const SparseMatrix&, const SparseMatrix&);
SparseMatrix operator*(const SparseMatrix&, const SparseMatrix&);
SparseMatrix trn(const SparseMatrix&);

#endif
