#ifndef DENSEMATRIX
#define DENSEMATRIX

#include <iostream.h>
#include "cholmod.h"
#include <fstream>
#include "Matrix.h"

using namespace std;

class DenseMatrix : public Matrix{
  public:
    DenseMatrix(int rows = 0, int cols = 0);
    DenseMatrix(const DenseMatrix&);
    ~DenseMatrix();
    DenseMatrix& operator=(const DenseMatrix& m);
	cholmod_dense *A;
	mutable cholmod_common c;

	virtual int get_cols() const;
	virtual int get_rows() const;
    virtual void set(int, int, double);
    virtual double get(int, int) const;
    void print_coord();
};

//DenseMatrix operator+(const DenseMatrix&, const DenseMatrix&);
//DenseMatrix operator-(const DenseMatrix&, const DenseMatrix&);
//DenseMatrix operator*(const DenseMatrix&, const DenseMatrix&);
//DenseMatrix trn(const DenseMatrix&);

#endif
