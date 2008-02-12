#ifndef SPARSEMATRIX
#define SPARSEMATRIX

#include <iostream.h>
#include "cholmod.h"
#include <fstream>
#include "Matrix.h"

using namespace std;

class SparseMatrix : public Matrix{
  public:
	  SparseMatrix();
	  SparseMatrix(int rows, int cols, int zmax);
	  SparseMatrix(const SparseMatrix&);
	  virtual ~SparseMatrix();
	  SparseMatrix& operator=(const SparseMatrix& m);
	  cholmod_sparse *A;
	  mutable cholmod_common c;

	  virtual int get_cols() const;
	  virtual int get_rows() const;
	  virtual void set(int, int, double);
	  virtual double get(int, int) const;
	  virtual void read_from_file(const char* filename);
	  void read_from_delimited_file(const char* filename);
	  virtual void print_coord() const;
	  SparseMatrix get_submatrix(int*,int, int*, int) const;
	  SparseMatrix get_submatrix(int, int, int, int) const;
	  void clear();
	  int max_num_nonzero() const;
	  int num_nonzero() const;
    	
};

SparseMatrix operator+(const SparseMatrix&, const SparseMatrix&);
SparseMatrix operator-(const SparseMatrix&, const SparseMatrix&);
SparseMatrix operator*(const SparseMatrix&, const SparseMatrix&);
SparseMatrix operator*(double, const SparseMatrix&);
bool operator==(const SparseMatrix&, const SparseMatrix&);
SparseMatrix trn(const SparseMatrix&);
SparseMatrix sqrt(const SparseMatrix&);
SparseMatrix vertcat(const SparseMatrix&, const SparseMatrix&);
SparseMatrix horzcat(const SparseMatrix&, const SparseMatrix&);
SparseMatrix zeros(int rows, int cols, int nznew = 0);
SparseMatrix ones(int rows, int cols);
void reallocate(SparseMatrix&, int nznew);

#endif
