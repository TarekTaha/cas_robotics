#ifndef MATRIXFUNCTIONS
#define MATRIXFUNCTIONS

#include "SparseSymmMatrix.h"
#include "CholeskyFactor.h"
#include "cholmod.h"

#define EXHIBIT_SYMPTOM 0

//Matrix to_dence_matrix(const SparseMatrix&);
//Matrix to_dence_matrix(const SparseSymmMatrix& md);
//SparseMatrix to_sparse_matrix(const Matrix& m);
//SparseSymmMatrix to_sparse_symm_matrix(const Matrix& m);
//SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m);

CholeskyFactor cholesky(const SparseSymmMatrix& m);
//void part_cholesky(SparseMatrix& L, SparseSymmMatrix m, int size, Timer& timer);
SparseMatrix solve_cholesky(const CholeskyFactor& L, const SparseMatrix& rhs);
SparseSymmMatrix inv(const SparseSymmMatrix&);
//SparseMatrix inv(const SparseMatrix&);
SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m);
SparseMatrix to_sparse_matrix_fast(const SparseSymmMatrix& m);
SparseMatrix to_sparse_matrix(const SparseSymmMatrix& m);
double max_eig(const SparseSymmMatrix&);
SparseSymmMatrix sqrt(const SparseSymmMatrix&);
//SparseMatrix solve_cholesky2(const SparseMatrix& L, SparseMatrix rhs, Timer& timer);
//SparseMatrix operator*(const SparseMatrix& spa, const Matrix& denc);
//Matrix operator+(Matrix denc, const SparseMatrix& spa);
//SparseSymmMatrix LtimesLtrn(const SparseMatrix& L, int row_start);

//SparseMatrix extract_sub_matrix(SparseSymmMatrix& m, int from_row, int from_col, int to_row, int to_col);
//void set(SparseSymmMatrix& set_m, int row, int col, const SparseMatrix& m);

#endif
