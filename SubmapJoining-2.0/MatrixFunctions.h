#ifndef MATRIXFUNCTIONS
#define MATRIXFUNCTIONS

#include "SparseSymmMatrix.h"
#include "CholeskyFactor.h"
#include "cholmod.h"
#include "Timer.h"
#include "sorting.h"

#define EXHIBIT_SYMPTOM 0

Permutation reorder_AMD(const SparseSymmMatrix& m);
CholeskyFactor cholesky(const SparseSymmMatrix& m);
CholeskyFactor cholesky2(const SparseSymmMatrix& m, Timer& timer);
SparseMatrix solve_cholesky(const CholeskyFactor& L, const SparseMatrix& rhs);
SparseMatrix solve_cholesky2(const CholeskyFactor& L, const SparseMatrix& rhs, Timer& timer);
SparseSymmMatrix inv(const SparseSymmMatrix&);
SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m);
SparseMatrix to_sparse_matrix_fast(const SparseSymmMatrix& m);
SparseMatrix to_sparse_matrix(const SparseSymmMatrix& m);
SparseMatrix to_sparse_matrix(const CholeskyFactor&);
double max_eig(const SparseSymmMatrix&);
SparseSymmMatrix sqrt(const SparseSymmMatrix&);
CholeskyFactor append(CholeskyFactor m1, const SparseMatrix& m2, const CholeskyFactor&);
CholeskyFactor to_factor(const SparseMatrix& result);

void compare(const CholeskyFactor& L1, const CholeskyFactor& L2);
#endif
