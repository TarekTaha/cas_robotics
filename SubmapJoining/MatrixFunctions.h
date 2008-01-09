#include "Matrix.h"
#include "SparseMatrix.h"
#include "SparseSymmMatrix.h"

Matrix to_dence_matrix(const SparseMatrix&);
Matrix to_dence_matrix(const SparseSymmMatrix& md);

SparseMatrix cholesky(SparseSymmMatrix m);
SparseMatrix solve_cholesky(const SparseMatrix& L, SparseMatrix rhs);
SparseMatrix operator*(const SparseMatrix& spa, const Matrix& denc);
