#include "Matrix.h"
#include "SparseMatrix.h"
#include "SparseSymmMatrix.h"
#include "MatrixException.h"

Matrix to_dence_matrix(const SparseMatrix&);
Matrix to_dence_matrix(const SparseSymmMatrix& md);
SparseMatrix to_sparse_matrix(const Matrix& m);
SparseSymmMatrix to_sparse_symm_matrix(const Matrix& m);
SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m);

SparseMatrix cholesky(SparseSymmMatrix m);
SparseMatrix solve_cholesky(const SparseMatrix& L, SparseMatrix rhs);
SparseMatrix operator*(const SparseMatrix& spa, const Matrix& denc);
Matrix operator+(Matrix denc, const SparseMatrix& spa);
