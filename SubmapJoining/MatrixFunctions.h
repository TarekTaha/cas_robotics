#include "Matrix.h"
#include "SparseMatrix.h"
#include "SparseSymmMatrix.h"

Matrix to_dence_matrix(const SparseMatrix&);
Matrix to_dence_matrix(const SparseSymmMatrix& md);

SparseMatrix cholesky(SparseSymmMatrix m);
