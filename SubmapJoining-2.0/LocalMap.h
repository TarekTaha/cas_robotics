#include "SparseMatrix.h"

#ifndef LOCALMAP
#define LOCALMAP

class LocalMap{
	public:
		LocalMap();
		LocalMap(const SparseMatrix& X, const SparseMatrix& P, const SparseMatrix& true_index = SparseMatrix());
		SparseMatrix X;
		SparseMatrix P;
		SparseMatrix true_index;
};

#endif
