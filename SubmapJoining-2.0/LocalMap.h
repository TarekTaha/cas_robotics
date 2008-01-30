#include "SparseMatrix.h"

#ifndef LOCALMAP
#define LOCALMAP

class LocalMap{
	public:
		LocalMap();
		LocalMap(const SparseMatrix& X, const SparseMatrix& P);
		SparseMatrix X;
		SparseMatrix P;
};

#endif
