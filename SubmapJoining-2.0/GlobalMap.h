#include "MatrixFunctions.h"

#ifndef GLOBALMAP
#define GLOBALMAP

class GlobalMap{
	public:
		GlobalMap();
		SparseMatrix i;
		SparseSymmMatrix I;
		CholeskyFactor L;
		SparseMatrix X;
};

#endif
