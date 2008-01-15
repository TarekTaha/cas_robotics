#include "MatrixFunctions.h"

#ifndef GLOBALMAP
#define GLOBALMAP

class GlobalMap{
	public:
		GlobalMap();
		SparseMatrix i;
		SparseSymmMatrix I;
		SparseMatrix L;
		SparseMatrix X;
};

#endif
