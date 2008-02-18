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
		//for AMD reordering
		SparseSymmMatrix I_small;
		int order[10000];
};

#endif
