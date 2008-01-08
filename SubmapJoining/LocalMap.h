#include "Matrix.h"

#ifndef LOCALMAP
#define LOCALMAP

class LocalMap{
	public:
		LocalMap();
		LocalMap(const Matrix& X, const Matrix& P);
		Matrix X;
		Matrix P;
};

#endif
