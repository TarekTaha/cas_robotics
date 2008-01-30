#include "LocalMap.h"

LocalMap::LocalMap(){}

LocalMap::LocalMap(const SparseMatrix& X, const SparseMatrix& P){
	this->X = X;
	this->P = P;
}
