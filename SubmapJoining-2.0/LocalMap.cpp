#include "LocalMap.h"

LocalMap::LocalMap(){}

LocalMap::LocalMap(const SparseMatrix& X, const SparseMatrix& P, const SparseMatrix& true_index){
	this->X = X;
	this->P = P;
	this->true_index = true_index;
}
