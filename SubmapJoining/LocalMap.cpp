#include "LocalMap.h"

LocalMap::LocalMap(){}

LocalMap::LocalMap(const Matrix& X, const Matrix& P){
	this->X = X;
	this->P = P;
}
