#ifndef ASSOCIATION
#define ASSOCIATION


#include "MatrixFunctions.h"


//The value of the chi-square inverse for two dimensions
static const double CHI2_CONFIDENCE_NN = 13.81551055796433; //for 99.9% confidence
//static const double CHI2_CONFIDENCE_NN = 9.21034037197618;  //for 99% confidence

int associate_beacons(int* associations, const SparseMatrix& beacX, const SparseMatrix& beacP, const SparseMatrix& obsX, const SparseMatrix& obsP);

#endif
