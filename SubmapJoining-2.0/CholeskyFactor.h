#ifndef CHOLESKYFACTOR
#define CHOLESKYFACTOR

#include <iostream.h>
#include "cholmod.h"
#include <fstream>
#include "Matrix.h"

using namespace std;

class CholeskyFactor : public Matrix{
  public:
    CholeskyFactor(int size = 0);
    CholeskyFactor(const CholeskyFactor&);
    ~CholeskyFactor();
    CholeskyFactor& operator=(const CholeskyFactor& m);
	cholmod_factor *A;
	mutable cholmod_common c;

	virtual int get_cols() const;
	virtual int get_rows() const;
    virtual void set(int, int, double);
    virtual double get(int, int) const;
};

#endif
