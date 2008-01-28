

#ifndef MATRIX
#define MATRIX

#include <iostream.h>
#include <cmath>
#include <exception>
#include <fstream>
#include "MatrixException.h"

using namespace std;


class Matrix{	
  public:
    virtual double get(int, int) const = 0;
    virtual void set(int, int, double) = 0;
    virtual int get_cols() const = 0;
    virtual int get_rows() const = 0;
    virtual void write_to_file(const char* filename) const;
    virtual void print() const;
};


#endif
