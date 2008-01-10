

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
    Matrix(int rows = 1, int columns = 1);
    ~Matrix();
    Matrix(const Matrix&);
    
    Matrix& operator=(const Matrix&);
    Matrix& operator+=(const Matrix&);
    Matrix& operator-=(const Matrix&);

    void write_to_file(const char* filename);
    void read_from_file(const char* filename);
    
    void set(int row, int column, double value);
    void set(int row, int column, const Matrix&);
    double get(int row, int column) const;
    void print() const;
    void add_rows(int num);
    void add_columns(int num);
    void delete_rows(int from_row, int to_row);
    void delete_columns(int from_column, int to_column);
    Matrix get_sub_matrix(int from_row, int from_column, int to_row, int to_column);

    friend Matrix operator+(Matrix, const Matrix&);
    friend Matrix operator-(Matrix, const Matrix&);
    friend Matrix trn(const Matrix&);
    friend Matrix operator*(const Matrix&, const Matrix&);
    friend Matrix operator*(double, const Matrix&);
    friend Matrix operator*(const Matrix&, double);
    friend Matrix inv(const Matrix& m1);
    friend double det(const Matrix& m1);
    friend bool operator==(const Matrix&, const Matrix&);

    int get_rows() const;
    int get_columns() const;

  //protected:
    double values[50][50];
    int rows;
    int columns;
    void row_op_factor(int row1, int row2, double factor);
    void row_op_swap(int row1, int row2);
    void row_op_divide(int row, double factor);
};

Matrix unit_matrix(int);
double unit_m(int);

#endif
