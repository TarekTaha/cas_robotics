#include "DenseMatrix.h"

DenseMatrix::DenseMatrix(int rows, int cols){
	cholmod_start (&c);
	if(rows > cols)
		A = cholmod_allocate_dense(rows, cols, rows, CHOLMOD_REAL, &c);
	else
		A = cholmod_allocate_dense(rows, cols, cols, CHOLMOD_REAL, &c);
}


DenseMatrix::DenseMatrix(const DenseMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_dense(md.A, &c);
}

DenseMatrix::~DenseMatrix(){
	cholmod_free_dense(&A, &c);
	cholmod_finish (&c);
}

DenseMatrix& DenseMatrix::operator=(const DenseMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_dense(md.A, &c);
}

void DenseMatrix::set(int set_row, int set_col, double value){
	double *x = (double*)A->x;
	x[(set_row - 1) * get_rows() + (set_col - 1)] = value;
}

int DenseMatrix::get_cols() const{
	return A->ncol;
}
int DenseMatrix::get_rows() const{
	return A->nrow;
}
double DenseMatrix::get(int get_row, int get_col) const{
	double *x = (double*)A->x;
	return x[(get_row - 1) * get_rows() + (get_col - 1)];
}

void DenseMatrix::print_coord(){
	cholmod_write_dense(stdout, A, NULL, &c);
}

