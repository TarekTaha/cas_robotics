#include "SparseSymmMatrix.h"

SparseSymmMatrix::SparseSymmMatrix(int rows, int cols, int zmax){
	cholmod_start (&c);
	A = cholmod_allocate_sparse(rows, cols, zmax + 2, true, true, 1, CHOLMOD_REAL, &c);
}

SparseSymmMatrix::~SparseSymmMatrix(){
	cholmod_free_sparse(&A, &c);
	cholmod_finish (&c);
}

SparseSymmMatrix::SparseSymmMatrix(const SparseSymmMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_sparse(md.A, &c);
}

SparseSymmMatrix& SparseSymmMatrix::operator=(const SparseSymmMatrix& md){
	A =  cholmod_copy_sparse(md.A, &c);
	return *this;
}

double SparseSymmMatrix::get(int get_row, int get_col) const{
	if(A->stype > 0){
		if(get_col < get_row){
			int temp = get_row;
			get_row = get_col;
			get_col = temp;
		}
	}
	else{
		if(get_col > get_row){
			int temp = get_row;
			get_row = get_col;
			get_col = temp;
		}
	}
	return SparseMatrix::get(get_row, get_col);
}

void SparseSymmMatrix::set(int set_row, int set_col, double val){
	if(A->stype > 0){
		if(set_col < set_row){
			int temp = set_row;
			set_row = set_col;
			set_col = temp;
		}
	}
	else{
		if(set_col > set_row){
			int temp = set_row;
			set_row = set_col;
			set_col = temp;
		}
	}
	SparseMatrix::set(set_row, set_col, val);
}

SparseSymmMatrix to_symm(const SparseMatrix& m){
	SparseSymmMatrix result;
	cholmod_start (&result.c);
	result.A =  cholmod_copy_sparse(m.A, &result.c);
	result.A->stype = 1;
	cholmod_sort(result.A, &result.c);
	return result;
}

SparseSymmMatrix operator+(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {1, 1};
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseSymmMatrix operator-(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {-1, 1};
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseSymmMatrix operator*(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	result.A = cholmod_ssmult(m1.A, m2.A, 1, true, true, &result.c);
	return result;
}

SparseSymmMatrix trn(const SparseSymmMatrix& m){
	SparseSymmMatrix result(m);
	return result;
}
