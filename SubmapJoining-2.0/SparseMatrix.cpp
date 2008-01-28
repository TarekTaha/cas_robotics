#include "SparseMatrix.h"

SparseMatrix::SparseMatrix(int rows, int cols, int zmax){
	cholmod_start (&c);
	A = cholmod_allocate_sparse(rows, cols, zmax + 2, true, true, 0, CHOLMOD_REAL, &c);
}

SparseMatrix::~SparseMatrix(){
	cholmod_free_sparse(&A, &c);
	cholmod_finish (&c);
}

SparseMatrix::SparseMatrix(const SparseMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_sparse(md.A, &c);
}

SparseMatrix& SparseMatrix::operator=(const SparseMatrix& md){
	A =  cholmod_copy_sparse(md.A, &c);
	return *this;
}

int SparseMatrix::get_cols() const{
	return A->ncol;
}

int SparseMatrix::get_rows() const{
	return A->nrow;
}

double SparseMatrix::get(int get_row, int get_col) const{
	int *row = (int*)A->i;
	int *col = (int*)A->p;
	double *x = (double*)A->x;
	int i = col[get_col - 1];
	while(i < col[get_col]){
		if(row[i] == get_row -1){
			return x[i];
		}
		++i;
	}
	return 0;
}

void SparseMatrix::set(int set_row, int set_col, double val){
	int *row = (int*)A->i;
	int *col = (int*)A->p;
	double *x = (double*)A->x;
	int i = set_col;
	int temp_col;
	int start;

	while(i < get_cols()){
		++col[i];
		++i;
	}
	++col[i];
	start = col[set_col] - 2;
	
	while(start > col[set_col - 1] - 1 && set_row - 1 < row[start]){
		--start;
	}
	if( set_row - 1 == row[start] && start != col[set_col - 1] - 1){
		x[start] = val;
		while(i >= set_col){
			--col[i];
			--i;
		}
	}
	else{
		for(int j = col[i]; j > start; --j){
			x[j + 1] = x[j];
			row[j + 1] = row[j];
		}
		x[start + 1] = val;
		row[start + 1] = set_row - 1;
	}
}

SparseMatrix trn(const SparseMatrix& m){
	SparseMatrix result;
	result.A = cholmod_transpose(m.A, 1, &result.c);
	return result;
}

void SparseMatrix::print_coord() const{
	cholmod_write_sparse(stdout, A, NULL, NULL, &c);
}

SparseMatrix operator+(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {1, 1};
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseMatrix operator-(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {-1, 1};
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseMatrix operator*(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	result.A = cholmod_ssmult(m1.A, m2.A, 0, true, true, &result.c);
	return result;
}





