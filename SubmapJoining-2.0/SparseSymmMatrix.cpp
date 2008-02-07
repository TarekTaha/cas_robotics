#include "SparseSymmMatrix.h"

SparseSymmMatrix::SparseSymmMatrix(int rows, int cols, int zmax) : SparseMatrix(rows, cols, zmax){
	//cout << "contruct symm" << endl;
	cholmod_start (&c);
	A->stype = 1;
	//A = cholmod_allocate_sparse(rows, cols, zmax + 2, true, true, 1, CHOLMOD_REAL, &c);
}

SparseSymmMatrix::~SparseSymmMatrix(){
	//cout << "destruc symm" << endl;
	//cholmod_free_sparse(&A, &c);
	//cholmod_finish (&c);
	//cholmod_free_work
	//(
			//&c
	//) ;

}

SparseSymmMatrix::SparseSymmMatrix(const SparseSymmMatrix& md) : SparseMatrix(md){
	//cout << "copy symm" << endl;
	//cholmod_start (&c);
	//A =  cholmod_copy_sparse(md.A, &c);
}

SparseSymmMatrix& SparseSymmMatrix::operator=(const SparseSymmMatrix& md){
	//cout << "tilldela symm" << endl;
	cholmod_free_sparse(&A, &c);
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



SparseSymmMatrix operator+(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {1, 1};
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseSymmMatrix operator-(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {-1, 1};
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseSymmMatrix operator*(const SparseSymmMatrix& m1, const SparseSymmMatrix& m2){
	SparseSymmMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_ssmult(m1.A, m2.A, 1, true, true, &result.c);
	return result;
}

SparseSymmMatrix trn(const SparseSymmMatrix& m){
	SparseSymmMatrix result(m);
	return result;
}

SparseSymmMatrix eye(int size){
	SparseSymmMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A  = cholmod_speye
	(
	    /* ---- input ---- */
	    size,        /* # of rows of A */
	    size,        /* # of columns of A */
	    CHOLMOD_REAL,          /* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
	    /* --------------- */
	    &result.c
	) ;
	return result;
}

SparseSymmMatrix operator*(double fact, const SparseSymmMatrix& m){
	SparseSymmMatrix result(m);
	double *x = (double*)result.A->x;
	for(int i = 0; i < result.A->nzmax; ++i){
		x[i] = 	fact * x[i];
	}
	return result;
}
