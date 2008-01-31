#include "SparseMatrix.h"
SparseMatrix::SparseMatrix(){
	cholmod_start (&c);
	A = cholmod_allocate_sparse(0, 0, 2, true, true, 0, CHOLMOD_REAL, &c);
}

SparseMatrix::SparseMatrix(int rows, int cols, int zmax){
	cholmod_start (&c);
	A = cholmod_allocate_sparse(rows, cols, zmax + 2, true, true, 0, CHOLMOD_REAL, &c);
}

SparseMatrix::~SparseMatrix(){
	//cout << "before free" << endl;
	cholmod_free_sparse(&A, &c);
	//cout << "after free" << endl;
	cholmod_finish (&c);
}

SparseMatrix::SparseMatrix(const SparseMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_sparse(md.A, &c);
}

SparseMatrix& SparseMatrix::operator=(const SparseMatrix& md){
	//cholmod_free_sparse(&A, &c);
	if(this != &md){
		//cholmod_free_sparse(&A, &c);
		A =  cholmod_copy_sparse(md.A, &c);
	}
	return *this;
}

void SparseMatrix::clear(){
	int rows = get_rows();
	int cols = get_cols();
	int nzmax = max_num_nonzero();
	cholmod_free_sparse(&A, &c);
	A = cholmod_allocate_sparse(rows, cols, nzmax, true, true, 0, CHOLMOD_REAL, &c);
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

void SparseMatrix::read_from_file(const char* filename){
	cholmod_free_sparse(&A, &c);
	FILE *f = fopen(filename, "r");
	A = cholmod_read_sparse(f, &c);
}

void SparseMatrix::read_from_delimited_file(const char* filename){
	cholmod_free_sparse(&A, &c);
	int row_count = 1, col_count = 1;
	double read_double;
	int rows, cols;
	char ch;
	ifstream file(filename);
	file >> rows;
	file >> cols;
	A = cholmod_allocate_sparse(rows, cols, rows * cols + 2, true, true, 0, CHOLMOD_REAL, &c);
	
	while(!file.eof() && file.peek() != -1){
		file >> read_double;
		set(row_count,col_count,read_double);
		file.get(ch);
		++col_count;
		if(ch == '\n'){
			++row_count;
			col_count = 1;
		}
	}
	rows = row_count;
	file.close();
}

SparseMatrix SparseMatrix::get_submatrix(int* rset, int rsize, int* cset, int csize) const{
	SparseMatrix result;

	result.A = cholmod_submatrix(A, rset, rsize, cset, csize, true, true, &c);
	return result;
}

SparseMatrix SparseMatrix::get_submatrix(int row_start, int col_start, int row_end, int col_end) const{
	int rsize = row_end - row_start + 1;
	int csize = col_end - col_start + 1;
	//cout << col_end << " " << col_start << endl;
	int rset[rsize];
	int cset[csize];
	for(int i = 0; i < rsize; ++i){
		//cout << "i: " << i << endl;
		rset[i] = row_start + i - 1;
		//cout << rset[i] << endl;
	}
	//cout << "ha" << endl;
	for(int i = 0; i < csize; ++i){
		//cout << "i: " << i << " " << csize<< endl;
		cset[i] = col_start + i - 1;
		//cout << cset[i] << endl;
	}
	return get_submatrix(rset, rsize, cset, csize);
}

SparseMatrix operator*(double fact, const SparseMatrix& m){
	SparseMatrix result(m);
	double *x = (double*)result.A->x;
	for(int i = 0; i < result.A->nzmax; ++i){
		x[i] = 	fact * x[i];
	}
	return result;
}

int SparseMatrix::max_num_nonzero() const{
	return A->nzmax;
}

SparseMatrix vertcat(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	result.A = cholmod_vertcat(m1.A, m2.A, true, &result.c);
	return result;
}

SparseMatrix horzcat(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	result.A = cholmod_horzcat(m1.A, m2.A, true, &result.c);
	return result;
}

SparseMatrix zeros(int rows, int cols){
	SparseMatrix result;
	result.A = cholmod_spzeros
	(
	    /* ---- input ---- */
	    rows,        /* # of rows of A */
	    cols,        /* # of columns of A */
	    0,       /* max # of nonzeros of A */
	    CHOLMOD_REAL,          /* CHOLMOD_PATTERN, _REAL, _COMPLEX, or _ZOMPLEX */
	    /* --------------- */
	    &result.c
	) ;
	return result;
}