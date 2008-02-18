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
	cholmod_free_sparse(&A, &c);
	cholmod_finish (&c);
}

SparseMatrix::SparseMatrix(const SparseMatrix& md){
	cholmod_start (&c);
	A =  cholmod_copy_sparse(md.A, &c);
}

SparseMatrix& SparseMatrix::operator=(const SparseMatrix& md){
	if(this != &md){
		cholmod_free_sparse(&A, &c);
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


	while(start > -1 && start > col[set_col - 1] - 1 && set_row - 1 < row[start]){
		--start;
	}

	if(start >= 0){
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
	cholmod_free_sparse(&result.A, &result.c);
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
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseMatrix operator-(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	double alpha[2] = {1, 1};
	double beta[2] = {-1, 1};
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_add(m1.A, m2.A, alpha, beta, true, true, &result.c);
	return result;
}

SparseMatrix operator*(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
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
		if(read_double){
			set(row_count,col_count,read_double);
		}
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
SparseMatrix SparseMatrix::get_submatrix(const Permutation& p1, const Permutation& p2) const{
	return get_submatrix(p1.p, p1.size, p2.p, p2.size);
}

SparseMatrix SparseMatrix::get_submatrix(int* rset, int rsize, int* cset, int csize) const{
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &c);
	result.A = cholmod_submatrix(A, rset, rsize, cset, csize, true, true, &c);
	return result;
}

SparseMatrix SparseMatrix::get_submatrix(int row_start, int col_start, int row_end, int col_end) const{
	int rsize = row_end - row_start + 1;
	int csize = col_end - col_start + 1;
	int rset[rsize];
	int cset[csize];
	for(int i = 0; i < rsize; ++i){
		rset[i] = row_start + i - 1;
	}
	for(int i = 0; i < csize; ++i){
		cset[i] = col_start + i - 1;
	}
	return get_submatrix(rset, rsize, cset, csize);
}

SparseMatrix operator*(double fact, const SparseMatrix& m){
	SparseMatrix result(m);
	double *x = (double*)result.A->x;
	for(int i = 1; i < result.A->nzmax; ++i){
		x[i] = 	fact * x[i];
	}
	return result;
}

int SparseMatrix::max_num_nonzero() const{
	return A->nzmax;
}

SparseMatrix vertcat(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_vertcat(m1.A, m2.A, true, &result.c);
	return result;
}

SparseMatrix horzcat(const SparseMatrix& m1, const SparseMatrix& m2){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_horzcat(m1.A, m2.A, true, &result.c);
	return result;
}

SparseMatrix zeros(int rows, int cols, int nznew){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_spzeros(rows, cols, nznew, CHOLMOD_REAL, &result.c);
	return result;
}

SparseMatrix ones(int rows, int cols){
	SparseMatrix result(rows, cols, rows*cols);
	for(int i = 1; i <= rows; ++i){
		for(int j = 1; j <= cols; ++j){
			result.set(i,j,1);
		}
	}
	return result;
}

bool operator==(const SparseMatrix& m1, const SparseMatrix& m2){
	if(m1.get_rows() != m2.get_rows())
		return false;
	if(m1.get_cols() != m2.get_cols())
		return false;
	for(int i = 1; i <= m1.get_rows(); ++i){
		for(int j = 1; j <= m1.get_cols(); ++j){
			if(abs(m1.get(i,j) - m2.get(i,j)) > 1e-8)
				return false;
		}
	}
	return true;
}

int SparseMatrix::num_nonzero() const{
	return cholmod_nnz(A, &c);
}

void reallocate(SparseMatrix& m, int nznew){
	cholmod_reallocate_sparse(nznew, m.A, &m.c);
}

SparseMatrix aat(const SparseMatrix& m){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	
	result.A = cholmod_aat
		(
		    /* ---- input ---- */
		    m.A, /* input matrix; C=A*A’ is constructed */
		    NULL,          /* subset of 0:(A->ncol)-1 */
		    0,       /* size of fset */
		    1,           /* >0: numerical, 0: pattern, <0: pattern (no diag),
		                         * -2: pattern only, no diagonal, add 50%+n extra
		                         * space to C */
		    /* --------------- */
		    &result.c
		) ;
	return result;
}
