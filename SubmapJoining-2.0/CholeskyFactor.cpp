#include "CholeskyFactor.h"

CholeskyFactor::CholeskyFactor(int size){
	cholmod_start (&c);
	A = cholmod_allocate_factor(size, &c);
}


CholeskyFactor::CholeskyFactor(const CholeskyFactor& md){
	cholmod_start (&c);
	A =  cholmod_copy_factor(md.A, &c);
}

CholeskyFactor::~CholeskyFactor(){
	cholmod_free_factor(&A, &c);
	cholmod_finish (&c);
}

CholeskyFactor& CholeskyFactor::operator=(const CholeskyFactor& md){
	cholmod_start (&c);
	A =  cholmod_copy_factor(md.A, &c);
}

int CholeskyFactor::get_cols() const{
	return A->n;
}

int CholeskyFactor::get_rows() const{
	return A->n;
}

double CholeskyFactor::get(int get_row, int get_col) const{
	int *row = (int*)A->i;
	int *col = (int*)A->p;
	double *x = (double*)A->x;
	/*cout << "Cholesky get" << endl;
	for(int i =0; i < 3; ++i){
		cout << "Col: " << col[i] << endl;
	}*/
	int i = col[get_col - 1];
	while(i < col[get_col]){
		if(row[i] == get_row -1){
			return x[i];
		}
		++i;
	}
	
	return 0;
}

void CholeskyFactor::set(int set_row, int set_col, double val){
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


