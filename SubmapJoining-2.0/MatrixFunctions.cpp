#include "MatrixFunctions.h"

Permutation reorder_AMD(const SparseSymmMatrix& m){
	CholeskyFactor result;
	result.c.nmethods = 0;
	//result.c.method [0].ordering = CHOLMOD_AMD;

	cholmod_free_factor(&result.A, &result.c);
	result.A = cholmod_analyze(m.A, &result.c);
	cout << "Meathod selected: " << result.c.selected << endl;
	
	Permutation ret(m.get_rows(), ((int*)result.A->Perm));
	return ret;
	//return to_sparse_symm_matrix(to_sparse_matrix(m).get_submatrix((int*)result.A->Perm, result.get_rows(), (int*)result.A->Perm, result.get_cols()));
}

CholeskyFactor cholesky(const SparseSymmMatrix& m){
	CholeskyFactor result;
	result.c.final_ll = true;
	result.c.nmethods = 1 ;
	result.c.method [0].ordering = CHOLMOD_NATURAL;
	result.c.postorder = false ;

	cholmod_free_factor(&result.A, &result.c);
	result.A = cholmod_analyze(m.A, &result.c);
	
	cholmod_factorize(m.A, result.A, &result.c);
	cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, result.A, &result.c);
	return result;
}

CholeskyFactor cholesky2(const SparseSymmMatrix& m, Timer& timer){
	CholeskyFactor result;
	result.c.final_ll = true;
	result.c.nmethods = 1 ;
	result.c.method [0].ordering = CHOLMOD_NATURAL ;
	result.c.postorder = false ;

	cholmod_free_factor(&result.A, &result.c);
	timer.start(91);
	result.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, result.A, &result.c);
	timer.stop(91);
	cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, result.A, &result.c);
	return result;
}

SparseMatrix solve_cholesky2(const CholeskyFactor& L, const SparseMatrix& rhs, Timer& timer){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	timer.start(90);
	result.A = cholmod_spsolve(CHOLMOD_A, L.A, rhs.A, &result.c);
	timer.stop(90);
	return result;
}

SparseMatrix solve_cholesky(const CholeskyFactor& L, const SparseMatrix& rhs){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_spsolve(CHOLMOD_A, L.A, rhs.A, &result.c);
	return result;
}

SparseSymmMatrix inv(const SparseSymmMatrix& m){
	if(m.get_rows() == 2){
		double temp = m.get(1,1) * m.get(2,2) - m.get(1,2)*m.get(2,1);
		SparseSymmMatrix tva(2,2,4);
		tva.set(1,1, m.get(2,2));
		tva.set(1, 2, -m.get(1,2));
		tva.set(2, 1, -m.get(2,1));
		tva.set(2, 2, m.get(1,1));
		return 1/temp*tva;
	}
	SparseMatrix result;
	CholeskyFactor L;
	cholmod_free_factor(&L.A, &L.c);
	L.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, L.A, &result.c);
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_spsolve(CHOLMOD_A, L.A, eye(m.get_rows()).A, &result.c);

	return to_sparse_symm_matrix(result);
}

SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m){
	SparseSymmMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 1;
	cholmod_sort(result.A, &result.c);
	return result;
}

SparseMatrix to_sparse_matrix_fast(const SparseSymmMatrix& m){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 0;
	return result;
}

SparseMatrix to_sparse_matrix(const CholeskyFactor& L){
	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A = cholmod_factor_to_sparse(L.A, &L.c);
	return result;
}

SparseSymmMatrix sqrt(const SparseSymmMatrix& m){
	SparseSymmMatrix Y(m);
	SparseSymmMatrix Z = eye(m.get_rows());
	SparseSymmMatrix temp_Y;
	for(int i = 0; i < 20; ++i){
		temp_Y = Y;
		Y = 0.5*(Y + inv(Z));
		Z = 0.5*(Z + inv(temp_Y));
		//cout << "after inv" << endl;
	}
	return Y;
}

double max_eig(const SparseSymmMatrix& m){
	//cout <<"Max eig" << endl;
	return (m.get(1,1) + m.get(2,2))/2 + sqrt(4 * m.get(1,2) * m.get(2,1) + (m.get(1,1) - m.get(2,2)) * (m.get(1,1) - m.get(2,2)))/2;
}

void compare(const CholeskyFactor& L1, const CholeskyFactor& L2){
	cout << "---- compare ---- " << endl;
	if(L1.A->n != L2.A->n){
		cout << "n: " << L1.A->n<< " / " << L2.A->n<< endl;
	}
	if(L1.A->minor != L2.A->minor){
		cout << "minor: " << L1.A->minor<< " / " << L2.A->minor<< endl;
	}
	for(int i = 0; i < L1.get_cols(); ++i){
		if(((int*)L1.A->Perm)[i] != ((int*)L2.A->Perm)[i])
			cout << "Co Perm: " << ((int*)L1.A->Perm)[i] << " / " << ((int*)L2.A->Perm)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols(); ++i){
		((int*)L1.A->ColCount)[i] = ((int*)L2.A->ColCount)[i];
		if(((int*)L1.A->ColCount)[i] != ((int*)L2.A->ColCount)[i]){
			cout << "Co ColCount: " << ((int*)L1.A->ColCount)[i] << " / " << ((int*)L2.A->ColCount)[i] << endl;
			//((int*)L1.A->ColCount)[i] = ((int*)L2.A->ColCount)[i];
		}
	}
	if(L1.A->nzmax != L2.A->nzmax){
		cout << "nzmax: " << L1.A->nzmax<< " / " << L2.A->nzmax<< endl;
	}
	for(int i = 0; i < L1.get_cols(); ++i){
		if(((int*)L1.A->p)[i] != ((int*)L2.A->p)[i])
			cout << "Co p: " << ((int*)L1.A->p)[i] << " / " << ((int*)L2.A->p)[i] << endl;
	}
	for(int i = 0; i < min(L1.A->nzmax, L2.A->nzmax); ++i){
		if(((int*)L1.A->i)[i] != ((int*)L2.A->i)[i])
			cout << "Co i: " << ((int*)L1.A->i)[i] << " / " << ((int*)L2.A->i)[i] << endl;
	}
	for(int i = 0; i < min(L1.A->nzmax, L2.A->nzmax); ++i){
		if( abs(((double*)L1.A->x)[i] - ((double*)L2.A->x)[i]) > 1e-8)
			cout << "Co x: " << ((double*)L1.A->x)[i] << " / " << ((double*)L2.A->x)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols(); ++i){
		if(((int*)L1.A->nz)[i] != ((int*)L2.A->nz)[i])
			cout << "Co nz: " << ((int*)L1.A->nz)[i] << " / " << ((int*)L2.A->nz)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols() + 2; ++i){
		if(((int*)L1.A->next)[i] != ((int*)L2.A->next)[i])
			cout << "Co next: " << ((int*)L1.A->next)[i] << " / " << ((int*)L2.A->next)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols() + 2; ++i){
		if(((int*)L1.A->prev)[i] != ((int*)L2.A->prev)[i])
			cout << "Co prev: " << ((int*)L1.A->prev)[i] << " / " << ((int*)L2.A->prev)[i] << endl;
	}
	if(L1.A->nsuper != L2.A->nsuper){
		cout << "nsuper: " << L1.A->nsuper << " / " << L2.A->nsuper << endl;
	}
	if(L1.A->ssize != L2.A->ssize){
		cout << "ssize: " << L1.A->ssize << " / " << L2.A->ssize << endl;
	}
	if(L1.A->xsize != L2.A->xsize){
		cout << "xsize: " << L1.A->xsize << " / " << L2.A->xsize << endl;
	}
	if(L1.A->maxcsize != L2.A->maxcsize){
		cout << "maxcsize: " << L1.A->maxcsize << " / " << L2.A->maxcsize<< endl;
	}
	if(L1.A->maxesize!= L2.A->maxesize){
		cout << "maxesize: " << L1.A->maxesize<< " / " << L2.A->maxesize<< endl;
	}


	for(int i = 0; i < L1.A->nsuper + 1; ++i){
		if(L1.A->super != 0 || L2.A->super != 0)
		if(((int*)L1.A->super)[i] != ((int*)L2.A->super)[i])
			cout << "super: " << ((int*)L1.A->super)[i] << " / " << ((int*)L2.A->super)[i] << endl;
	}
	for(int i = 0; i < L1.A->nsuper + 1; ++i){
		if(L1.A->pi!= 0 || L2.A->pi!= 0)
		if(((int*)L1.A->pi)[i] != ((int*)L2.A->pi)[i])
			cout << "pi: " << ((int*)L1.A->pi)[i] << " / " << ((int*)L2.A->pi)[i] << endl;
	}
	for(int i = 0; i < L1.A->nsuper + 1; ++i){
		if(L1.A->px!= 0 || L2.A->px!= 0)
		if(((int*)L1.A->px)[i] != ((int*)L2.A->px)[i])
			cout << "px: " << ((int*)L1.A->px)[i] << " / " << ((int*)L2.A->px)[i] << endl;
	}
	for(int i = 0; i < L1.A->ssize+ 1; ++i){
		if(L1.A->s!= 0 || L2.A->s!= 0)
		if(((int*)L1.A->s)[i] != ((int*)L2.A->s)[i])
			cout << "s: " << ((int*)L1.A->s)[i] << " / " << ((int*)L2.A->s)[i] << endl;
	}
	if(L1.A->ordering != L2.A->ordering){
		cout << "Ordering: " << L1.A->ordering << " / " << L2.A->ordering << endl;
	}
	if(L1.A->is_ll != L2.A->is_ll){
		cout << "is_ll: " << L1.A->is_ll << " / " << L2.A->is_ll<< endl;
	}
	if(L1.A->is_super!= L2.A->is_super){
		cout << "is_super: " << L1.A->is_super<< " / " << L2.A->is_super<< endl;
	}
	if(L1.A->is_monotonic!= L2.A->is_monotonic){
		cout << "is_monotonic: " << L1.A->is_monotonic<< " / " << L2.A->is_monotonic<< endl;
	}
	if(L1.A->itype!= L2.A->itype){
		cout << "itype: " << L1.A->itype << " / " << L2.A->itype << endl;
	}
	if(L1.A->xtype != L2.A->xtype){
		cout << "xtype: " << L1.A->xtype << " / " << L2.A->xtype << endl;
	}
	if(L1.A->dtype != L2.A->dtype){
		cout << "dtype: " << L1.A->dtype << " / " << L2.A->dtype << endl;
	}

	
	cout << " --- after compare ---- " << endl;
}

CholeskyFactor append(CholeskyFactor m1, const SparseMatrix& m2, const CholeskyFactor& temp_in){
	SparseMatrix result;
	SparseMatrix temp;
	temp = to_sparse_matrix(m1);
	
	result = horzcat(temp, zeros(temp.get_rows(),m2.get_cols() - temp.get_cols()));
	result = vertcat(result, m2);
	return to_factor(result);
}

CholeskyFactor to_factor(const SparseMatrix& result){
	CholeskyFactor result2(result.A->nrow);
	cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, result2.A, &result2.c);
	cholmod_reallocate_factor(result.A->nzmax, result2.A, &result2.c);

	for(int i = 0; i < result.A->nzmax; ++i){
		((double*)result2.A->x)[i] = ((double*)result.A->x)[i];
		((int*)result2.A->i)[i] = ((int*)result.A->i)[i];
	}
	for(int i =0; i < result.A->ncol + 1; ++i){
		((int*)result2.A->p)[i] = ((int*)result.A->p)[i];
	}

	int *nex = ((int*)result2.A->next);
	for(int i = 0; i < result.get_cols(); ++i){
		nex[i] = i + 1;
	}
	nex[result.get_cols()] = -1;
	nex[result.get_cols() + 1] = 0;
	int *pre = ((int*)result2.A->prev);
	for(int i = 1; i < result.get_cols() + 1; ++i){
		pre[i] = i  - 1;
	}
	pre[0] = result.get_cols() + 1;
	pre[result.get_cols() + 1] = -1;

	int *colco = ((int*)result2.A->ColCount);
	for(int i = 0; i < result.get_cols(); ++i){
		colco[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	
	int *perm = ((int*)result2.A->Perm);
	for(int i = 0; i < result.get_cols(); ++i){
		perm[i] = i;
	}

	int *nz = ((int*)result2.A->nz);
	for(int i = 0; i < result.get_cols(); ++i){
		nz[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	
	result2.A->ordering = 0;
	result2.A->is_ll = 1;
	result2.A->is_super = 0;
	result2.A->is_monotonic = 1;
	result2.A->itype = 0;
	result2.A->xtype = 1;
	result2.A->dtype = 0;
	result2.A->minor= result.get_cols();

	return result2;
}

void set_cols(int* ar1, int* ar2, int num_cols, int size){
	int index = 0;
	ar2[0] = 0;
	for(int i = 0; i < num_cols - 1; ++i){
		while(ar1[index] == i){
			++index;
		}
		ar2[i + 1] = index;
	}
	ar2[num_cols] = size;
}

SparseMatrix to_sparse_matrix(const SparseSymmMatrix& m){

	SparseMatrix result;
	cholmod_free_sparse(&result.A, &result.c);
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 0;
	cholmod_reallocate_sparse(2 * result.max_num_nonzero(), result.A, &result.c);
	int cols[result.max_num_nonzero()];	

	
	int *rowM = (int*)m.A->i;
	int *colM = (int*)m.A->p;
	double *xM = (double*)m.A->x;
	int *rowR = (int*)result.A->i;
	int *colR = (int*)result.A->p;
	double *xR = (double*)result.A->x;
	
	int index = 0;
	for(int i = 0; i < m.get_cols(); ++i){
		for(int j = colM[i]; j < colM[i + 1]; ++j){
			cols[index] = i;
			++index;
		}
	}
	for(int i = 0; i < m.get_cols(); ++i){
		for(int j = colM[i]; j < colM[i + 1]; ++j){
			if(i != rowM[j]){
				xR[index] = xM[j];
				rowR[index] = i;
				cols[index] = rowM[j];
				++index;
			}
		}
	}
	quicksort_di2_cd1(cols, rowR, xR, 0, index - 1);
	set_cols(cols, colR, result.get_cols(), index);
	return result;
}
