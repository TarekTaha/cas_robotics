#include "MatrixFunctions.h"

CholeskyFactor cholesky(const SparseSymmMatrix& m){
	CholeskyFactor result(m.get_rows());
	result.c.final_ll = true;
	result.c.nmethods = 1 ;
	result.c.method [0].ordering = CHOLMOD_NATURAL ;
	result.c.postorder = false ;
	//m.write_to_file("SavedMatrices/ill1");


	//cholmod_reallocate_factor(2 * m.max_num_nonzero(), result.A, &result.c);
	result.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, result.A, &result.c);
	cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, result.A, &result.c);
	return result;
}

SparseMatrix solve_cholesky(const CholeskyFactor& L, const SparseMatrix& rhs){
	SparseMatrix result;
	//cout << "bla" << endl;
	result.A = cholmod_spsolve(CHOLMOD_LDLt, L.A, rhs.A, &result.c);
	//cout << "bla2" << endl;
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
	//result.c.final_ll = true;
	//result.c.nmethods = 1 ;
	//result.c.method [0].ordering = CHOLMOD_NATURAL ;
	//result.c.postorder = false ;
	CholeskyFactor L;
	L.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, L.A, &result.c);
	result.A = cholmod_spsolve(CHOLMOD_A, L.A, eye(m.get_rows()).A, &result.c);
	
	//CholeskyFactor L = cholesky(m);
	//return to_sparse_symm_matrix(solve_cholesky(L, eye(m.get_rows())));
	return to_sparse_symm_matrix(result);
}

SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m){
	SparseSymmMatrix result;
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 1;
	cholmod_sort(result.A, &result.c);
	return result;
}

SparseMatrix to_sparse_matrix_fast(const SparseSymmMatrix& m){
	SparseMatrix result;
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 0;
	return result;
}

SparseMatrix to_sparse_matrix(const CholeskyFactor& L){
	SparseMatrix result;
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
	//cholmod_free_factor(&result2.A, &result2.c);
	//cout << "start: " <<  result2.A->nzmax << endl;
    //cholmod_free(1, sizeof (int), result2.A->p,  &result2.c) ;
    //cholmod_free(2, sizeof (int), result2.A->next,  &result2.c) ;
    //cholmod_free(2, sizeof (int), result2.A->prev,  &result2.c) ;
	//delete[] result2.A->x;
	//delete[] result2.A->i;
	//delete[] result2.A->p;
	//result2.A->nzmax = result.A->nzmax;
	//result2.A->n = result.A->nrow;
	//result2.A->x = new double[result.A->nzmax];
	//result2.A->i = new int[result.A->nzmax];
	//cout << "Size: " << result.A->nzmax << endl;
	cholmod_change_factor(CHOLMOD_REAL, true, false, true, false, result2.A, &result2.c);
	cholmod_reallocate_factor(result.A->nzmax, result2.A, &result2.c);

	for(int i = 0; i < result.A->nzmax; ++i){
		((double*)result2.A->x)[i] = ((double*)result.A->x)[i];
		((int*)result2.A->i)[i] = ((int*)result.A->i)[i];
	}
	for(int i =0; i < result.A->ncol + 1; ++i){
		((int*)result2.A->p)[i] = ((int*)result.A->p)[i];
	}
	/*for(int i = 0; i < result.A->nzmax; ++i){
		cout << ((int*)result.A->i)[i] << " / " << ((int*)temp_in.A->i)[i] << endl;
	}*/
	//result2.A->x = temp_in.A->x;
	//((int*)(result2.A->i))[9] = 3;
	//result2.A->i = temp_in.A->i;
	//result2.A->p = temp_in.A->p;
	int *nex = ((int*)result2.A->next);
	for(int i = 0; i < result.get_cols(); ++i){
		nex[i] = i + 1;
	}
	nex[result.get_cols()] = -1;
	nex[result.get_cols() + 1] = 0;
	//nex[0] = 1; nex[1] = 2; nex[2] = 3; nex[3] = 4; nex[4] = -1; nex[5] = 0;//{1 , 2, 3, 4, -1, 0};
	int *pre = ((int*)result2.A->prev);
	for(int i = 1; i < result.get_cols() + 1; ++i){
		pre[i] = i  - 1;
	}
	pre[0] = result.get_cols() + 1;
	pre[result.get_cols() + 1] = -1;

	//pre[0] = 5; pre[1] = 0; pre[2] = 1; pre[3] = 2; pre[4] = 3; pre[5] = -1;//{5, 0, 1, 2, 3, -1};
	int *colco = ((int*)result2.A->ColCount);
	for(int i = 0; i < result.get_cols(); ++i){
		colco[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//colco[0] = 4; colco[1] = 3; colco[2] = 1; colco[3] = 1; //{4, 3, 2, 1};
	int *perm = ((int*)result2.A->Perm);
	for(int i = 0; i < result.get_cols(); ++i){
		perm[i] = i;
	}
	//perm[0] = 0; perm[1] = 1; perm[2] = 2; perm[3] = 3; //{0, 1, 2, 3};
	int *nz = ((int*)result2.A->nz);
	for(int i = 0; i < result.get_cols(); ++i){
		nz[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//nz[0] = 4; nz[1] = 3; nz[2] = 1; nz[3] = 1; //{4, 3, 2, 1};
	/*result2.A->next = nex;
	result2.A->prev = pre;
	result2.A->ColCount = colco;
	result2.A->Perm = perm;
	result2.A->nz = nz;*/
	
	result2.A->ordering = 0;
	result2.A->is_ll = 1;
	result2.A->is_super = 0;
	result2.A->is_monotonic = 1;
	result2.A->itype = 0;
	result2.A->xtype = 1;
	result2.A->dtype = 0;
	result2.A->minor= result.get_cols();
	
	//compare(result2, temp_in);
	//result2.print();
	/*cholmod_change_factor
	(
	    CHOLMOD_REAL,
	    true, 
	    false, 
	    true,
	    false,
	    result2.A,
	    &result2.c
	) ;*/
	//cout << " end" << endl;
	return result2;
}

void set_cols(int* ar1, int* ar2, int num_cols, int size){
	int index = 0;
	for(int i = 0; i < num_cols; ++i){
		ar2[i] = index;
		while(ar1[index] == i){
			++index;
		}
	}
	ar2[num_cols] = index;
}

int partition(int* ar1, int* ar2, double* ar3, int top, int bottom)
{
     int x1 = ar1[top];
     int x2 = ar2[top];
     int i = top - 1;
     int j = bottom + 1;
     int temp;
     do
     {
           do     
           {
        	   --j;
           }while ((x1 < ar1[j]) || (x1 == ar1[j] && x2 < ar2[j]));

          do  
         {
        	  ++i;
          } while ((x1 > ar1[i]) || (x1 == ar1[i] && x2 > ar2[i]));

          if (i < j)
         { 
  			swap(ar1[i], ar1[j]);
  			swap(ar2[i], ar2[j]);
  			swap(ar3[i], ar3[j]);
                 /*temp = array[i];    // switch elements at positions i and j
                 array[i] = array[j];
                 array[j] = temp;*/
         }
     }while (i < j);    
     return j;           // returns middle index
}

void quicksort(int* ar1, int* ar2, double* ar3, int top, int bottom)
{
      // top = subscript of beginning of vector being considered
      // bottom = subscript of end of vector being considered
      // this process uses recursion - the process of calling itself
     int middle;
     if (top < bottom)
    {
          middle = partition(ar1, ar2, ar3, top, bottom);
          quicksort(ar1, ar2, ar3, top, middle);   // sort top partition
          quicksort(ar1, ar2, ar3, middle+1, bottom);    // sort bottom partition
     }
     return;
}

void sort(int* ar1, int* ar2, double* ar3, int size){
	int change_index;
	for(int i = 0; i < size; ++i){
		change_index = i;
		for(int j = i; j < size; ++j){
			if(ar1[j] < ar1[change_index]){
				change_index = j;
			}
			else if(ar1[j] == ar1[change_index]){
				if(ar2[j] < ar2[change_index]){
					change_index = j;
				}
			}
		}
		if(change_index != i){
			swap(ar1[i], ar1[change_index]);
			swap(ar2[i], ar2[change_index]);
			swap(ar3[i], ar3[change_index]);
		}
	}
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
			//result.set(i + 1, row[j] + 1, x[j]);
		}
	}
	/*cout << "Cols: ";
	for(int i = 0; i < index; ++i){
		cout << cols[i] << " ";
	}
	cout << endl;
	
	cout << "Rows: ";
	for(int i = 0; i < index; ++i){
		cout << rowR[i] << " ";
	}
	cout << endl;
	
	cout << "Values: ";
	for(int i = 0; i < index; ++i){
		cout << xR[i] << " ";
	}
	cout << endl;*/
	//timer.start(30);
	quicksort(cols, rowR, xR, 0, index - 1);
	//timer.stop(30);
	/*cout << " ----------- " << endl;
	cout << "Cols: ";
	for(int i = 0; i < index; ++i){
		cout << cols[i] << " ";
	}
	cout << endl;
	
	cout << "Rows: ";
	for(int i = 0; i < index; ++i){
		cout << rowR[i] << " ";
	}
	cout << endl;
	cout << "Values: ";
	for(int i = 0; i < index; ++i){
		cout << xR[i] << " ";
	}
	cout << endl;*/
	
	
	set_cols(cols, colR, result.get_cols(), index);
	return result;
}
