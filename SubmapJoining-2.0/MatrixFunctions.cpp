#include "MatrixFunctions.h"
/*
Matrix to_dence_matrix(const SparseMatrix& md){
  Matrix result(md.rows, md.cols);
  SparseMatrixElement *row_ptr;
  for(int i = 1; i <= md.rows; ++i){
	  row_ptr = md.first_in_row[i];
	  while(row_ptr){
		  result.set(row_ptr->row, row_ptr->col, row_ptr->value);
		  row_ptr = row_ptr->next_in_row;
	  }
  }

  return result;
}

Matrix to_dence_matrix(const SparseSymmMatrix& md){
  Matrix result(md.rows, md.cols);
  SparseMatrixElement *row_ptr;
  for(int i = 1; i <= md.rows; ++i){
	  row_ptr = md.first_in_row[i];
	  while(row_ptr){
		  result.set(row_ptr->row, row_ptr->col, row_ptr->value);
		  result.set(row_ptr->col, row_ptr->row, row_ptr->value);
		  row_ptr = row_ptr->next_in_row;
	  }
  }

  return result;
}

SparseMatrix to_sparse_matrix(const Matrix& m){
  SparseMatrix result(m.rows, m.columns);
  for(int i = 0; i < m.rows; ++i){
    for(int j = 0; j < m.columns; ++j){
      if(m.values[i][j])
        result.set(i + 1,j + 1, m.values[i][j]); 
    }
  }
  return result;
}

SparseSymmMatrix to_sparse_symm_matrix(const Matrix& m){

  SparseSymmMatrix result(m.rows, m.columns);
  for(int i = 0; i < m.rows; ++i){
    for(int j = i; j < m.rows; ++j){
      if(m.values[i][j]){
        result.set(i + 1,j + 1, m.values[i][j]);
      }
    }
  }
  return result;
}

SparseSymmMatrix to_sparse_symm_matrix(const SparseMatrix& m){
	SparseSymmMatrix result(m.rows, m.cols);
	SparseMatrixElement *row_ptr;
	for(int i = 1; i <= m.rows; ++i){
		row_ptr = m.first_in_row[i];
		while(row_ptr){
			if(row_ptr->col >= row_ptr->row)
				result.set(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
	return result;
}

SparseSymmMatrix LtimesLtrn(const SparseMatrix& L, int row_start){

	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr3;
	SparseMatrixElement **last_in_row;
	SparseSymmMatrix result(L.rows, L.rows);
	double sum;
	for(int i = row_start; i <= L.rows; ++i){
		last_in_row = &result.first_in_row[i];
		for(int j = i; j <= L.rows; ++j){
			sum = 0;
			row_ptr1 = L.first_in_row[i];
			row_ptr3 = L.first_in_row[j];
			while(row_ptr1 && row_ptr3){
				if(row_ptr1->col > row_ptr3->col){
					row_ptr3 = row_ptr3->next_in_row;
				}
				else if(row_ptr1->col < row_ptr3->col){
					row_ptr1 = row_ptr1->next_in_row;
				}
				else if(row_ptr1->col == row_ptr3->col){
					sum += row_ptr1->value * row_ptr3->value;
					row_ptr1 = row_ptr1->next_in_row;
					row_ptr3 = row_ptr3->next_in_row;
				}
			}
			if(sum){
				*last_in_row = new SparseMatrixElement(i, j, sum);
				last_in_row = &(*last_in_row)->next_in_row;
			}
		}
	}
	return result;
}

void part_cholesky(SparseMatrix& L, SparseSymmMatrix m, int size, Timer& timer){

	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr2;

	if(size + 1 == m.rows){
		L = cholesky(m, timer);
		return;
	}

	timer.start(10);
	double saved_values[1000];
	int row[1000];
	int num_saved;
	for(int i = 1; i <= m.rows; ++i){
		row_ptr1 = L.first_in_row[i];
		if(row_ptr1){
			if(row_ptr1->col >= m.rows - size){
				L.first_in_row[i] = 0;
			}
			else{
				while(row_ptr1->next_in_row){
					//cout << "COL2 " << L.first_in_row[1]->col << endl;
					if(row_ptr1->next_in_row->col >= m.rows - size){
						row_ptr2 = row_ptr1->next_in_row;
						row_ptr1->next_in_row = 0;
						row_ptr1 = row_ptr2;
						break;
					}
					row_ptr1 = row_ptr1->next_in_row;
				}
			}
			if(row_ptr1){
				if(row_ptr1->col >= m.rows - size){
					while(row_ptr1){
						row_ptr2 = row_ptr1->next_in_row;
						delete row_ptr1;
						row_ptr1 = row_ptr2;
				
					}
				}
			}
		}
	}
	timer.stop(10);

	L.rows = m.rows;
	L.cols = m.cols - size - 1;
	//L.print();
	for(int i = 1; i < m.rows - size; ++i){
		m.remove_row(i);
	}
	
	
	//A.write_to_file_coord("temp");

	timer.start(11);
	SparseSymmMatrix A = LtimesLtrn(L, m.rows - size);
	//for(int i = 1; i < m.rows - size; ++i){
		//A.remove_row(i);
	//}
	timer.stop(11);
	timer.start(12);
	m = m - A;
	timer.stop(12);
	//cout << "after" << endl;

	timer.start(13);
	//m.print();
	L.cols = m.cols;
	double lambda;
	//cout << "START at row: " << m.rows - size << endl;
	for(int i = m.rows - size; i <= m.rows; ++i){
		timer.start(14);
		num_saved = 0;
		row_ptr1 = m.first_in_row[i];
		if(m.first_in_row[i]){
			if(m.first_in_row[i]->value <= 0)
				throw MatrixException("Error in cholesky(SparseSymmMatrix m): matrices must be positive definite");
			lambda = sqrt(m.first_in_row[i]->value);
			L.set(i, i, lambda);
			row_ptr1 = row_ptr1->next_in_row;
			while(row_ptr1){
				saved_values[num_saved] = row_ptr1->value/lambda;
				row[num_saved] = row_ptr1->col;
				++num_saved;

				L.set(row_ptr1->col, i, row_ptr1->value/lambda);

				row_ptr1 = row_ptr1->next_in_row;
			}
		}
		timer.stop(14);
		timer.start(15);
		m.remove_row(i);
		SparseSymmMatrix temp(m.rows, m.cols);
		SparseMatrixElement **end_of_row;
		for(int i = 0; i < num_saved; ++i){
			end_of_row = &temp.first_in_row[row[i]];
			for(int j = i; j < num_saved; ++j){
				*end_of_row = new SparseMatrixElement(row[i], row[j], -saved_values[i] * saved_values[j]);
				end_of_row = &(*end_of_row)->next_in_row;
			}
		}
		timer.stop(15);
		timer.start(16);
		m.plus_equals(temp);
		//m = m + temp;
		timer.stop(16);
	}
	timer.stop(13);
}

void part_cholesky_trn(SparseMatrix& L, SparseSymmMatrix m, int size, Timer& timer){

	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr2;
	//SparseMatrixElement *last_added;

	
	for(int i = m.rows - size; i <= m.rows; ++i){
		L.remove_row(i);
	}
	L.rows = m.rows - size - 1;
	L.cols = m.cols;

	
	
	//A.write_to_file_coord("temp");

	SparseSymmMatrix A = to_sparse_symm_matrix(trn(L)*L);

	m = m - A;

	

	L.rows = m.rows;
	double lambda;
	for(int i = m.rows - size; i <= m.rows; ++i){
		row_ptr1 = m.first_in_row[i];
		if(m.first_in_row[i]){
			if(m.first_in_row[i]->value <= 0)
				throw MatrixException("Error in cholesky(SparseSymmMatrix m): matrices must be positive definite");
			lambda = sqrt(m.first_in_row[i]->value);
			L.set(i, i, lambda);
			row_ptr1 = row_ptr1->next_in_row;
			while(row_ptr1){
				L.set(i, row_ptr1->col, row_ptr1->value/lambda);
				row_ptr1 = row_ptr1->next_in_row;
			}
		}
		row_ptr1 = L.first_in_row[i];
		while(row_ptr1){
			row_ptr2 = L.first_in_row[i];
			while(row_ptr2 != row_ptr1){
				m.add(row_ptr2->col, row_ptr1->col, -row_ptr1->value * row_ptr2->value);
				row_ptr2 = row_ptr2->next_in_row;
			}
			m.add(row_ptr1->col, row_ptr2->col, -row_ptr1->value * row_ptr2->value);
			row_ptr1 = row_ptr1->next_in_row;
		}
	}
}

SparseMatrix cholesky(SparseSymmMatrix m, Timer& timer){
	SparseMatrix result(m.rows, m.cols);
	//SparseMatrix result2(m.rows, m.cols);
	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr2;
	//SparseMatrixElement *last_added;
	double saved_values[1000];
	int row[1000];
	int num_saved;
	double lambda;
	for(int i = 1; i <= m.rows; ++i){
		num_saved = 0;
		row_ptr1 = m.first_in_row[i];
		if(m.first_in_row[i]){
			if(m.first_in_row[i]->value <= 0)
				throw MatrixException("Error in cholesky(SparseSymmMatrix m): matrices must be positive definite");
			lambda = sqrt(m.first_in_row[i]->value);
			result.set(i, i, lambda);
			row_ptr1 = row_ptr1->next_in_row;
			while(row_ptr1){
				saved_values[num_saved] = row_ptr1->value/lambda;
				row[num_saved] = row_ptr1->col;
				++num_saved;
				
				result.set(row_ptr1->col, i, row_ptr1->value/lambda);
				row_ptr1 = row_ptr1->next_in_row;
			}
		}
		m.remove_row(i);
		SparseSymmMatrix temp(m.rows, m.cols);
		SparseMatrixElement **end_of_row;
		for(int i = 0; i < num_saved; ++i){
			end_of_row = &temp.first_in_row[row[i]];
			for(int j = i; j < num_saved; ++j){
				*end_of_row = new SparseMatrixElement(row[i], row[j], -saved_values[i] * saved_values[j]);
				end_of_row = &(*end_of_row)->next_in_row;
				//m.add(row[i], row[j], -saved_values[i] * saved_values[j]);
			}
		}
		m = m + temp;
	}
	return result;
}

SparseMatrix cholesky_trn(SparseSymmMatrix m, Timer& timer){
	SparseMatrix result(m.rows, m.cols);
	//SparseMatrix result2(m.rows, m.cols);
	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr2;
	//SparseMatrixElement *last_added;
	double lambda;
	for(int i = 1; i <= m.rows; ++i){
		row_ptr1 = m.first_in_row[i];
		if(m.first_in_row[i]){
			if(m.first_in_row[i]->value <= 0)
				throw MatrixException("Error in cholesky(SparseSymmMatrix m): matrices must be positive definite");
			lambda = sqrt(m.first_in_row[i]->value);
			result.set(i, i, lambda);
			row_ptr1 = row_ptr1->next_in_row;
			while(row_ptr1){
				result.set(i, row_ptr1->col, row_ptr1->value/lambda);
				//result2.set(row_ptr1->col, i, row_ptr1->value/lambda);
				row_ptr1 = row_ptr1->next_in_row;
			}
		}
		row_ptr1 = result.first_in_row[i];
		while(row_ptr1){
			row_ptr2 = result.first_in_row[i];
			while(row_ptr2 != row_ptr1){
				m.add(row_ptr2->col, row_ptr1->col, -row_ptr1->value * row_ptr2->value);
				row_ptr2 = row_ptr2->next_in_row;
			}
			m.add(row_ptr1->col, row_ptr2->col, -row_ptr1->value * row_ptr2->value);
			row_ptr1 = row_ptr1->next_in_row;
		}
	}
	return result;
}

double solve_cholesky_help_func(SparseMatrixElement *row_ptr, SparseMatrix& rhs2){
	if(!row_ptr->next_in_row){
		if(rhs2.first_in_row[row_ptr->col])
			return rhs2.first_in_row[row_ptr->col]->value/row_ptr->value;
		return 0;
	}
	double temp = solve_cholesky_help_func(row_ptr->next_in_row, rhs2);
	rhs2.add(row_ptr->col, 1, -temp*row_ptr->value);
	return temp;
}

SparseMatrix solve_cholesky(const SparseMatrix& L, SparseMatrix rhs, Timer& timer){
	SparseMatrixElement *row_ptr;
	SparseMatrix rhs2(rhs.rows, 1);
	SparseMatrix result(rhs.rows, 1);
	double temp_d;

	for(int i = 1; i <= L.rows; ++i){
		row_ptr = L.first_in_row[i];
		if(rhs.first_in_row[i]){
			temp_d = rhs.first_in_row[i]->value/row_ptr->value;
			rhs2.set(i, 1, temp_d);
			row_ptr = row_ptr->next_in_row;
			while(row_ptr){
				rhs.add(row_ptr->col, 1, -temp_d*row_ptr->value);
				row_ptr = row_ptr->next_in_row;
			}
		}
	}

	SparseMatrix trnL = trn(L);

	for(int i = L.rows; i >= 1; --i){
		row_ptr = trnL.first_in_row[i];
		result.set(i, 1, solve_cholesky_help_func(row_ptr, rhs2));
	}

	return result;
}

double solve_cholesky_help_func2_1(SparseMatrixElement *row_ptr, const SparseMatrix& res, double rhs){
	if(row_ptr->next_in_row){
		if(res.first_in_row[row_ptr->col]){
			return solve_cholesky_help_func2_1(row_ptr->next_in_row, res, rhs - row_ptr->value * res.first_in_row[row_ptr->col]->value);
		}
		return solve_cholesky_help_func2_1(row_ptr->next_in_row, res, rhs);
	}
	return rhs/row_ptr->value;
}

double solve_cholesky_help_func2_2(SparseMatrixElement *row_ptr, SparseMatrix& rhs){
	if(row_ptr->next_in_row){
		double temp = solve_cholesky_help_func2_2(row_ptr->next_in_row, rhs);
		rhs.add(row_ptr->col, 1, -row_ptr->value * temp);
		return temp;
	}
	return rhs.first_in_row[row_ptr->col]->value/row_ptr->value;
}

SparseMatrix solve_cholesky2(const SparseMatrix& L, SparseMatrix rhs, Timer& timer){
	SparseMatrixElement *row_ptr;
	SparseMatrix rhs2(rhs.rows, 1);
	SparseMatrix result(rhs.rows, 1);
	double temp_d;
	for(int i = 1; i <= L.rows; ++i){
		if(rhs.first_in_row[i]){
			rhs2.set(i, 1, solve_cholesky_help_func2_1( L.first_in_row[i], rhs2, rhs.first_in_row[i]->value));
		}
		else{
			rhs2.set(i, 1, solve_cholesky_help_func2_1( L.first_in_row[i], rhs2, 0));
		}
	}

	for(int i = L.rows; i >= 1; --i){
		if(rhs2.first_in_row[i]){
			result.set(i, 1, solve_cholesky_help_func2_2( L.first_in_row[i], rhs2));
		}
		else{
			result.set(i, 1, 0);
		}
	}
	return result;
	//return rhs2;
}

double solve_cholesky_help_func2_trn(SparseMatrixElement *row_ptr, const SparseMatrix& res, double rhs){
	if(row_ptr){
		if(res.first_in_row[row_ptr->col]){
			return solve_cholesky_help_func2_trn(row_ptr->next_in_row, res, rhs) - row_ptr->value * res.first_in_row[row_ptr->col]->value;
		}
		return solve_cholesky_help_func2_trn(row_ptr->next_in_row, res, rhs);
	}
	return rhs;
}

SparseMatrix solve_cholesky2_trn(const SparseMatrix& L, SparseMatrix rhs, Timer& timer){
	SparseMatrixElement *row_ptr;
	SparseMatrix rhs2(rhs.rows, 1);
	SparseMatrix result(rhs.rows, 1);
	double temp_d;
	for(int i = 1; i <= L.rows; ++i){
		row_ptr = L.first_in_row[i];
		if(rhs.first_in_row[i]){
			temp_d = rhs.first_in_row[i]->value/row_ptr->value;
			rhs2.set(i, 1, temp_d);
			row_ptr = row_ptr->next_in_row;
			while(row_ptr){
				rhs.add(row_ptr->col, 1, -temp_d*row_ptr->value);
				row_ptr = row_ptr->next_in_row;
			}
		}
	}

	
	for(int i = L.rows; i >= 1; --i){
		row_ptr = L.first_in_row[i];
		if(rhs2.first_in_row[i]){
			result.set(i, 1, solve_cholesky_help_func2_trn(row_ptr->next_in_row, result, rhs2.first_in_row[i]->value)/row_ptr->value);
		}
		else{
			result.set(i, 1, solve_cholesky_help_func2_trn(row_ptr->next_in_row, result, 0)/row_ptr->value);
		}
	}
	return result;
}

SparseMatrix operator*(const SparseMatrix& spa, const Matrix& denc){
	if(spa.cols != denc.rows)
		throw MatrixException("Error in operator*(const SparseMatrix& spa, const Matrix& denc): wrong dimensions");
	
	SparseMatrixElement *row_ptr;
	SparseMatrix result(spa.rows, denc.columns);
	double sum;
	for(int i = 1; i <= spa.rows; ++i){
		for(int j = 1; j <= denc.columns; ++j){
			sum = 0;
			row_ptr = spa.first_in_row[i];
			while(row_ptr){
				sum += row_ptr->value * denc.get(row_ptr->col,j);
				row_ptr = row_ptr->next_in_row;
			}
			if(sum){
				result.set(i, j, sum);
			}
		}
	}
	return result;
}


Matrix operator+(Matrix denc, const SparseMatrix& spa){
	SparseMatrixElement *row_ptr;
	for(int i = 1; i <= spa.rows; ++i){
		row_ptr = spa.first_in_row[i];
		while(row_ptr){
			denc.values[i - 1][row_ptr->col - 1] += row_ptr->value;
			row_ptr = row_ptr->next_in_row;
		}
	}
	return denc;
}

SparseMatrix extract_sub_matrix(SparseSymmMatrix& m, int from_row, int from_col, int to_row, int to_col){
	SparseMatrix result(to_row - from_row + 1, to_col - from_col + 1);
	for(int i = 1; i <= result.rows; ++i){
		m.extract_sub_segment(from_row - 1 + i, from_col, to_col, &result.first_in_row[i], 0, -from_row + 1, -from_col + 1);
	}
	//result.print();
	return result;
}

void set(SparseSymmMatrix& set_m, int row, int col, const SparseMatrix& m){
	SparseMatrixElement *row_ptr;
	for(int i = 1; i <= m.rows; ++i){
		row_ptr = m.first_in_row[i];
		while(row_ptr){
			set_m.set(i + row - 1, col + row_ptr->col - 1, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
}
*/

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

SparseMatrix to_sparse_matrix(const SparseSymmMatrix& m){
	SparseMatrix result;
	result.A =  cholmod_copy_sparse(m.A, &m.c);
	result.A->stype = 0;
	cholmod_reallocate_sparse(2 * result.max_num_nonzero(), result.A, &result.c);

	
	int *row = (int*)m.A->i;
	int *col = (int*)m.A->p;
	double *x = (double*)m.A->x;
	for(int i = 0; i < m.get_cols(); ++i){
		for(int j = col[i]; j < col[i + 1]; ++j){
			result.set(i + 1, row[j] + 1, x[j]);
		}
	}
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
	for(int i = 0; i < L1.get_cols(); ++i){
		if(((int*)L1.A->nz)[i] != ((int*)L2.A->nz)[i])
			cout << "Co nz: " << ((int*)L1.A->nz)[i] << " / " << ((int*)L2.A->nz)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols() + 2; ++i){
		if(((int*)L1.A->prev)[i] != ((int*)L2.A->prev)[i])
			cout << "Co prev: " << ((int*)L1.A->prev)[i] << " / " << ((int*)L2.A->prev)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols() + 2; ++i){
		if(((int*)L1.A->next)[i] != ((int*)L2.A->next)[i])
			cout << "Co next: " << ((int*)L1.A->next)[i] << " / " << ((int*)L2.A->next)[i] << endl;
	}
	for(int i = 0; i < min(L1.A->nzmax, L2.A->nzmax); ++i){
		if(((int*)L1.A->i)[i] != ((int*)L2.A->i)[i])
			cout << "Co i: " << ((int*)L1.A->i)[i] << " / " << ((int*)L2.A->i)[i] << endl;
	}
	for(int i = 0; i < L1.get_cols(); ++i){
		if(((int*)L1.A->p)[i] != ((int*)L2.A->p)[i])
			cout << "Co p: " << ((int*)L1.A->p)[i] << " / " << ((int*)L2.A->p)[i] << endl;
	}
	for(int i = 0; i < min(L1.A->nzmax, L2.A->nzmax); ++i){
		if( abs(((double*)L1.A->x)[i] - ((double*)L2.A->x)[i]) > 1e-8)
			cout << "Co x: " << ((double*)L1.A->x)[i] << " / " << ((double*)L2.A->x)[i] << endl;
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
	if(L1.A->minor != L2.A->minor){
		cout << "minor: " << L1.A->minor<< " / " << L2.A->minor<< endl;
	}
	
	cout << " --- after compare ---- " << endl;
}

CholeskyFactor append(CholeskyFactor m1, const SparseMatrix& m2, const CholeskyFactor& temp_in){
	SparseMatrix result;
	SparseMatrix temp;
	CholeskyFactor result2(4);

	temp.A = cholmod_factor_to_sparse(m1.A, &m1.c);

	//temp = temp.get_submatrix(1, 1, 2, 2);
	result.A = cholmod_horzcat(temp.A, zeros(temp.get_rows(),m2.get_cols() - temp.get_cols()).A, true, &result.c);
	result.A = cholmod_vertcat(result.A, m2.A, true, &result.c);


	//cholmod_reallocate_sparse(result.get_cols() * result.get_rows(), result.A, &result.c);
	
	/*result2.A->x = result.A->x;
	result2.A->p = result.A->p;
	result2.A->i = result.A->i;*/
	/*for(int i = 1; i <= 4; ++i){
		for(int j = 1; j <= i; ++j){
			result.set(i, j, temp_in.get(i,j));
		}
	}*/

		
	result2.A->nzmax = result.A->nzmax;
	result2.A->n = result.A->nrow;
	
	result2.A->x = new double[result.A->nzmax];
	result2.A->i = new int[result.A->nzmax];
	for(int i =0; i < result.A->nzmax; ++i){
		((double*)result2.A->x)[i] = ((double*)result.A->x)[i];
		((int*)result2.A->i)[i] = ((int*)result.A->i)[i];
	}
	result2.A->p = new int[result.A->ncol + 1];
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
	int *nex = new int[result.get_cols() + 2];
	for(int i = 0; i < result.get_cols(); ++i){
		nex[i] = i + 1;
	}
	nex[result.get_cols()] = -1;
	nex[result.get_cols() + 1] = 0;
	//nex[0] = 1; nex[1] = 2; nex[2] = 3; nex[3] = 4; nex[4] = -1; nex[5] = 0;//{1 , 2, 3, 4, -1, 0};
	int *pre = new int[result.get_cols() + 2];
	for(int i = 1; i < result.get_cols() + 1; ++i){
		pre[i] = i  - 1;
	}
	pre[0] = result.get_cols() + 1;
	pre[result.get_cols() + 1] = -1;

	//pre[0] = 5; pre[1] = 0; pre[2] = 1; pre[3] = 2; pre[4] = 3; pre[5] = -1;//{5, 0, 1, 2, 3, -1};
	int *colco = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		colco[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//colco[0] = 4; colco[1] = 3; colco[2] = 1; colco[3] = 1; //{4, 3, 2, 1};
	int *perm = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		perm[i] = i;
	}
	//perm[0] = 0; perm[1] = 1; perm[2] = 2; perm[3] = 3; //{0, 1, 2, 3};
	int *nz = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		nz[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//nz[0] = 4; nz[1] = 3; nz[2] = 1; nz[3] = 1; //{4, 3, 2, 1};
	result2.A->next = nex;
	result2.A->prev = pre;
	result2.A->ColCount = colco;
	result2.A->Perm = perm;
	result2.A->nz = nz;
	
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

	if(!cholmod_check_factor(result2.A, &result2.c)){
		compare(result2, temp_in);
	}
	//cout << "Valid: " << cholmod_check_factor(result2.A, &result2.c)  << endl;
	return result2;
}

CholeskyFactor to_factor(const SparseMatrix& result){
	CholeskyFactor result2;
	//cholmod_free_factor(&result2.A, &result2.c);
	//cout << "start: " <<  result2.A->nzmax << endl;
    //cholmod_free(1, sizeof (int), result2.A->p,  &result2.c) ;
    //cholmod_free(2, sizeof (int), result2.A->next,  &result2.c) ;
    //cholmod_free(2, sizeof (int), result2.A->prev,  &result2.c) ;
	//delete[] result2.A->x;
	//delete[] result2.A->i;
	//delete[] result2.A->p;
	result2.A->nzmax = result.A->nzmax;
	result2.A->n = result.A->nrow;
	result2.A->x = new double[result.A->nzmax];
	result2.A->i = new int[result.A->nzmax];
	for(int i =0; i < result.A->nzmax; ++i){
		((double*)result2.A->x)[i] = ((double*)result.A->x)[i];
		((int*)result2.A->i)[i] = ((int*)result.A->i)[i];
	}
	result2.A->p = new int[result.A->ncol + 1];
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
	int *nex = new int[result.get_cols() + 2];
	for(int i = 0; i < result.get_cols(); ++i){
		nex[i] = i + 1;
	}
	nex[result.get_cols()] = -1;
	nex[result.get_cols() + 1] = 0;
	//nex[0] = 1; nex[1] = 2; nex[2] = 3; nex[3] = 4; nex[4] = -1; nex[5] = 0;//{1 , 2, 3, 4, -1, 0};
	int *pre = new int[result.get_cols() + 2];
	for(int i = 1; i < result.get_cols() + 1; ++i){
		pre[i] = i  - 1;
	}
	pre[0] = result.get_cols() + 1;
	pre[result.get_cols() + 1] = -1;

	//pre[0] = 5; pre[1] = 0; pre[2] = 1; pre[3] = 2; pre[4] = 3; pre[5] = -1;//{5, 0, 1, 2, 3, -1};
	int *colco = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		colco[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//colco[0] = 4; colco[1] = 3; colco[2] = 1; colco[3] = 1; //{4, 3, 2, 1};
	int *perm = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		perm[i] = i;
	}
	//perm[0] = 0; perm[1] = 1; perm[2] = 2; perm[3] = 3; //{0, 1, 2, 3};
	int *nz = new int[result.get_cols()];
	for(int i = 0; i < result.get_cols(); ++i){
		nz[i] = ((int*)result.A->p)[i + 1] - ((int*)result.A->p)[i];
	}
	//nz[0] = 4; nz[1] = 3; nz[2] = 1; nz[3] = 1; //{4, 3, 2, 1};
	result2.A->next = nex;
	result2.A->prev = pre;
	result2.A->ColCount = colco;
	result2.A->Perm = perm;
	result2.A->nz = nz;
	
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

SparseMatrix to_sparse_matrix2(const SparseSymmMatrix& m){

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
