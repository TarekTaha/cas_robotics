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

	result.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, result.A, &result.c);
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
	SparseMatrix result;
	result.c.final_ll = true;
	result.c.nmethods = 1 ;
	result.c.method [0].ordering = CHOLMOD_NATURAL ;
	result.c.postorder = false ;
	CholeskyFactor L;
	L.A = cholmod_analyze(m.A, &result.c);
	cholmod_factorize(m.A, L.A, &result.c);
	result.A = cholmod_spsolve(CHOLMOD_LDLt, L.A, eye(m.get_rows()).A, &result.c);
	
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

SparseSymmMatrix sqrt(const SparseSymmMatrix& m){
	SparseSymmMatrix Y(m);
	SparseSymmMatrix Z = eye(m.get_rows());
	SparseSymmMatrix temp_Y;
	for(int i = 0; i < 5; ++i){
		temp_Y = Y;
		//cout << "inv" << endl;
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
