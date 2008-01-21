#include "MatrixFunctions.h"

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


SparseMatrix cholesky(SparseSymmMatrix m){
	SparseMatrix result(m.rows, m.cols);
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

SparseMatrix solve_cholesky(const SparseMatrix& L, SparseMatrix rhs){
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
