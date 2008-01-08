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

SparseMatrix solve_cholesky(const SparseMatrix& L, SparseMatrix rhs){
	SparseMatrixElement *row_ptr;
	SparseMatrix rhs2(rhs.rows, 1);
	double temp_d;
	for(int i = 1; i <= L.rows; ++i){
		row_ptr = L.first_in_row[i];
		if(rhs.first_in_row[rhs.rows + 1 - i]){
			temp_d = rhs.first_in_row[rhs.rows + 1 - i]->value/row_ptr->value;
			rhs2.set(i, 1, temp_d);
			row_ptr = row_ptr->next_in_row;
			while(row_ptr){
				rhs.add(rhs.rows + 1 - row_ptr->col, 1, -temp_d*row_ptr->value);
				row_ptr = row_ptr->next_in_row;
			}
		}
	}
	return rhs2;
}
