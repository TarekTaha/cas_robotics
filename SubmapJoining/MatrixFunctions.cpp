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
