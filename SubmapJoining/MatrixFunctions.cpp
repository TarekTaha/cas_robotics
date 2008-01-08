#include "MatrixFunctions.h"

Matrix sparse_matrix_to_dence_matrix(const SparseMatrix& md){
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
