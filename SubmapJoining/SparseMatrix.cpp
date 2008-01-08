#include "SparseMatrix.h"

SparseMatrix::SparseMatrix(int rows, int cols){
	this->rows = rows;
	this->cols = cols;
}

SparseMatrix::SparseMatrix(const SparseMatrix& md){
	rows = md.rows;
	cols = md.cols;
	SparseMatrixElement *row_ptr;
	SparseMatrixElement *temp_ptr;
	for(int i = 1; i <= md.rows; ++i){
		row_ptr = md.first_in_row[i];
		if(row_ptr){
			first_in_row[i] = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			temp_ptr =first_in_row[i];
			row_ptr = row_ptr->next_in_row;
		}
		while(row_ptr){
			temp_ptr->next_in_row = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
}

SparseMatrix& SparseMatrix::operator=(const SparseMatrix& md){
	rows = md.rows;
	cols = md.cols;
	SparseMatrixElement *row_ptr;
	SparseMatrixElement *temp_ptr;
	for(int i = 1; i <= md.rows; ++i){
		row_ptr = md.first_in_row[i];
		if(row_ptr){
			first_in_row[i] = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			temp_ptr =first_in_row[i];
			row_ptr = row_ptr->next_in_row;
		}
		while(row_ptr){
			temp_ptr->next_in_row = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
	return *this;
}

void SparseMatrix::set(int row, int column, double value){
	SparseMatrixElement *row_ptr = first_in_row[row];
	SparseMatrixElement *new_elem = new SparseMatrixElement(row, column, value);
	if(!first_in_row[row]){
		first_in_row[row] = new_elem;
	}
	else if(!first_in_row[row]->col > column){
		new_elem->next_in_row = first_in_row[row];
		first_in_row[row] = new_elem;
	}
	else{
		while(row_ptr->next_in_row){
			if(row_ptr->next_in_row->col > column){
				new_elem->next_in_row = row_ptr->next_in_row;
				row_ptr->next_in_row = new_elem;
				break;
			}
			row_ptr = row_ptr->next_in_row;
		}
		if(!row_ptr->next_in_row){
			row_ptr->next_in_row = new_elem;
		}
	}
}

void SparseMatrix::print(){
	SparseMatrixElement *row_ptr;
	int last_col;
	for(int i = 1; i <= rows; ++i){
		last_col = 0;
		row_ptr = first_in_row[i];
		if(!first_in_row[i]){
			for(int j = 1; j <= cols; ++j)
				cout << "0 ";
		}
		else{
			while(row_ptr){
				for(int j = last_col; j < row_ptr->col - 1; ++j){
					cout << "0 ";
				}
				cout << row_ptr->value << " ";
				if(!row_ptr->next_in_row){
					for(int j = row_ptr->col; j < cols; ++j){
						cout << "0 ";
					}
				}
				last_col = row_ptr->col;
				row_ptr = row_ptr->next_in_row;
			}
		}
		cout << endl;
	}
}
