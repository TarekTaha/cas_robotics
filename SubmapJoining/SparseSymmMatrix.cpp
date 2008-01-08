#include "SparseSymmMatrix.h"

SparseSymmMatrix::SparseSymmMatrix(int rows, int cols){
	this->rows = rows;
	this->cols = cols;
	for(int i = 0; i < rows; ++i){
		first_in_row[i] = 0;
	}
}

SparseSymmMatrix::~SparseSymmMatrix(){
	remove_all_elements();
}

SparseSymmMatrix::SparseSymmMatrix(const SparseSymmMatrix& md){
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
		else{
			first_in_row[i] = 0;
		}
		while(row_ptr){
			temp_ptr->next_in_row = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
			temp_ptr = temp_ptr->next_in_row;
		}
	}
}

SparseSymmMatrix& SparseSymmMatrix::operator=(const SparseSymmMatrix& md){
	remove_all_elements();

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
		else{
			first_in_row[i] = 0;
		}
		while(row_ptr){
			temp_ptr->next_in_row = new SparseMatrixElement(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
			temp_ptr = temp_ptr->next_in_row;
		}
	}
	return *this;
}

void SparseSymmMatrix::set(int row, int column, double value){

	if(row > column){
		swap(row, column);
	}
	SparseMatrixElement *row_ptr = first_in_row[row];
	SparseMatrixElement *new_elem = new SparseMatrixElement(row, column, value);
	if(!first_in_row[row]){
		first_in_row[row] = new_elem;
	}
	else if(first_in_row[row]->col > column){
		new_elem->next_in_row = first_in_row[row];
		first_in_row[row] = new_elem;
	}
	else{
		while(row_ptr->next_in_row){
			if(row_ptr->next_in_row->col > column){
				if(row_ptr->col == column){
					delete new_elem;
					row_ptr->value = value;
				}
				else{
					new_elem->next_in_row = row_ptr->next_in_row;
					row_ptr->next_in_row = new_elem;
				}
				break;
			}
			row_ptr = row_ptr->next_in_row;
		}
		if(!row_ptr->next_in_row){
			if(row_ptr->col == column){
				delete new_elem;
				row_ptr->value = value;
			}
			else{
				row_ptr->next_in_row = new_elem;
			}
		}
	}
}

void SparseSymmMatrix::add(int row, int column, double value){
	SparseMatrixElement *row_ptr = first_in_row[row];
	SparseMatrixElement *new_elem = new SparseMatrixElement(row, column, value);
	if(!first_in_row[row]){
		first_in_row[row] = new_elem;
	}
	else if(first_in_row[row]->col > column){
		new_elem->next_in_row = first_in_row[row];
		first_in_row[row] = new_elem;
	}
	else{
		while(row_ptr->next_in_row){
			if(row_ptr->next_in_row->col > column){
				if(row_ptr->col == column){
					delete new_elem;
					row_ptr->value += value;
				}
				else{
					new_elem->next_in_row = row_ptr->next_in_row;
					row_ptr->next_in_row = new_elem;
				}
				break;
			}
			row_ptr = row_ptr->next_in_row;
		}
		if(!row_ptr->next_in_row){
			if(row_ptr->col == column){
				delete new_elem;
				row_ptr->value += value;
			}
			else{
				row_ptr->next_in_row = new_elem;
			}
		}
	}
}

void SparseSymmMatrix::remove_all_elements(){
	SparseMatrixElement *row_ptr;
	SparseMatrixElement *temp;
	for(int i = 1; i <=rows; ++i){
		row_ptr = first_in_row[i];
		while(row_ptr){
			temp = row_ptr->next_in_row;
			delete row_ptr;
			row_ptr = temp;
		}
		first_in_row[i] = 0;
	}
}

void SparseSymmMatrix::print(){
	cout << "Symmetry matrix" << endl;
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

SparseSymmMatrix operator+(SparseSymmMatrix m1, const SparseSymmMatrix& m2){
	SparseMatrixElement *row_ptr;
	for(int i = 1; i <= m2.rows; ++i){
		row_ptr = m2.first_in_row[i];
		while(row_ptr){
			m1.add(row_ptr->row, row_ptr->col, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
	return m1;
}
