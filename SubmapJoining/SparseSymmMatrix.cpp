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
	 if(row > rows || column > cols)
	    throw MatrixException("Error in SparseSymmMatrix::set(int row, int column, double value): trying to set element outside matrix");
	 
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
	if(row > rows || column > cols)
		throw MatrixException("Error in SparseSymmMatrix::add(int row, int column, double value): trying to set element outside matrix");
	
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

void SparseSymmMatrix::print() const{
	cout << "Symmetry matrix" << endl;
	cout.precision(20);
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
	  if(m1.rows != m2.rows)
	    throw MatrixException("Error in operator+(SparseSymmMatrix m1, const SparseSymmMatrix& m2): matices must have same dimensions");
	  if(m1.cols != m2.cols)
	    throw MatrixException("Error in operator+(SparseSymmMatrix m1, const SparseSymmMatrix& m2): matices must have same dimensions");
	  
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

double SparseSymmMatrix::get(int row, int column){
	if(row > column){
		swap(row, column);
	}
	 if(row > rows || column > cols)
	    throw MatrixException("Error in SparseMatrix::get(int row, int column, double value): trying to get element outside matrix");
	 
	 SparseMatrixElement *row_ptr;
	 row_ptr = first_in_row[row];
	 while(row_ptr){
		 if(row_ptr->col == column){
			 return row_ptr->value;
		 }
		 if(row_ptr->col > column){
			 return 0;
		 }
		 row_ptr = row_ptr->next_in_row;
	 }
	 return 0;
}

void SparseSymmMatrix::write_to_file(const char* filename){
  ofstream file;
  file.open(filename);
  file.precision(30);
  for(int i = 1; i <= rows; ++i){
    for(int j = 1; j <= cols; ++j){
      if(j != 1)
        file << ",";
      file << get(i,j);
    }
    file << endl;
  }
  file.close();
}

void SparseSymmMatrix::extract_sub_segment(int row, int from_col, int to_col, SparseMatrixElement **start, SparseMatrixElement *end, int row_num_change, int col_num_change){
	//cout << "Extract: " << row << " " << from_col << " " << to_col << " " << col_num_change << endl;
	SparseMatrixElement *row_ptr;
	SparseMatrixElement *last_row_ptr;
	SparseMatrixElement *cut_row_ptr;
	row_ptr = first_in_row[row];
	last_row_ptr = row_ptr;
	if(!row_ptr){
		*start = end;
		return;
	}

	//cout << "Col: " << row_ptr->col <<  " " << row_ptr->value << endl;
	if(row_ptr->col >= from_col){
		if(row_ptr->col > to_col){
			*start = end;
			return;
		}
		*start = row_ptr;
		while(row_ptr){
			if(row_ptr->col > to_col){
				break;
			}
			row_ptr->row += row_num_change;
			row_ptr->col += col_num_change;
			last_row_ptr = row_ptr;
			row_ptr = row_ptr->next_in_row;
		}
		first_in_row[row] = row_ptr;
		last_row_ptr->next_in_row = end;
	}
	else{
		while(row_ptr){
			if(row_ptr->col >= from_col){
				break;
			}
			last_row_ptr = row_ptr;
			row_ptr = row_ptr->next_in_row;
		}
		cut_row_ptr = last_row_ptr;
		if(row_ptr){
			if(row_ptr->col > to_col){
				*start = end;
				return;
			}
			*start = row_ptr;
		}
		while(row_ptr){
			if(row_ptr->col > to_col){
				break;
			}
			row_ptr->row += row_num_change;
			row_ptr->col += col_num_change;
			last_row_ptr = row_ptr;
			row_ptr = row_ptr->next_in_row;
		}
		cut_row_ptr->next_in_row = row_ptr;
		last_row_ptr->next_in_row = end;
	}
}

