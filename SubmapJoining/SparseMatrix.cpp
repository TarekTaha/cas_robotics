#include "SparseMatrix.h"

SparseMatrix::SparseMatrix(int rows, int cols){
	this->rows = rows;
	this->cols = cols;
	for(int i = 0; i <= rows; ++i){
		first_in_row[i] = 0;
	}
}

SparseMatrix::~SparseMatrix(){
	//cout << "entering destructing" << endl;
	//print();
	remove_all_elements();
	//cout << "exiting destructing" << endl;
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

SparseMatrix& SparseMatrix::operator=(const SparseMatrix& md){
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

double SparseMatrix::get(int row, int column){
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
void SparseMatrix::set(int row, int column, double value){
	if(!value)
		return;
	 if(row > rows || column > cols)
	    throw MatrixException("Error in SparseMatrix::set(int row, int column, double value): trying to set element outside matrix");
	SparseMatrixElement *row_ptr = first_in_row[row];
	SparseMatrixElement *new_elem = new SparseMatrixElement(row, column, value);

	if(!first_in_row[row]){
		//cout << "first in row" << endl;
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

void SparseMatrix::add(int row, int column, double value){
	if(!value)
		return;
	if(row > rows || column > cols)
		throw MatrixException("Error in SparseMatrix::add(int row, int column, double value): trying to set element outside matrix");
	
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

void SparseMatrix::remove_all_elements(){
	SparseMatrixElement *row_ptr;
	SparseMatrixElement *temp;
	//cout << "entering remove all" << endl;
	for(int i = 1; i <=rows; ++i){
		//cout << "here4" << endl;
		row_ptr = first_in_row[i];
		//cout << "here5" << endl;
		while(row_ptr){
			//cout << "here" << endl;
			temp = row_ptr->next_in_row;
			//cout << "here1" << endl;
			delete row_ptr;
			//cout << "here2" << endl;
			row_ptr = temp;
			//cout << "here3" << endl;
		}
		//cout << "here6" << endl;
		first_in_row[i] = 0;
		//cout << "here7" << endl;
	}
	//cout << "exiting remove all" << endl;
}

void SparseMatrix::print() const{
	//cout << "Print " << cols << endl;

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
				//cout << "Print3 " << row_ptr->col<< " " << last_col << endl;
				for(int j = last_col; j < row_ptr->col - 1; ++j){
					cout << "0 ";
				}
				cout << row_ptr->value << " ";
				if(!row_ptr->next_in_row){
					//cout << "Print2 " << row_ptr->col<< " " << cols << endl;
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

void SparseMatrix::print_coord_format() const{
	SparseMatrixElement *row_ptr;
	for(int i = 1; i <= rows; ++i){
		row_ptr = first_in_row[i];
		while(row_ptr){
			cout << "(" << row_ptr->col << "," << row_ptr->row << ")" << "  " << row_ptr->value << endl;
			row_ptr = row_ptr->next_in_row;
		}
	}
}

SparseMatrix operator+(SparseMatrix m1, const SparseMatrix& m2){
	if(m1.rows != m2.rows)
		throw MatrixException("Error in operator+(SparseMatrix m1, const SparseMatrix& m2): matices must have same dimensions");
	if(m1.cols != m2.cols)
		throw MatrixException("Error in operator+(SparseMatrix m1, const SparseMatrix& m2): matices must have same dimensions");
	  
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

SparseMatrix operator*(const SparseMatrix& m1, const SparseMatrix& m2){
	//cout << "entering operator*(const SparseMatrix& m1, const SparseMatrix& m2)" << endl;
	if(m1.cols != m2.rows)
		throw MatrixException("Error in operator*(const SparseMatrix& m1, const SparseMatrix& m2): wrong dimensions");
	

	SparseMatrix m3(trn(m2));

	SparseMatrixElement *row_ptr1;
	SparseMatrixElement *row_ptr3;
	SparseMatrix result(m1.rows, m2.cols);
	double sum;
	for(int i = 1; i <= m1.rows; ++i){
		for(int j = 1; j <= m3.rows; ++j){
			sum = 0;
			row_ptr1 = m1.first_in_row[i];
			row_ptr3 = m3.first_in_row[j];
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
				result.set(i, j, sum);
			}
		}
	}
	row_ptr1 = 0;
	row_ptr3 = 0;
	//cout << "exiting operator*(const SparseMatrix& m1, const SparseMatrix& m2)" << endl;
	return result;
}

SparseMatrix trn(const SparseMatrix& m){
	SparseMatrix result(m.cols, m.rows);
	SparseMatrixElement *row_ptr;

	for(int i = 1; i <= m.rows; ++i){
		row_ptr = m.first_in_row[i];
		while(row_ptr){
			result.set(row_ptr->col, row_ptr->row, row_ptr->value);
			row_ptr = row_ptr->next_in_row;
		}
	}
	/*row_ptr = m.first_in_row[11];
	row_ptr = row_ptr->next_in_row;
	while(row_ptr){
		result.set(row_ptr->col, row_ptr->row, row_ptr->value);
		row_ptr = row_ptr->next_in_row;
	}*/
	return result;
}

void SparseMatrix::write_to_file(const char* filename){
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
