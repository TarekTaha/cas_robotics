
#include "Matrix.h"

Matrix::Matrix(int rows, int columns){
  if(rows > 50 || columns > 50)
    throw MatrixException("Error in Matrix(int,int): matrices with more than 50 rows or columns are not handeled");

  this->rows = rows,
  this->columns = columns;

  for(int i = 0; i < rows; ++i){
    for(int j = 0; j < columns; ++j){
      values[i][j] = 0;
    }
  }
}

Matrix::Matrix(const Matrix& m){
  rows = m.rows;
  columns = m.columns;

  if(rows > 50 || columns > 50)
    throw MatrixException("Error in Matrix(const Matrix&): matrices with more than 50 rows or columns are not handeled");

  for(int i = 1; i <= m.rows; ++i){
    for(int j = 1; j <= m.columns; ++j){
      set(i, j, m.get(i,j));
    }
  }
}

Matrix& Matrix::operator=(const Matrix& m){
  rows = m.rows;
  columns = m.columns;

  if(rows > 50 || columns > 50)
    throw MatrixException("Error in Matrix(const Matrix&): matrices with more than 50 rows or columns are not handeled");

  for(int i = 1; i <= m.rows; ++i){
    for(int j = 1; j <= m.columns; ++j){
      set(i, j, m.get(i,j));
    }
  }
  return *this;
}

Matrix& Matrix::operator+=(const Matrix& m){
  *this = *this + m;
  return *this;
}

Matrix& Matrix::operator-=(const Matrix& m){
  *this = *this - m;
  return *this;
}

Matrix::~Matrix(){}

void Matrix::set(int row, int column, double value){
  if(row > rows)
    throw MatrixException("Error in set(int,int,double): row dose note exist");
  if(row <= 0)
    throw MatrixException("Error in set(int,int,double): row dose note exist");
  if(column > columns)
    throw MatrixException("Error in set(int,int,double): column dose note exist");
  if(column <= 0)
    throw MatrixException("Error in set(int,int,double): column dose note exist");
  values[row - 1][column - 1] = value;
}

void Matrix::set(int row, int column, const Matrix& m){
  try{
    for(int i = 1; i <= m.rows; ++i){
      for(int j = 1; j <= m.columns; ++j){
	set(i + row - 1,j + column - 1, m.get(i,j));
      }
    }
  }
  catch(MatrixException me){
    throw MatrixException("Error in set(int,int,const Matrix&): column dose note exist");
  }
}

double Matrix::get(int row, int column) const{
  if(row > rows)
    throw MatrixException("Error in get(int,int,double): row dose note exist");
  if(row <= 0)
    throw MatrixException("Error in get(int,int,double): row dose note exist");
  if(column > columns)
    throw MatrixException("Error in get(int,int,double): column dose note exist");
  if(column <= 0)
    throw MatrixException("Error in get(int,int,double): column dose note exist");

  return values[row - 1][ column - 1];
}

int Matrix::get_rows() const{
   return rows;
}

int Matrix::get_columns() const{
  return columns;
}

void Matrix::print() const{
  cout.precision(10);
  for(int i = 0; i < rows; ++i){
    cout << "[";
    for(int j = 0; j < columns; ++j){
      if(j != 0)
        cout << " ";
      if(abs(values[i][j]) < 0.0000005)
	cout << 0.0;
      else
	cout << values[i][j];
    }
    cout << "]" << endl;
  }
}

Matrix operator+(Matrix m1, const Matrix& m2){
  if(m1.rows != m2.rows)
    throw MatrixException("Error in operator+(Matrix, const Matrix&): matices must have same dimensions");
  if(m1.columns != m2.columns)
    throw MatrixException("Error in operator+(Matrix, const Matrix&): matices must have same dimensions");

  for(int i = 0; i < m1.rows; ++i){
    for(int j = 0; j < m1.columns; ++j){
      m1.values[i][j] += m2.values[i][j];
    }
  }
  return m1;
}

Matrix operator-(Matrix m1, const Matrix& m2){
  if(m1.rows != m2.rows){
    throw MatrixException("Error in operator-(Matrix, const Matrix&): matices must have same dimensions");
  }
  if(m1.columns != m2.columns)
    throw MatrixException("Error in operator-(Matrix, const Matrix&): matices must have same dimensions");

  for(int i = 0; i < m1.rows; ++i){
    for(int j = 0; j < m1.columns; ++j){
      m1.values[i][j] -= m2.values[i][j];
    }
  }
  return m1;
}

Matrix operator*(const Matrix& m1, const Matrix& m2){
  if(m1.columns != m2.rows)
    throw MatrixException("Error in operator*(Matrix, const Matrix&): wrong dimensions");

  Matrix m3(m1.rows, m2.columns);
  double sum;
  for(int i = 1; i <= m1.rows; ++i){
    for(int j = 1; j <= m2.columns; ++j){
      sum = 0;
      for(int k = 1; k <= m1.columns ; ++k){
        sum += m1.get(i,k) * m2.get(k,j);
      }
      m3.set(i,j,sum);
    }
  }
  return m3;
}

Matrix operator*(double fact, const Matrix& m1){
  Matrix m2(m1.rows, m1.columns);
  for(int i = 1; i <= m1.rows; ++i){
    for(int j = 1; j <= m1.columns; ++j){
      m2.set(i,j, fact*m1.get(i,j));
    }
  }
  return m2;
}
Matrix operator*(const Matrix& m, double fact){
  return fact * m;
}

Matrix trn(const Matrix& m1){
  Matrix m2(m1.columns, m1.rows);

  for(int i = 1; i <= m1.rows; ++i){
    for(int j = 1; j <= m1.columns; ++j){
      m2.set(j, i, m1.get(i, j));
    }
  }
  return m2;
}

Matrix unit_matrix(int size){
  Matrix m(size, size);
  for(int i = 1; i <= size; ++i){
    m.set(i,i,1);
  }
  return m;
}

double unit_m(int size){
  Matrix m(size, size);
  for(int i = 1; i <= size; ++i){
    m.set(i,i,1);
  }
  return 0.1;
}

void Matrix::row_op_factor(int row1, int row2, double factor){
  if(row1 > rows)
    throw MatrixException("Internal error");
  if(row2 > rows)
    throw MatrixException("Internal error");
  if(row1 < 1)
    throw MatrixException("Internal error");
  if(row2 < 1)
    throw MatrixException("Internal error");
 
  for(int i = 1; i <= columns; ++i){
    set(row2, i, get(row2, i) + factor * get(row1, i));
  }
}

void Matrix::row_op_swap(int row1, int row2){
  if(row1 > rows)
    throw MatrixException("Internal error");
  if(row2 > rows)
    throw MatrixException("Internal error");
  if(row1 < 1)
    throw MatrixException("Internal error");
  if(row2 < 1)
    throw MatrixException("Internal error");

  double temp;
  for(int i = 1; i <= columns; ++i){
    temp = get(row2,i);
    set(row2, i, get(row1, i));
    set(row1,i,temp);
  }
}

void Matrix::row_op_divide(int row, double factor){
  if(row > rows)
    throw MatrixException("Internal error");
  if(row < 1)
    throw MatrixException("Internal error");

  for(int i = 1; i <= columns; ++i){
    set(row,i,get(row,i)/factor);
  }
}

Matrix inv(const Matrix& m1){
  if(m1.rows != m1.columns)
    throw MatrixException("Error in inv(const Matrix&): Matrix must be squared");

  Matrix m2(m1);
  Matrix m3;
  m3 = unit_matrix(m2.rows);

  int k;

  for(int i = 1; i < m2.rows; ++i){
    if(abs(m2.get(i,i)) < 0.0001){
      m2.set(i,i,0);
      k = i + 1;
      while(abs(m2.get(k,i)) < 0.0001){
	++k;
	if(k > m2.rows)
          throw MatrixException("Error in inv(const Matrix&): invers dose not exist");
      }
      m3.row_op_swap(i,k);
      m2.row_op_swap(i,k);
    }
    for(int j = i + 1; j <= m2.rows; ++j){
      m3.row_op_factor(i,j, -m2.get(j,i)/m2.get(i,i));
      m2.row_op_factor(i,j, -m2.get(j,i)/m2.get(i,i));
    }
  }

  for(int i = m2.rows; i >= 1; --i){
    if(abs(m2.get(i,i)) < 0.0001)
      throw MatrixException("Error in inv(const Matrix&): invers dose not exist");

    m3.row_op_divide(i, m2.get(i,i));
    m2.row_op_divide(i, m2.get(i,i));
    for(int j = 1; j < i; ++j){
      m3.row_op_factor(i,j, -m2.get(j,i)/m2.get(i,i));
      m2.row_op_factor(i,j, -m2.get(j,i)/m2.get(i,i));
    }
  }

  return m3;
}

bool operator==(const Matrix& m1, const Matrix& m2){
  if(m1.rows != m2.rows)
    return 0;
  if(m1.columns != m2.columns)
    return 0;

  for(int i = 1; i <= m1.rows; ++i){
    for(int j = 1; j <= m1.columns; ++j){
      if(abs(m1.get(i,j) - m2.get(i,j)) > 0.0001)
	return 0;
    }
  }
  return 1;
}

void Matrix::add_rows(int num){
  rows += num;
  if(rows > 50 || columns > 50)
    throw MatrixException("Error in add_rows(int): matrices with more than 50 rows or columns are not handeled");

}

void Matrix::add_columns(int num){
  columns += num;
  if(rows > 50 || columns > 50)
    throw MatrixException("Error in add_columns(int): matrices with more than 50 rows or columns are not handeled");
}

void Matrix::delete_rows(int from_row, int to_row){
  if(to_row > rows)
    throw MatrixException("Error in delete_rows(int, int) : Row dose not exsist");
  if(from_row < 1)
    throw MatrixException("Error in delete_rows(int, int) : Row dose not exsist");
  if(to_row < from_row)
    throw MatrixException("Error in delete_rows(int, int) : first int must be less or equal to second");
  
  for(int i = from_row; i <= rows - (to_row - from_row + 1); ++i){
    for(int j = 1; j <= columns; ++j){
      set(i,j,get(i + (to_row - from_row + 1), j));
    }
  }
  rows -= (to_row - from_row + 1);
}

void Matrix::delete_columns(int from_column, int to_column){
  if(to_column > columns)
    throw MatrixException("Error in delete_rows(int, int) : Row dose not exsist");
  if(from_column < 1)
    throw MatrixException("Error in delete_rows(int, int) : Row dose not exsist");
  if(to_column < from_column)
    throw MatrixException("Error in delete_rows(int, int) : first int must be less or equal to second");
  
  for(int i = 1; i <= rows; ++i){
    for(int j = from_column; j <= columns - (to_column - from_column + 1); ++j){
      set(i,j,get(i , j + (to_column - from_column + 1)));
    }
  }
  columns -= (to_column - from_column + 1);

}

Matrix Matrix::get_sub_matrix(int from_row, int from_column, int to_row, int to_column){
  if(to_column > columns)
    throw MatrixException("Error in get_sub_matrix(int, int) : Row dose not exsist");
  if(from_column < 1)
    throw MatrixException("Error in get_sub_matrix(int, int) : Row dose not exsist");
  if(to_column < from_column)
    throw MatrixException("Error in get_sub_matrix(int, int) : first two integers must be less or equal to the last two ingters");
  if(to_row > rows)
    throw MatrixException("Error in get_sub_matrix(int, int) : Row dose not exsist");
  if(from_row < 1)
    throw MatrixException("Error in get_sub_matrix(int, int) : Row dose not exsist");
  if(to_row < from_row)
    throw MatrixException("Error in get_sub_matrix(int, int) : first two integers must be less or equal to the last two ingters");

  Matrix m(to_row - from_row + 1, to_column - from_column + 1);
   
  for(int i = 1; i <= m.rows; ++i){
    for(int j = 1; j <= m.columns; ++j){
      m.set(i,j, get(i + from_row - 1, j + from_column -1));
    }
  }

  return m;
}

double det(const Matrix& m){
  if(m.rows != m.columns)
    throw MatrixException("Error in det(const Matrix&): Matrix must be squared");

  if(m.rows == 1)
    return m.get(1,1);

  Matrix temp;
  double sum = 0;
  for(int i = 1; i <= m.rows; ++i){
    int j = 1;
    temp = m;
    temp.delete_rows(i,i);
    temp.delete_columns(j,j);
    if((double)(i+j)/2 -0.1 > (double)((i+j)/2)){
      sum -= m.get(i,j)*det(temp);
    }
    else{
      sum += m.get(i,j)*det(temp);
    }
  }

  return sum;
}

void Matrix::write_to_file(const char* filename){
  ofstream file;
  file.open(filename);
  for(int i = 0; i < rows; ++i){
    for(int j = 0; j < columns; ++j){
      if(j != 0)
        file << ",";
      file << values[i][j];
    }
    file << endl;
  }
  file.close();
}

void Matrix::read_from_file(const char* filename){
	int row_count = 0, col_count = 0;
	double read_double;
	char c;
	ifstream file(filename);

	while(!file.eof() && file.peek() != -1){
		file >> read_double;
		values[row_count][col_count] = read_double;
		file.get(c);
		++col_count;
		if(c == '\n'){
			++row_count;
			columns = col_count;
			col_count = 0;
		}
	}
	rows = row_count;
	file.close();
}
