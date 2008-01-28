
#include "Matrix.h"

void Matrix::write_to_file(const char* filename) const{
  ofstream file;
  file.open(filename);
  file.precision(30);
  for(int i = 1; i <= get_rows(); ++i){
    for(int j = 1; j <= get_cols(); ++j){
      if(j != 1)
        file << ",";
      file << get(i,j);
    }
    file << endl;
  }
  file.close();
}

void Matrix::print() const{
	for(int i = 1; i <= get_rows(); ++i){
	    for(int j = 1; j <= get_cols(); ++j){
	    	if(j != 1)
	    		cout << ",";
	    	cout << get(i,j);
	    }
	    cout << endl;
	 }
}
