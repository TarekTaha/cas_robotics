
#ifndef MATRIXEXCEPTION
#define MATRIXEXCEPTION

#include <exception>

using namespace std;

class MatrixException: public exception{
  public:  
    MatrixException(char* msg);
    virtual const char* what() const throw(){
       return msg;
    }

  private:
   char* msg;
};

#endif
