CCC = g++
CCCFLAGS = -Wno-deprecated
LDFLAGS = -IMatrixLib
#MOBJECTS = ./MatrixLib/SparseMatrix.o ./MatrixLib/Matrix.o ./MatrixLib/SparseMatrixElement.o ./MatrixLib/MatrixLib.o
INCLUDE =

all: main

st : sparse_test.o SparseMatrix.o SparseSymmMatrix.o SparseMatrixElement.o Matrix.o MatrixFunctions.o MatrixException.o Timer.o
	$(CCC) $(CCCFLAGS) sparse_test.o SparseMatrix.o SparseSymmMatrix.o SparseMatrixElement.o Matrix.o MatrixFunctions.o MatrixException.o Timer.o -o main

sparse_test.o : sparse_test.cpp
	$(CCC) $(CCCFLAGS) -c sparse_test.cpp $(LDFLAGS)

SparseMatrix.o : SparseMatrix.cpp SparseMatrix.h
	$(CCC) $(CCCFLAGS) -c SparseMatrix.cpp $(LDFLAGS)
	
SparseSymmMatrix.o : SparseSymmMatrix.cpp SparseSymmMatrix.h
	$(CCC) $(CCCFLAGS) -c SparseSymmMatrix.cpp $(LDFLAGS)
	
SparseMatrixElement.o : SparseMatrixElement.cpp SparseMatrixElement.h
	$(CCC) $(CCCFLAGS) -c SparseMatrixElement.cpp $(LDFLAGS)

Matrix.o : Matrix.cpp Matrix.h
	$(CCC) $(CCCFLAGS) -c Matrix.cpp $(LDFLAGS)
	
MatrixException.o : MatrixException.h MatrixException.cpp
	$(CCC) $(CCCFLAGS) -c MatrixException.cpp $(LDFLAGS)

MatrixFunctions.o : MatrixFunctions.cpp MatrixFunctions.h
	$(CCC) $(CCCFLAGS) -c MatrixFunctions.cpp $(LDFLAGS)



main : main.o LocalMap.o LocalMapList.o GlobalMap.o SparseMatrix.o SparseSymmMatrix.o SparseMatrixElement.o Matrix.o MatrixFunctions.o MatrixException.o MapFuser.o Timer.o
	$(CCC) $(CCCFLAGS) -o main main.o LocalMap.o LocalMapList.o GlobalMap.o SparseMatrix.o SparseSymmMatrix.o SparseMatrixElement.o Matrix.o MatrixFunctions.o MatrixException.o MapFuser.o Timer.o

main.o : main.cc
	$(CCC) $(CCCFLAGS) -c main.cc $(LDFLAGS)
	
LocalMap.o: LocalMap.h LocalMap.cpp
	$(CCC) $(CCCFLAGS) -c LocalMap.cpp $(LDFLAGS)
	
LocalMapList.o: LocalMapList.h LocalMapList.cpp
	$(CCC) $(CCCFLAGS) -c LocalMapList.cpp $(LDFLAGS)

GlobalMap.o: GlobalMap.h GlobalMap.cpp
	$(CCC) $(CCCFLAGS) -c GlobalMap.cpp $(LDFLAGS)
	
MapFuser.o: MapFuser.h MapFuser.cpp
	$(CCC) $(CCCFLAGS) -c MapFuser.cpp $(LDFLAGS)

Timer.o: Timer.h Timer.cpp
	$(CCC) $(CCCFLAGS) -c Timer.cpp $(LDFLAGS)

clean:
	rm *.o
