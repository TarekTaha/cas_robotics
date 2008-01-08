CCC = g++
CCCFLAGS = -Wno-deprecated
LDFLAGS = -IMatrixLib
MOBJECTS = ./MatrixLib/SparseMatrix.o ./MatrixLib/Matrix.o ./MatrixLib/SparseMatrixElement.o ./MatrixLib/MatrixLib.o

all: main

main : main.o LocalMap.o LocalMapList.o GlobalMap.o
	$(CCC) $(CCCFLAGS) -o main main.o LocalMap.o LocalMapList.o GlobalMap.o $(MOBJECTS)

main.o : main.cc
	$(CCC) $(CCCFLAGS) -c main.cc $(LDFLAGS)
	
LocalMap.o: LocalMap.h LocalMap.cpp
	$(CCC) $(CCCFLAGS) -c LocalMap.cpp $(LDFLAGS)
	
LocalMapList.o: LocalMapList.h LocalMapList.cpp
	$(CCC) $(CCCFLAGS) -c LocalMapList.cpp $(LDFLAGS)

GlobalMap.o: GlobalMap.h GlobalMap.cpp
	$(CCC) $(CCCFLAGS) -c GlobalMap.cpp $(LDFLAGS)
	rm *.o
clean:
	rm *.o
