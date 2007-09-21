all: libPathPlanner

#CFLAGS = -O3 
#CXXFLAGS = -O3 

clean:
	-rm -f *.o *.so *.*~
libPathPlanner: 
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags gtk+-2.0 ` -c src/*.cpp -Iinclude -I/usr/local/include
	g++ -shared $(CXXFLAGS) -o libPathPlanner.so *.o
#	rm *.o
#test:
	g++ -Iinclude -I/usr/local/include  `pkg-config --libs --cflags gtk+-2.0 ` test/plannertest.cpp -o pathPlanner -lPathPlanner
	rm *.o
