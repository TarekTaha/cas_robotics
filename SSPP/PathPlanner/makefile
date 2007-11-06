all: libPathPlanner
test:testBuild
install:libInstall
#CFLAGS = -O3 
#CXXFLAGS = -O3 

clean:
	rm -f *.o *.so *.*~
libPathPlanner: 
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags gtk+-2.0 ` -c src/*.cpp -Iinclude -I/usr/local/include
	g++ -shared $(CXXFLAGS) -o libPathPlanner.so *.o
	rm *.o
testBuild:
	g++ -Iinclude -I/usr/local/include  `pkg-config --libs --cflags gtk+-2.0 ` test/planner.cpp -o pathPlanner -lPathPlanner
	g++ -Iinclude -I/usr/local/include  `pkg-config --libs --cflags gtk+-2.0 ` test/test.cpp -o plannerTest -lPathPlanner	
	rm -rf *.o
libInstall:
	cp -f libPathPlanner.so /usr/local/lib
	mkdir /usr/local/include/pathplanner
	cp include/*.h /usr/local/include/pathplanner