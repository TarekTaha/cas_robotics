lib = $(PLAYERPATH)/lib/libplayerclient.a
all: libPathPlanner
test:testBuild
install:libInstall

clean:
	rm -f *.o *.so *.*~
libPathPlanner: 
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags gtk+-2.0 ` -c PathPlanner/src/*.cpp -IPathPlanner/include -I/usr/local/include
	g++ -shared $(CXXFLAGS) -o libPathPlanner.so *.o
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags gtk+-2.0 playercore playerc++` -c SSPP_driver.cpp -IPathPlanner/include -I/usr/local/include -I/usr/local/include/player -lPathPlanner
	g++ -shared $(CXXFLAGS) -o SSPP.so *.o
	rm *.o
