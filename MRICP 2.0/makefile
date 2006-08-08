lib = $(PLAYERPATH)/lib/libplayerclient.a
all: MrIcpDriver

#Following options are optimal on CASTER -- when compiling for optimisations, enable these. 
#CFLAGS = -O3 
#CXXFLAGS = -O3 

clean:
	-rm -f *.o *.so *.*~
MrIcpDriver: 
	echo $(lib)
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags gtk+-2.0 playercore playerc++` -c src/*.cpp -Iinclude -I/usr/local/include -I/usr/local/include/player 
	g++ -shared $(CXXFLAGS) -o MrIcpDriver.so *.o
	rm *.o
