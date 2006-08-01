lib = $(PLAYERPATH)/lib/libplayerclient.a
all: WheelChairDriver

#Following options are optimal on WheenChair -- when compiling for optimisations, enable these. 
#CFLAGS = -O3 
#CXXFLAGS = -O3 

clean:
	-rm -f *.o *.so *.*~
WheelChairDriver: 
	echo $(lib)
	g++ -Wall -fpic -g3 $(CXXFLAGS) `pkg-config --cflags playercore playerc++` -c *.cpp
	g++ -shared $(CXXFLAGS) -o WheelChairDriver.so *.o
	rm *.o
