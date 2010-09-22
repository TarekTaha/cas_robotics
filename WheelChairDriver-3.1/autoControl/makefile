all: autocontrol

#Following options are optimal on WheenChair -- when compiling for optimisations, enable these. 
#CFLAGS = -O3 
#CXXFLAGS = -O3 

clean:
	-rm -f *.o *.so *.*~
autocontrol: 
	g++ -Wall -fpic -g3 $(CXXFLAGS) `sdl-config --cflags --libs` *.cpp -o auto
	#g++ $(CXXFLAGS) -o auto *.o
	rm -rf *.o
	rm -rf *.*~
	rm -rf *~
