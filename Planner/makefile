lib = $(PLAYERPATH)/libplayerclient.a
all: wheelchairplanner

clean:
	-rm -f *.o wheelchairplanner

wheelchairplanner: 
	echo $(lib)
	g++ -Wall -g  -Iincludes `sdl-config --libs --cflags` `pkg-config --cflags --libs gtk+-2.0 gthread-2.0 player`  src/wheelchairplanner.cpp -o wheelchairplanner $(lib)
