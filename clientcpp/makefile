all: Control 

clean:
	rm -f *.o *.so *.*~
Control: 
	g++ `sdl-config --cflags --libs` -o control `pkg-config --cflags playerc++` *.cpp `pkg-config --libs playerc++`
