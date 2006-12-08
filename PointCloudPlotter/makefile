all: 3dscanner

clean:
	-rm -f *.o *.so *.*~
3dscanner: 
	#echo 
	g++ -c laser_scan.cc
	g++ -c zpr.c
	g++ -c AI.cc
	g++ -c simpleSerialComms.cc
	g++ laser_scan.o AI.o simpleSerialComms.o zpr.o -o laser_scan -lglut `pkg-config --libs --cflags playerc++`	
