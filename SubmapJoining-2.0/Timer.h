#ifndef TIMER
#define TIMER

#include <iostream.h>
#include <time.h>


using namespace std;

class Timer{
	public:
		Timer();
		double start_time[100];
		double sum_time[100];
		bool started[100];
		void start(int timer);
		void stop(int timer);
		void print();
};

#endif
