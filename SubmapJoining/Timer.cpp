#include "Timer.h"

Timer::Timer(){
	for(int i = 0; i < 100; ++i){
		start_time[i] = 0;
		sum_time[i] = 0;
		started[i] = false;
	}
}

void Timer::start(int timer){
	if(!started[timer]){
		start_time[ timer] = clock();
		started[timer] = true;
	}
}

void Timer::stop(int timer){
	if(started[timer]){
		sum_time[timer] += clock() - start_time[timer];
		started[timer] = false;
	}
}

void Timer::print(){
	cout << "Printing times" << endl;
	for(int i = 0; i < 100; ++i){
		if(sum_time[i] > 0){
			cout << "Timer " << i << " " << sum_time[i] << endl;
		}
	}
}
