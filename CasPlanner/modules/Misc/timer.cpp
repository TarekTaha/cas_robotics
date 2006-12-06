#include "timer.h"

Timer::Timer()
{
	gettimeofday(&start_time,NULL);
}

double  Timer::usecElapsed() // return in usec
{
	gettimeofday(&end_time,NULL);
	time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) -  
	            ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
	return time_diff;
}

double Timer::msecElapsed() // return in usec
{
	return (usecElapsed()/1000.0f);
}
double Timer::secElapsed()
{
	return (usecElapsed()/double(1e6));;
}

Timer::~Timer()
{
}
void Timer::restart()
{
	gettimeofday(&start_time,NULL);
}
void Timer::msecSynch(double period)
{
	double time_elapsed = this->msecElapsed();
	if( time_elapsed < period*1e3)
		usleep((int)(period*1e3 -time_elapsed)); 
}
