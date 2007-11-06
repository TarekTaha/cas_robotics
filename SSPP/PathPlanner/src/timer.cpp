#include "timer.h"
//! Constructor
Timer::Timer()
{
	gettimeofday(&start_time,NULL);
}
//! returns the useconds elapsed since last call.
double  Timer::usecElapsed()
{
	gettimeofday(&end_time,NULL);
	time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) -  
	            ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
	return time_diff;
}
//! returns the milli seconds elapsed since last call.
double Timer::msecElapsed()
{
	return (usecElapsed()/1000.0f);
}
//! returns the seconds elapsed since last call.
double Timer::secElapsed()
{
	return (usecElapsed()/double(1e6));;
}
//! Class Destructor.
Timer::~Timer()
{
}
//! Restarts the timer.
void Timer::restart()
{
	gettimeofday(&start_time,NULL);
}
//! Synchronizes the loop with a certain frequency.
void Timer::msecSynch(double period)
{
	double time_elapsed = this->msecElapsed();
	if( time_elapsed < period*1e3)
		usleep((int)(period*1e3 -time_elapsed)); 
}
