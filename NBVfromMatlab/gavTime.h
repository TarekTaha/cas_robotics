/* This Header file hold the time functionality
 * */
#ifndef GAVTIME_H_
#define GAVTIME_H_

#endif /*GAVTIME_H_*/

class gavTimer
{
	public:
		gavTimer();
		
	//start timer
	gavTimer(int nothing)
	{
		gettimeofday(&start_time,NULL);
	};
			
	void Print()
	{
		gettimeofday(&end_time,NULL);
		time_diff = ((double)end_time.tv_sec*1e6   + (double)end_time.tv_usec) - ((double)start_time.tv_sec*1e6 + (double)start_time.tv_usec);
		cout<<"Current time taken = "<<time_diff<<" usecs"<<endl;
		fflush(stdout);		
	};

	~gavTimer(){};
					
	private:	
		double time_diff;
		struct timeval start_time,end_time;
	
};

