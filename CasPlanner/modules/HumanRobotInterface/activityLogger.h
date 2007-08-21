#ifndef ACTIVITYLOGGER_H_
#define ACTIVITYLOGGER_H_

#include <iostream>
#include <fstream>
using namespace std;

class ActivityLogger
{
public:
	ActivityLogger();
	virtual ~ActivityLogger();
	fstream logFile;
	void addState(int state, int Observation);
	void endCurrentTask();
	void startNewTask(); 
};

#endif /*ACTIVITYLOGGER_H_*/
