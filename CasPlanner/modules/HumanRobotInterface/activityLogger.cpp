#include "activityLogger.h"

ActivityLogger::ActivityLogger()
{
	logFile.open ("tasks.list", ios::out | ios::app | ios::binary);
	if (logFile.is_open())
	{
		printf("\nLogger File Opened");
	}
	else
		printf("\nActivity Logger: File can not be Opened");
}

ActivityLogger::~ActivityLogger()
{
	logFile.close();
}

void ActivityLogger::addState(int state, int observation)
{
	switch(observation)
	{
		case 0:
			logFile << "%d"<<state<<" Up ";
			break;
		case 1:
			logFile << "%d"<<state<<" Down ";
			break;
		case 2:
			logFile << "%d"<<state<<" Right ";
			break;
		case 3:
			logFile << "%d"<<state<<" Left ";
			break;
		case 4:
			logFile << "%d"<<state<<" Nothing ";
			break;
		default:
			perror("ActivityLogger: Unknown Observation!!!");
	}
}

void ActivityLogger::endCurrentTask()
{
	logFile << "\n";
}

void ActivityLogger::startNewTask()
{
	logFile << "\n";
}
