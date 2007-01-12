#include "statusbar.h"

StatusLogger::StatusLogger(QStatusBar *in_statusBar):
statusBar(in_statusBar)    
{
    log.setReadOnly(true); 
}

StatusLogger::StatusLogger():
statusBar(NULL)
{
    log.setReadOnly(true); 
}

void StatusLogger::showLog()
{
    qDebug("Displaying Log Window"); 
    log.show(); 
}
StatusLogger::~StatusLogger()
{
}


void StatusLogger::addStatusMsg(int messageId, int messageType, QString message)
{
	switch (messageType)
	{
		case INFO:
			log.setTextColor(Qt::green); 
			break;
		case ERROR:
			log.setTextColor(Qt::red); 		
			break;
		case WARNING:
			log.setTextColor(Qt::yellow); 		
			break;
		default:
			log.setTextColor(Qt::black); 
	}
    log.append(message); 
    if(statusBar)
    	statusBar->showMessage(message); 
}
