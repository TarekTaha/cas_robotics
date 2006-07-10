#include "statusbar.h"

StatusLogger::StatusLogger(QStatusBar *in_statusBar)
{
    statusBar = in_statusBar;
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
		case 0:
			log.setTextColor(Qt::black); 
			break;
		case 1:
			log.setTextColor(Qt::red); 		
			break;
		case 2:
			log.setTextColor(Qt::green); 		
			break;
		case 3:
			log.setTextColor(Qt::yellow); 		
			break;
		default:
			log.setTextColor(Qt::black); 
	}
    log.append(message); 
    statusBar->showMessage(message); 
}
