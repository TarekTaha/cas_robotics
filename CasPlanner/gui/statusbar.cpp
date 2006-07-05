#include "statusbar.h"

StatusLogger::StatusLogger(QStatusBar *in_statusBar)
{
    statusBar = in_statusBar;
    logButton = new QPushButton("Log");
    connect(logButton, SIGNAL(clicked()),this, SLOT(showLog())); 
    statusBar->addPermanentWidget(logButton);
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
    if(messageType ==0)
    {
		log.setTextColor(Qt::black); 
    }
    if(messageType ==1)
    {
		log.setTextColor(Qt::green); 
    }
    if(messageType ==2)
    {
		log.setTextColor(Qt::black); 
    }
    log.append(message); 
    statusBar->showMessage(message); 
}
