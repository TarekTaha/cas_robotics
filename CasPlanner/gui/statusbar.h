#ifndef STATUSLOGGER_H
#define STATUSLOGGER_H

#include <QObject>
#include <QFile> 
#include <QStatusBar> 
#include <QPushButton>
#include <QTextEdit>

class StatusLogger : public QObject
{
Q_OBJECT
    public:
	StatusLogger(QStatusBar *status);
	~StatusLogger();
    public slots:
	void addStatusMsg(int messageId, int messageType, QString message); 
	void showLog();
    private:
	QStatusBar *statusBar; 
	QPushButton *logButton;
	QTextEdit log; 
};

#endif
