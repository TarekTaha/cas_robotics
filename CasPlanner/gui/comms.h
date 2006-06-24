#ifndef COMMS_H
#define COMMS_H
#include <QObject>
#include <QString> 
#include <QStringList>
#include "configfile.h"
class Comms: public QObject 
{
    Q_OBJECT 
    public:
        virtual int config(ConfigFile *cf, int sectionid)=0;
        virtual int start()=0;
        virtual int stop()=0; 
		virtual QString getName()
		{
		    return name; 
		}
    signals:
        void newData(); 
		void statusMsg(int,int,QString); 
    public slots:
        virtual void emergencyStop()=0; 
        virtual void emergencyRelease()=0; 
    protected:
        QString name;
};

#endif 

