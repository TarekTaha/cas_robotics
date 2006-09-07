#ifndef COMMS_H
#define COMMS_H
#include <QObject>
#include <QString> 
#include <QStringList>
#include <QVector>
#include "configfile.h"
class Comms: public QObject 
{
    Q_OBJECT 
    public:
        virtual int readConfigs(ConfigFile *cf,int secId)=0;
        virtual int start()=0;
        virtual int stop()=0;
        bool connected,localized;
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
		bool startConnected,activateControl,ptzEnabled,occMapEnabled,localizerEnabled,laserEnabled
			 ,vfhEnabled;
    	QString name,playerIp; 
        int playerPort,positionControlId,ptzId,mapId,localizerId,vfhId;
};

#endif 

