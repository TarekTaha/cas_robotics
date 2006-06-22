#ifndef COMMMANAGER_H
#define COMMMANAGER_H

class CommManager; 
class PlayerInterface; 
class Comms;
#include "comms.h"
#include "interfaceprovider.h"
#include "playerinterface.h"

class CommManager: public Comms, public ImgProvider, public LaserProvider, public SpeedProvider 
{
        Q_OBJECT
        public:
            CommManager();
            ~CommManager(); 
            virtual int config(ConfigFile *cf, int sectionid);
            virtual int start(); 
            virtual int stop();
            virtual QVector<QPointF> getLaserScan(int laserId);
            virtual double getSpeed(); 
            virtual double getTurnRate(); 
            virtual void setSpeed(double speed);
            virtual void setTurnRate(double turnRate); 
		    virtual SimpleImage provideImg(CameraId camId); 
		    virtual void provideSpeed(double &speed, double &turnRate);
//		    virtual void setMapManager(QTMapDataInterface *in_mapManager)
//		    {
//				mapManager = in_mapManager;
//		    } 
	    
        public slots: 
            virtual void setSpeed(double speed, double turnRate); 
            virtual void emergencyStop();
            virtual void emergencyRelease();
        signals:
		    void dataUpdated();
//		    void imgUpdate(unsigned int camId); // Dang. Can't use camera id directly.  
//          void ogUpdated(QString ,QString); 
//          void patchCreated(QString, QString, QString);
		    void statusMsg(int,int, QString); 
        protected:
            // Player stuff 
            PlayerInterface *player;
            bool startConnected,activateControl,laserEnabled;
            QString playerIp; 
            int playerPort,positionControlId,laserId; 
//	    	bool mappingEnabled; 
//	   		QTMapDataInterface *mapManager; 
			int tmp;  
};
#endif

