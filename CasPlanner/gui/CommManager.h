#ifndef COMMMANAGER_H
#define COMMMANAGER_H

class CommManager; 
class PlayerInterface;
class Comms;
#include "comms.h"
#include "interfaceprovider.h"
#include "playerinterface.h"

class CommManager: public Comms, public MapProvider, public LaserProvider, public SpeedProvider ,public LocationProvider
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
            virtual Pose   getLocation();
            virtual void   setSpeed(double speed);
            virtual void   setPtz(double pan, double tilt);
            virtual void   setTurnRate(double turnRate); 
		    virtual Map    provideMap(); 
		    virtual void   provideSpeed(double &speed, double &turnRate);
		    virtual void   provideLocation(Pose & location);
        public slots: 
            virtual void setSpeed(double speed, double turnRate); 
            virtual void setLocation(Pose location);
            virtual void emergencyStop();
            virtual void emergencyRelease();
        signals:
		    void dataUpdated();
		    void statusMsg(int,int, QString); 
        protected:
            // Player stuff 
            PlayerInterface *player;
};
#endif

