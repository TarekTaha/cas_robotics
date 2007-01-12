#ifndef COMMMANAGER_H
#define COMMMANAGER_H
#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

class CommManager;
class PlayerInterface;
class Comms;
class PlayGround;

#include "comms.h"
#include "interfaceprovider.h"
#include "playerinterface.h"
#include "robot.h"
#include "statusbar.h"
#include "playground.h"

class CommManager: public Comms, public MapProvider, public LaserProvider, public SpeedProvider ,public LocationProvider
{
    Q_OBJECT
    public:
        CommManager(Robot *,PlayGround * playG);
        ~CommManager(); 
        virtual int readConfigs(ConfigFile *cf,int secId);
        virtual int start(); 
        virtual int stop();
        virtual LaserScan getLaserScan();
        virtual double getSpeed(); 
        virtual double getTurnRate();
        virtual Pose   getLocation();
        virtual Pose   getOdomLocation();
        virtual void   gotoGoal(Pose);
        virtual void   vfhGoto(Pose);
        virtual void   setSpeed(double speed);
        virtual void   setPtz(double pan, double tilt);
        virtual void   setTurnRate(double turnRate); 
	    virtual Map    provideMap(); 
	    virtual void   provideSpeed(double &speed, double &turnRate);
	    virtual void   provideLocation(Pose & location);
	    virtual bool   getLocalized();
		virtual QVector<DeviceType> * getDevices(QString host,int port );
    public slots: 
        virtual void setSpeed(double speed, double turnRate); 
        virtual void setLocation(Pose location);
        virtual void emergencyStop();
        virtual void emergencyRelease();
    signals:
	    void dataUpdated();
	    void addMsg(int,int, QString);
    protected:
        // Player stuff 
        PlayerInterface *player;
       	QVector <Laser> lasers;
        Robot * robot;
       	PlayGround *playGround;
};
#endif

