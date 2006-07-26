#ifndef PLAYERINTERFACE_H
#define PLAYERINTERFACE_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>

//#include <playerclient.h>
#include "playerinterface.h"
#include "utils.h"
#include "map.h"

using namespace PlayerCc;
class PlayerInterface: public QThread 
{
Q_OBJECT    
    public:
        static const int MAX_LASERS = 2;
        static const int MAX_MOTORS = 3;
        PlayerInterface(QString playerHost, int playerPort);
        void stop();
        void run();
        void enableControl(int driveId);
        void enableLaser(int whichLaser, int playerId); 
		void enablePtz(int ptzId);
		void enableMap(int mapId);
		void enableLocalizer(int localizerId);
        QVector<QPointF> getLaserScan(int laserId);
        void provideLocation(Pose location);
	    Map provideMap(); 
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate();
        double getClosestObst();
        bool getLocalized();
        Pose getLocation(); 
        void setSpeed(double speed);
        void setTurnRate(double turnRate); 
        void setSpeed(double speed, double turnRate); 
        void setLocation(Pose location);
        void emergencyStop();
        void emergencyRelease();
    signals:
        void newData(); 
    private:
        QString playerHost; 
        int playerPort; 
        PlayerClient *pc;
     
        bool laserEnabled[MAX_LASERS],ptzEnabled,ctrEnabled,mapEnabled,localizerEnabled,localized,emergencyStopped; 
        int playerLaserId[MAX_LASERS],positionId,ptzId,mapId,localizerId;
        LaserProxy *laser[MAX_LASERS]; 
        Position2dProxy *drive;
        MapProxy *map;
		PtzProxy *ptz;
		LocalizeProxy *localizer;
		Pose location;
		double pan,tilt,pose[3], pose_covar[3];
       
        double speed,turnRate,getspeed,getturnrate;
    
        QReadWriteLock dataLock;       
};
#endif 
