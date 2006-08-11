#ifndef PLAYERINTERFACE_H
#define PLAYERINTERFACE_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>

#include "playerinterface.h"
#include "utils.h"
#include "map.h"
#define MAX_LASERS 4

using namespace PlayerCc;
class PlayerInterface: public QThread 
{
Q_OBJECT    
    public:
        PlayerInterface(QString playerHost, int playerPort);
        void stop();
        void run();
        void enableControl(int driveId);
        void setLasers(QVector<int>laserIds); 
		void enablePtz(int ptzId);
		void enableMap(int mapId);
		void enableLocalizer(int localizerId);
        QVector<QPointF> getLaserScan();
        void provideLocation(Pose location);
	    Map provideMap(); 
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate();
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
     
        bool ptzEnabled,ctrEnabled,mapEnabled,localizerEnabled,localized,emergencyStopped; 
        int positionId,ptzId,mapId,localizerId;
        QVector <int> laserIds;
        QVector <LaserProxy *> laser;
        Position2dProxy *drive;
        MapProxy *map;
		PtzProxy *ptz;
		LocalizeProxy *localizer;
		Pose location;
		double pan,tilt,pose[3], pose_covar[3];
        QVector<QPointF> laserScanPoints;
        double speed,turnRate,getspeed,getturnrate;
    
        QReadWriteLock dataLock;       
};
#endif 
