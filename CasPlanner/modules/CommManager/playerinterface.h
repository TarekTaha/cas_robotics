#ifndef PLAYERINTERFACE_H
#define PLAYERINTERFACE_H

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>
#include <libplayerc/playerc.h>

#include <QThread> 
#include <QReadWriteLock>
#include <QTime>
#include <iostream>

#include "playerinterface.h"
#include "utils.h"
#include "map.h"
#include "timer.h"

#define MAX_LASERS 4

using namespace PlayerCc;
using namespace std;

// Callback prototypes
typedef void (*fndestroy_t) (void*);
typedef void (*fnupdate_t) (void*);

// Somewhere to store which devices are available.
typedef struct
{
  // Device identifier.
  player_devaddr_t addr;

  // Driver name
  char *drivername;

  // Handle to the GUI proxy for this device.
  void *proxy;

  // Callbacks
  fndestroy_t fndestroy;
  fnupdate_t fnupdate;

  // Non-zero if should be subscribed.
  int subscribe;

} device_t;

class Laser
{
	public: 
		LaserProxy * lp;
		int index;
		Pose pose;
		~Laser(){};
};
class LaserScan
{
	public:
		QVector<QPointF> points;
		Pose laserPose;
		LaserScan(){};		
		~LaserScan(){};
};
class PlayerInterface: public QThread 
{
Q_OBJECT    
    public:
        PlayerInterface(QString playerHost, int playerPort);
        void stop();
        void run();
        void enableControl(int driveId);
        void setLasers(QVector<Laser> lasers); 
		void enablePtz(int ptzId);
		void enableVfh(int vfhId);
		void enableMap(int mapId);
		void enableLocalizer(int localizerId);
        LaserScan getLaserScan();
        void provideLocation(Pose location);
	    Map provideMap(); 
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate();
        bool getLocalized();
        Pose getLocation();
        Pose getOdomLocation();
        void listDevices();
        void gotoGoal(Pose);
        void vfhGoto(Pose);
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
       	playerc_client_t *client;
	    device_t devices[PLAYER_MAX_DEVICES];
	  	device_t *device;
	  	void *proxy;       	
        bool ptzEnabled,ctrEnabled,mapEnabled,localizerEnabled,localized,emergencyStopped,
        	 velControl,vfhEnabled; 
        int positionId,ptzId,mapId,localizerId,vfhId;
        QVector <Laser> lasers;
        Position2dProxy *drive, *vfh;
        MapProxy *map;
		PtzProxy *ptz;
		LocalizeProxy *localizer;
		Pose location,goal,odom_location,vfhGoal;
		double pan,tilt,pose[3], pose_covar[3];
        LaserScan laserScan;
        double speed,turnRate,getspeed,getturnrate;
        QReadWriteLock dataLock;       
};
#endif 
