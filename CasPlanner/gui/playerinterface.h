#ifndef PLAYERINTERFACE_H
#define PLAYERINTERFACE_H
#include <QThread> 
#include "playerinterface.h"
#include "CommManager.h"
#include <playerclient.h>
#include <QReadWriteLock>
#include <QTime>

class PlayerInterface: public QThread 
{
Q_OBJECT    
    public:
        static const int MAX_LASERS = 2;
        static const int MAX_MOTORS = 3; 
        PlayerInterface(CommManager *comms, QString playerHost, int playerPort); 
        void stop();
        void run(); 
        void enableDrive(int driveId);
        void enableLaser(int whichLaser, int playerId); 
		void enablePtz(int ptzId);
        QVector<QPointF> getLaserScan(int laserId);
		void setPtz(double pan, double tilt);
        double getSpeed(); 
        double getTurnRate(); 
        void setSpeed(double speed);
        void setTurnRate(double turnRate); 
        void setSpeed(double speed, double turnRate); 
        void emergencyStop();
        void emergencyRelease();
    signals:
        void newData(); 
    private:
        QString playerHost; 
        int playerPort; 
        PlayerClient *pc;
        CommManager *comms; 
        
        bool laserEnabled[MAX_LASERS],ptzEnabled,driveEnabled,tiltEnabled; 
        int playerLaserId[MAX_LASERS],driveId,tiltId,ptzId; 
        LaserProxy *laser[MAX_LASERS]; 
        PositionProxy *drive;	
		PtzProxy *ptz;
		double pan;
		double tilt;
       
        double speed; 
        double turnRate; 
        
        //Emerg 
        bool emergencyStopped; 
        QReadWriteLock dataLock;       
};
#endif 
