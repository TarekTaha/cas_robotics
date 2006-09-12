#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include "PathPlanner.h"
#include "configfile.h"
#include "mapmanager.h"
#include "robotmanager.h"
#include "utils.h"
#include <QObject>
#include <QString>
#include <QPointF>
#include <sys/types.h>
#include <sys/stat.h>

class RobotManager;
using namespace CasPlanner;
class PlanningManager : public MapManager
{
        Q_OBJECT
        public:
            PlanningManager(RobotManager *);
            PlanningManager(RobotManager *,double,double,double,double,double,double,double,double);
            ~PlanningManager();
            void setRobotManager(RobotManager *);
            int readConfigs(ConfigFile *cf);
            int setupPlanner(); 
            int stop();
           	PathPlanner * pathPlanner;
        public slots: 
	        Node* findPath(int coord);
	        void  generateSpace();
	        void  setStart(Pose);
	        void  setEnd(Pose);
	        void  setMap(QImage mpa);
	        void  setMap(LaserScan laserScan,double local_dist,Pose pose);
	        void  setBridgeTest(int);
	        void  setConnNodes(int);
	        void  setRegGrid(int);
	        void  setObstPen(int);
	        void  setExpObst(int);
	        void  setShowTree(int);
	        void  setBridgeTestValue(double);
	        void  setConnNodesValue(double);
	        void  setRegGridValue(double);
	        void  setObstPenValue(double);
	        void  setExpObstValue(double);
			void  setBridgeResValue(double val);
			bool fileExist(const char * fname);
       signals:
		    void statusMsg(int,int, QString);
        protected:
            double pixel_res,dist_goal,bridge_len,bridge_res,reg_grid,obst_exp,conn_rad,obst_pen;
            RobotManager *robotManager;
            bool bridgeTestEnabled,connNodesEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,showTreeEnabled,negate;
            QPointF rotation_center;
            QString robot_model;
            Pose start,end;
};
#endif /*PLANNINGMANAGER_H_*/
