#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include "pathplanner.h"
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
class PlanningManager : public QObject
{
        Q_OBJECT
        public:
            PlanningManager(RobotManager *);
            PlanningManager(RobotManager *,double,double,double,double,double,double,double,double,double);
            ~PlanningManager();
            void setRobotManager(RobotManager *);
            int readConfigs(ConfigFile *cf);
            int setupPlanner();
            int stop();
           	PathPlanner * pathPlanner;
           	bool renderTree;
        public slots:
	        Node* findPath(int coord);
	        void  generateSpace();
	        void  setStart(Pose);
	        void  setEnd(Pose);
	        void  setMap(Map *);
//	        void  setMap(LaserScan laserScan,double local_dist,Pose pose);
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
//			void  updateMap(LaserScan laserScan,double local_dist,Pose robotLocation);
			bool  fileExist(const char * fname);
       signals:
		    void addMsg(int,int, QString);
		    void updateMap(Map* map);
        protected:
            double pixel_res,dist_goal,bridge_len,bridge_res,reg_grid,obst_exp,reg_grid_conn_rad,obst_pen,bridge_conn_rad;
            RobotManager *robotManager;
            bool bridgeTestEnabled,connNodesEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,showTreeEnabled,negate;
            QPointF rotation_center;
            QString robot_model;
            Pose start,end;
};
#endif /*PLANNINGMANAGER_H_*/
