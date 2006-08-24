#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include "PathPlanner.h"
#include "configfile.h"
#include "MapManager.h"
#include "robotmanager.h"
#include "utils.h"
#include <QObject>
#include <QString>
#include <QPointF>

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
            virtual int readConfigs(ConfigFile *cf);
            virtual int start(); 
            virtual int stop();
           	PathPlanner * pathPlanner;
        public slots: 
	        Node * FindPath(Pose start,Pose end);
	        void GenerateSpace();
	        void SetMap(QImage mpa);
	        void SetMap(QVector<QPointF> laser_scan,double local_dist,Pose pose);
	        void setBridgeTest(int);
	        void setConnNodes(int);
	        void setRegGrid(int);
	        void setObstPen(int);
	        void setExpObst(int);
	        void setShowTree(int);
	        void setBridgeTestValue(double);
	        void setConnNodesValue(double);
	        void setRegGridValue(double);
	        void setObstPenValue(double);
	        void setExpObstValue(double);
			void setBridgeResValue(double val);
        signals:
		    //void statusMsg(int,int, QString);
		    void rePaint(Map *);
        protected:
            double pixel_res,dist_goal,bridge_len,bridge_res,reg_grid,obst_exp,conn_rad,obst_pen;
            RobotManager *robotManager;
            bool bridgeTestEnabled,connNodesEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,showTreeEnabled,negate;
            QPointF rotation_center;
            QString robot_model;
};
#endif /*PLANNINGMANAGER_H_*/
