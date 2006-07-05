#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include "PathPlanner.h"
#include "configfile.h"
#include "MapManager.h"
#include "utils.h"
#include <QObject>
#include <QString>
#include <QPointF>

using namespace CasPlanner;
class PlanningManager : public MapManager
{
        Q_OBJECT
        public:
            PlanningManager();
            PlanningManager(double,double,QString,QPointF,double,double,double,double,double,double,double);
            ~PlanningManager();
            virtual int config(ConfigFile *cf, int sectionid);
            virtual int start(); 
            virtual int stop();
           	PathPlanner * pathPlanner;
        public slots: 
	        Node * FindPath(Pose start,Pose end);
	        void GenerateSpace();
	        void SetMap(QImage mpa);
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
		    //void dataUpdated();
		    //void statusMsg(int,int, QString); 
        protected:
            double pixel_res,bridge_len,bridge_res,reg_grid,obst_exp,conn_rad,obst_pen,
            	   robot_length, robot_width;
            bool bridgeTestEnabled,connNodesEnabled,regGridEnabled,obstPenEnabled,expObstEnabled,showTreeEnabled;
            QPointF rotation_center;
            QString robot_model;
};
#endif /*PLANNINGMANAGER_H_*/
