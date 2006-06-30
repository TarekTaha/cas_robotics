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
        //Q_OBJECT
        public:
            PlanningManager();
            ~PlanningManager();
            virtual int config(ConfigFile *cf, int sectionid);
            virtual int start(); 
            virtual int stop();
            Node * FindPath(QImage map,Pose start,Pose end);
        public slots: 
            //virtual void setSpeed(double speed, double turnRate); 
            //virtual void emergencyStop();
            //virtual void emergencyRelease();
        signals:
		    //void dataUpdated();
		    //void statusMsg(int,int, QString); 
        protected:
        	PathPlanner * planner;
            double pixel_res,bridge_len,bridge_res,reg_grid,obst_exp,conn_rad,obst_pen,
            	   robot_length, robot_width; 
            QPointF rotation_center;
            QString robot_model;
};
#endif /*PLANNINGMANAGER_H_*/
