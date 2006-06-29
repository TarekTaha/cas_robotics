#ifndef PLANNINGMANAGER_H_
#define PLANNINGMANAGER_H_

#include "PathPlanner.h"
#include <QString>

using namespace CasPlanner;
class PlanningManager
{
        Q_OBJECT
        public:
            PlanningManager();
            ~PlanningManager(); 
            virtual int config(ConfigFile *cf, int sectionid);
            virtual int start(); 
            virtual int stop();
        public slots: 
            virtual void setSpeed(double speed, double turnRate); 
            virtual void emergencyStop();
            virtual void emergencyRelease();
        signals:
		    void dataUpdated();
		    void statusMsg(int,int, QString); 
        protected:
        	PathPlanner * planner;
            double pixel_res,bridge_len,bridge_res,reg_grid,obst_exp,conn_rad,obst_pen
            	   robot_length, robot_height; 
            QString robot_model;
};
#endif /*PLANNINGMANAGER_H_*/
