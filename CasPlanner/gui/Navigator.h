#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <QObject>
#include <QString>
#include <QPointF>

#include <QReadWriteLock>
#include <QTime>
#include "robotmanager.h"
#include "planningmanager.h"
#include "Controller.h"
#include "configfile.h"
#include "Node.h"

class PlanningManager;
using namespace CasPlanner;

class Navigator : public Controller
{
	Q_OBJECT
	public:
		Navigator(RobotManager *);
		~Navigator();	
		QString obst_avoid;
		Node * globalPath, *localPath;
		int readConfigs(ConfigFile *cf);
		void setPath(Node *path);
		double NearestObstacle(QVector<QPointF> laser_scan,Pose pose);
		Node * FindClosest(QPointF location,Node * all_path);	
        void stop();
        void run();
        void setupLocalPlanner();
        RobotManager *robotManager;
	public
	slots:
		void FollowPath();
		void StopNavigating();
	signals:
		void drawLocalPath(PathPlanner *,Pose *,int *);
	protected:
		Pose	old_amcl,amcl_location,EstimatedPos;
		double 	angle,prev_angle,theta,error_orientation,
				displacement,wdem,distance,distance_to_next,
				minR,minL,minAhead,avoidance_distance_left,
				avoidance_distance_right,avoidance_distance_ahead,speed,estimate_x,
				estimate_y,estimate_theta,velocity,delta_t,last_time;
		double robot_length, robot_width,pixel_res,dist_goal,local_dist,traversable_dist,
			   bridge_len, bridge_res, reg_grid, obst_exp, conn_rad, obst_pen;
		QString robot_model;
		QPointF rotation_center;
		QPointF	begin,tracking_point,ni,SegmentStart,SegmentEnd;
		bool	log,position_found,end_reached,segment_navigated,stop_navigating;
		int		platform,direction,path2Draw;					
		Node * local_path,* global_path,*last,*first,*path2Follow;
		PlanningManager *local_planner;
		FILE * file;
};

#endif /*NAVIGATOR_H_*/
