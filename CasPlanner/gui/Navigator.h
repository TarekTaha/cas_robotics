#ifndef NAVIGATOR_H_
#define NAVIGATOR_H_

#include <libplayerc++/playerc++.h>
#include <libplayercore/player.h>

#include <QObject>
#include <QString>
#include <QPointF>

#include <QReadWriteLock>
#include <QTime>
#include <vector>
#include "robotmanager.h"
#include "planningmanager.h"
#include "Controller.h"
#include "configfile.h"
#include "Node.h"
#include "icp.h"

class PlanningManager;
using namespace CasPlanner;
//using namespace Geom2D;

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
		void StopNavigating();
		double NearestObstacle(QVector<QPointF> laser_scan,Pose pose);
		void  setPause(bool pause);
		Node * ClosestPathSeg(QPointF location,Node * all_path);	
		void GenerateLocalMap(QVector<QPointF> laser_scan,Pose laser_pose, Pose rob_location);
		bool MapModified(QVector<QPointF> laser_scan,Pose rob_location);
        void stop();
        void run();
        void setupLocalPlanner();
        RobotManager *robotManager;
	public
	slots:
		void FollowPath();
	signals:
		void drawLocalPath(PathPlanner *,Pose *,int *);
	protected:
		Pose	old_amcl,amcl_location,EstimatedPos,laser_pose;
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
		bool	log,position_found,end_reached,segment_navigated,stop_navigating,pause;
		int		platform,direction,path2Draw;					
		Node * local_path,* global_path,*last,*first,*path2Follow;
		PlanningManager *local_planner,*global_planner;
		Geom2D::ICP icp;
		std::vector<Geom2D::Point> local_map_icp,laser_scan_icp;
		Geom2D::Pose	delta_pose;		
		QVector<QPointF> local_map;
		FILE * file;
};

#endif /*NAVIGATOR_H_*/
