#ifndef PATHFOLLOWER_H_
#define PATHFOLLOWER_H_
#include<PathPlanner.h>
// Player includes
#include <player/drivertable.h>
#include <player/driver.h>
#include <player.h>
#include <assert.h>
#include "devicetable.h"
#include "error.h"
#include "playertime.h"
#include<common.h>
#include<wheelchairproxy.h>
// end Player Includes
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtkmain.h>
#include<gtk/gtk.h>
#include<glib/gprintf.h>
#include<Vector2D.h>
namespace CasPlanner
{
enum {WHEELCHAIR,STAGE};
typedef struct _control_action
	{
		double linear_velocity;
		double angular_velocity;
	} ControlAction;
class PathFollower
{
	public :
		Node 	*path,*last,*first;
		GdkPixbuf *pixbuf;
		PathPlanner * Planner;
		double 	kd,kt,ko,tracking_distance,angle,prev_angle,theta,error_orientation,
				displacement,wdem,distance,distance_to_next,obstacle_avoidance_force,
				pose[3], pose_var[3][3],minR,minL,minAhead,avoidance_distance_left,
				avoidance_distance_right,avoidance_distance_ahead,speed,estimate_x,
				estimate_y,estimate_theta,velocity,delta_t,last_time;;
		bool	log,position_found,end_reached,segment_navigated;
		int		platform,direction;
		Point 	old_amcl,begin,amcl_location,tracking_point,ni,SegmentStart,SegmentEnd,
				EstimatedPos;
		GtkWidget 		*widget;
		GTimer *timer2,*delta_timer;
		PlayerClient 	*robot;
		WheelChairProxy *WCp;
		PositionProxy 	*pp;
		LaserProxy 		*laser;
		LocalizeProxy 	*localizer;
		FILE 			* file;
	public:
		bool stop;
		PathFollower(); // Start with no initialization
		PathFollower(Node * p,double kd, double kt,double ko,double tracking_distance,GtkWidget *widget,bool log,PathPlanner *Planner,GdkPixbuf *pixbuf);
		~PathFollower(); 
		void Connect(int);
		void RenderGui(Point,double);
		void FollowPath(Node *path);
		void AddText( char const * text);
		void ResetWheelchair();
		void Localize();
		double Magnitude( Point p1, Point p2 );
		double DistanceToLine(Point LineStart, Point LineEnd, Point P);
		double DistToLineSegment(Point LineStart, Point LineEnd, Point p);
		ControlAction Controller(double angle_ref,double angle_currnet,double displacement,int direction);
};

}

#endif /*PATHFOLLOWER_H_*/
