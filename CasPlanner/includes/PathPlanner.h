#ifndef PATHPLANNER_H_
#define PATHPLANNER_H_
#include <defs.h>
#include <Robot.h>
#include <Point.h>
#include <LList.h>
#include <Map.h>
#include <SearchSpaceNode.h>
#include <math.h>
#include <player/driver.h>
#define FORWARD 1
#define BACKWARD -1
#include <gdk-pixbuf/gdk-pixbuf.h>
#include <gtk/gtkmain.h>
#include<gtk/gtk.h>
#include<glib/gprintf.h>
namespace CasPlanner
{
typedef struct _tree
	{
		Point location;
		vector <Point> children;
	}Tree;
class PathPlanner 
	{
	public :
		Robot * wheelchair; // Location of the wheelchair's points to check in the wheelchair coordinate system
		int number_of_point_to_check,map_height,map_width,MAXNODES;
		double pixel_size,initial_angle,obstacle_radius,node_gap,bridge_length,final_angle;
		bool simulate;
		Point start, end, * points_to_check,local_edge_points[4],translated_edge_points[4];
		const char * MapFilename;
		GtkWidget * widget;
		GdkPixbuf * pixbuf;
		MapInfo mapinfo;	
		Node * test,*root,*path, *p;
		SearchSpaceNode * search_space,* temp;
		Node *current, *childList, *curChild, *q;
		LList *openList,*closedList;
		//list <Node> openList,closedList;
		Map * map;
		//PathFollower follower;
		vector <Tree> tree;
	public :
		double Calculate_g   (Node *); // Distance from Start
		double Calculate_h   (Node *); // Herustic Distance to Goal
		double anglediff(double alfa, double beta);
		bool   GoalReached   (Node *); 
		Node  *MakeChildrenNodes(Node *parent) ;
		void   FreeNode      (Node *);
		void   PrintNodeList ();
		int    StartSearch   (Point start, Point end,double initial_angle,double final_angle);
		void   Translate(Point  , double);
		void   Translate_edges(Point  , double);
		int    check_line (Point start,Point end);
		int    Obstacle   (Point p, double angle);
		void   ConvertPixel   (Point *p);
		void   ConvertToPixel (Point *p);
		void   ReadMap(); // Reads the map file and sets the attributes
		void   ExpandObstacles();
		void   AddCostToNodes(double distance);
		void   BridgeTest(double length,double gap);
		bool   CheckShortestDistance(double i,double j,double neigbhour_pixel_distance);
		void   GenerateRegularGrid(double gap);
		void   ConnectNodes(double distance);
		void   ShowConnections();
		void   SaveSearchSpace();
		void   DetermineCheckPoints();
		void   AddText ( char const * text);
		void   FindRoot();
		void   draw_path();
		void   draw_tree();
		void   Print();
		void   FreePath();
		void   PathFollow(Node *,double kd, double kt,double ko,double tracking_distance);
		PathPlanner(Point start,Point end,double initial_angle,double final_angle,double pixel_size,double radius,GtkWidget*,const char * MapFilename); 
		PathPlanner();
		~PathPlanner();
	};

}

#endif /*PATHPLANNER_H_*/
